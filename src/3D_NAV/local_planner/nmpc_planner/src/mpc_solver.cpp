#include "nmpc_planner/mpc_solver.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>

namespace nmpc_planner {

MPCSolver::MPCSolver() : MPCSolver(MPCParams()) {}

MPCSolver::MPCSolver(const MPCParams& params) : params_(params) {
    initializeOptimizer();
}

MPCSolver::~MPCSolver() = default;

void MPCSolver::initializeOptimizer() {
    opti_ = std::make_unique<casadi::Opti>();
}

MPCSolver::MPCResult MPCSolver::solve(const State& current_state, 
                                     const std::vector<State>& goal_trajectory,
                                     const std::vector<Obstacle>& obstacles) {
    MPCResult result;
    
    try {
        // 重新初始化优化器
        opti_ = std::make_unique<casadi::Opti>();
        
        // 检查目标轨迹长度（移除这个严格检查，因为LocalPlanner会处理长度不足的情况）
        if (goal_trajectory.empty()) {
            std::cerr << "Goal trajectory is empty" << std::endl;
            return result;
        }
        
        // 定义优化变量
        auto opt_x0 = opti_->parameter(3);  // 初始状态
        auto opt_controls = opti_->variable(params_.N, 2);  // 控制量
        auto opt_states = opti_->variable(params_.N + 1, 3);  // 状态量
        
        auto v = opt_controls(casadi::Slice(0, params_.N), 0);      // 线速度
        auto omega = opt_controls(casadi::Slice(0, params_.N), 1);  // 角速度
        
        auto x = opt_states(casadi::Slice(0, params_.N + 1), 0);    // x坐标
        auto y = opt_states(casadi::Slice(0, params_.N + 1), 1);    // y坐标 
        auto theta = opt_states(casadi::Slice(0, params_.N + 1), 2); // 航向角
        
        // 初始状态约束
        std::vector<double> init_state = {current_state.x, current_state.y, current_state.theta};
        opti_->subject_to(opt_states(0, casadi::Slice()) == opt_x0.T());
        
        // 控制约束
        opti_->subject_to(opti_->bounded(0, v, params_.v_max));
        opti_->subject_to(opti_->bounded(-params_.omega_max, omega, params_.omega_max));
        
        // 系统动力学约束
        for (int i = 0; i < params_.N; ++i) {
            auto state_i = opt_states(i, casadi::Slice());
            auto control_i = opt_controls(i, casadi::Slice());
            
            // 简化的运动学模型
            auto x_dot = control_i(0) * casadi::MX::cos(state_i(2));
            auto y_dot = control_i(0) * casadi::MX::sin(state_i(2));
            auto theta_dot = control_i(1);
            
            // 确保维度匹配：state_i是1x3，状态导数也应该是1x3
            auto state_derivative = casadi::MX::horzcat({x_dot, y_dot, theta_dot});
            auto state_next = state_i + params_.T * state_derivative;
            opti_->subject_to(opt_states(i + 1, casadi::Slice()) == state_next);
        }
        
        // 目标函数
        casadi::MX obj = 0;
        
        // 状态跟踪代价
        for (int i = 0; i < params_.N; ++i) {
            // 确保维度匹配：使用horzcat创建1x3的目标状态
            auto goal_state = casadi::MX::horzcat({goal_trajectory[i].x, 
                                                  goal_trajectory[i].y, 
                                                  goal_trajectory[i].theta});
            auto state_error = opt_states(i, casadi::Slice()) - goal_state;
            
            // Q矩阵的对角线权重
            auto state_cost = params_.Q(0,0) * casadi::MX::pow(state_error(0), 2) +
                             params_.Q(1,1) * casadi::MX::pow(state_error(1), 2) +
                             params_.Q(2,2) * casadi::MX::pow(state_error(2), 2);
            
            obj += 0.1 * state_cost;
        }
        
        // 控制输入代价
        for (int i = 0; i < params_.N; ++i) {
            auto control_cost = params_.R(0,0) * casadi::MX::pow(opt_controls(i, 0), 2) +
                               params_.R(1,1) * casadi::MX::pow(opt_controls(i, 1), 2);
            obj += control_cost;
        }
        
        // 控制平滑项（控制变化率惩罚）- 减少急剧的控制变化
        for (int i = 1; i < params_.N; ++i) {
            // 计算相邻时刻的控制输入差异
            auto du = opt_controls(i, casadi::Slice()) - opt_controls(i-1, casadi::Slice());
            
            // 对控制变化率进行二次惩罚
            auto smooth_cost = params_.S(0,0) * casadi::MX::pow(du(0), 2) +  // 线速度变化惩罚
                              params_.S(1,1) * casadi::MX::pow(du(1), 2);     // 角速度变化惩罚
            obj += smooth_cost;
        }
        
        // 终端状态代价
        auto terminal_goal = casadi::MX::horzcat({goal_trajectory[params_.N - 1].x,
                                                 goal_trajectory[params_.N - 1].y,
                                                 goal_trajectory[params_.N - 1].theta});
        auto terminal_error = opt_states(params_.N - 1, casadi::Slice()) - terminal_goal;
        
        auto terminal_cost = params_.Q(0,0) * casadi::MX::pow(terminal_error(0), 2) +
                            params_.Q(1,1) * casadi::MX::pow(terminal_error(1), 2) +
                            params_.Q(2,2) * casadi::MX::pow(terminal_error(2), 2);
        
        obj += 2.0 * terminal_cost;
        
        // ========== 障碍物避障处理（改进TEB形式） ==========
        if (!obstacles.empty()) {
            // 1. 预处理：筛选有效障碍物（在影响范围内）
            std::vector<Obstacle> nearby_obstacles;
            for (const auto& obs : obstacles) {
                double dist_to_robot = std::sqrt(
                    std::pow(obs.x - current_state.x, 2) +
                    std::pow(obs.y - current_state.y, 2)
                );

                if (dist_to_robot < params_.obstacle_influence_range) {
                    nearby_obstacles.push_back(obs);
                }
            }

            // 2. 按距离排序，只考虑最近的几个障碍物
            std::sort(nearby_obstacles.begin(), nearby_obstacles.end(),
                [&current_state](const Obstacle& a, const Obstacle& b) {
                    double dist_a = std::pow(a.x - current_state.x, 2) +
                                   std::pow(a.y - current_state.y, 2);
                    double dist_b = std::pow(b.x - current_state.x, 2) +
                                   std::pow(b.y - current_state.y, 2);
                    return dist_a < dist_b;
                });

            // 限制考虑的障碍物数量
            if (nearby_obstacles.size() > static_cast<size_t>(params_.max_obstacles_consider)) {
                nearby_obstacles.resize(params_.max_obstacles_consider);
            }

            // 3. 对每个预测步骤和每个障碍物应用改进TEB约束
            for (int i = 0; i < params_.N; ++i) {
                for (const auto& obs : nearby_obstacles) {
                    // 计算到障碍物的欧式距离
                    auto dx = x(i) - obs.x;
                    auto dy = y(i) - obs.y;
                    auto dist_sq = dx * dx + dy * dy;
                    auto dist = casadi::MX::sqrt(dist_sq + 1e-6);  // 添加小值避免数值问题

                    // 定义安全距离和影响距离（包含障碍物半径）
                    double d_safe = params_.safe_distance + obs.radius;
                    double d_influence = params_.influence_distance + obs.radius;

                    // TEB风格的分段惩罚函数
                    // 计算违反量：violation = d_safe - dist
                    auto violation = d_safe - dist;

                    // 使用平滑max函数：smooth_max(0, violation)
                    // 采用平方根平滑形式，梯度最稳定
                    auto smooth_max_violation = 0.5 * (violation + casadi::MX::sqrt(
                        violation * violation + params_.smooth_epsilon));

                    // 计算时间权重（可选）
                    double time_weight = 1.0;
                    if (params_.use_time_weight) {
                        // 前1/3步骤：权重加倍，确保近期安全
                        time_weight = (i < params_.N / 3) ? 2.0 : 1.0;
                    }

                    // TEB核心约束：E_obs = w_obs * f_obs^2
                    // 当dist >= d_safe时，violation <= 0，smooth_max ≈ 0，无惩罚
                    // 当dist < d_safe时，violation > 0，产生二次惩罚
                    auto teb_penalty = casadi::MX::pow(smooth_max_violation, 2);
                    obj += time_weight * params_.obstacle_weight * teb_penalty;

                    // 额外的远距离引导项（在影响范围内但未违反安全距离）
                    // 只在 d_safe < dist < d_influence 时生效
                    if (d_influence > d_safe) {
                        // 计算影响因子：距离越近，影响越大
                        // influence_factor在[0,1]范围内，当dist=d_safe时为1，dist=d_influence时为0
                        auto influence_violation = d_influence - dist;
                        auto smooth_influence = 0.5 * (influence_violation + casadi::MX::sqrt(
                            influence_violation * influence_violation + params_.smooth_epsilon));

                        // 归一化到[0,1]
                        auto influence_factor = smooth_influence / (d_influence - d_safe);

                        // 温和的引导惩罚（权重为主惩罚的10%）
                        auto guidance_penalty = casadi::MX::pow(influence_factor, 2);
                        obj += time_weight * params_.obstacle_weight * 0.1 * guidance_penalty;
                    }
                }
            }
        }
        // ========== 障碍物避障处理结束 ==========
        
        // 设置目标函数
        opti_->minimize(obj);
        
        // 求解器设置。IPOPT runtime on this NUC does not match the local CasADi
        // plugin ABI, so use qrsqp from the same local CasADi tree.
        casadi::Dict opts;
        opts["print_time"] = false;
        opts["error_on_fail"] = false;
        opts["print_header"] = false;
        opts["print_iteration"] = false;
        casadi::Dict qpsol_opts;
        qpsol_opts["error_on_fail"] = false;
        qpsol_opts["print_header"] = false;
        qpsol_opts["print_iter"] = false;
        opts["qpsol_options"] = qpsol_opts;
        
        opti_->solver("qrsqp", opts);
        
        // 设置初始状态值
        opti_->set_value(opt_x0, casadi::DM(init_state));
        
        // 求解
        auto sol = opti_->solve();
        
        // 提取结果
        auto u_res = sol.value(opt_controls);
        auto state_res = sol.value(opt_states);
        
        // 转换为结果格式
        result.trajectory.reserve(params_.N + 1);
        result.controls.reserve(params_.N);
        
        for (int i = 0; i <= params_.N; ++i) {
            State state;
            state.x = static_cast<double>(state_res(i, 0));
            state.y = static_cast<double>(state_res(i, 1));
            state.theta = static_cast<double>(state_res(i, 2));
            result.trajectory.push_back(state);
        }
        
        for (int i = 0; i < params_.N; ++i) {
            Control control;
            control.v = static_cast<double>(u_res(i, 0));
            control.omega = static_cast<double>(u_res(i, 1));
            result.controls.push_back(control);
        }
        
        result.success = true;
        
    } catch (const std::exception& e) {
        std::cerr << "MPC solve failed: " << e.what() << std::endl;
        
        // 返回安全的零控制输入
        result.trajectory.clear();
        result.controls.clear();
        
        // 填充默认轨迹（当前状态重复）
        for (int i = 0; i <= params_.N; ++i) {
            result.trajectory.push_back(current_state);
        }
        
        // 填充零控制
        for (int i = 0; i < params_.N; ++i) {
            result.controls.push_back(Control(0.0, 0.0));
        }
        
        result.success = false;
    }
    
    return result;
}

void MPCSolver::setParams(const MPCParams& params) {
    params_ = params;
    initializeOptimizer();
}

double MPCSolver::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace nmpc_planner
