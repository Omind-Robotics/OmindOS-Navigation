#ifndef NMPC_PLANNER_MPC_SOLVER_HPP
#define NMPC_PLANNER_MPC_SOLVER_HPP

#include <vector>
#include <array>
#include <memory>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

namespace nmpc_planner {

class MPCSolver {
public:
    // 数据结构定义
    struct State {
        double x;
        double y;
        double theta;
        
        State() : x(0.0), y(0.0), theta(0.0) {}
        State(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
    };
    
    struct Control {
        double v;      // 线速度
        double omega;  // 角速度
        
        Control() : v(0.0), omega(0.0) {}
        Control(double v_, double omega_) : v(v_), omega(omega_) {}
    };
    
    struct Obstacle {
        double x, y, radius;
        Obstacle() : x(0.0), y(0.0), radius(0.15) {}  // 默认构造函数
        Obstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}
    };
    
    struct MPCResult {
        std::vector<State> trajectory;
        std::vector<Control> controls;
        bool success;
        
        MPCResult() : success(false) {}
    };
    
    // MPC参数结构
    struct MPCParams {
        double T;                    // 控制周期
        int N;                      // 预测步数
        double v_max;               // 最大线速度
        double omega_max;           // 最大角速度
        Eigen::Matrix3d Q;          // 状态权重矩阵
        Eigen::Matrix2d R;          // 控制权重矩阵
        Eigen::Matrix2d S;          // 控制变化率权重矩阵（平滑项）

        // 障碍物避障参数（改进TEB形式）
        double safe_distance;           // 安全距离（必须保持的最小距离）
        double influence_distance;      // 影响距离（开始产生引导力的距离）
        double obstacle_weight;         // 障碍物惩罚权重
        bool use_time_weight;           // 是否使用时间加权（前期步骤权重更大）
        int max_obstacles_consider;     // 最多考虑的障碍物数量
        double obstacle_influence_range;// 障碍物影响范围（米）
        double smooth_epsilon;          // 平滑max函数的epsilon参数
        
        MPCParams() {
            T = 0.5;
            N = 10;
            v_max = 0.5;
            omega_max = 1.0;

            Q = Eigen::Matrix3d::Zero();
            Q(0,0) = 1.2; Q(1,1) = 1.2; Q(2,2) = 0.0;

            R = Eigen::Matrix2d::Zero();
            R(0,0) = 0.2; R(1,1) = 0.15;

            S = Eigen::Matrix2d::Zero();
            S(0,0) = 0.3; S(1,1) = 0.5;  // 控制平滑权重：角速度变化惩罚更大

            // 障碍物避障默认参数（改进TEB形式）
            safe_distance = 0.4;             // 安全距离：必须保持0.4m以上距离
            influence_distance = 1.2;        // 影响距离：1.2m内开始产生引导力
            obstacle_weight = 8.0;           // 障碍物惩罚权重
            use_time_weight = true;          // 使用时间加权
            max_obstacles_consider = 10;     // 最多考虑10个障碍物
            obstacle_influence_range = 5.0;  // 只考虑5m内的障碍物
            smooth_epsilon = 0.01;           // 平滑max的epsilon参数
        }
    };

public:
    MPCSolver();
    explicit MPCSolver(const MPCParams& params);
    ~MPCSolver();
    
    // 主要求解接口
    MPCResult solve(const State& current_state, 
                   const std::vector<State>& goal_trajectory,
                   const std::vector<Obstacle>& obstacles = {});
    
    // 参数设置
    void setParams(const MPCParams& params);
    const MPCParams& getParams() const { return params_; }
    
    // 实用函数
    static double normalizeAngle(double angle);

private:
    MPCParams params_;
    
    // CasADi相关成员
    std::unique_ptr<casadi::Opti> opti_;
    
    // 初始化优化器
    void initializeOptimizer();
    
    // 辅助函数
    casadi::MX createStateTransitionModel(const casadi::MX& state, 
                                         const casadi::MX& control,
                                         const State& current_velocity);
};

} // namespace nmpc_planner

#endif // NMPC_PLANNER_MPC_SOLVER_HPP