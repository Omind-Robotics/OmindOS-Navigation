#include "nmpc_planner/local_planner.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>

namespace nmpc_planner {

LocalPlanner::LocalPlanner()
    : nh_(), private_nh_("~"), ref_path_set_(false), state_received_(false),
      nav_state_(NavState::WAITING), goal_x_(0.0), goal_y_(0.0), goal_yaw_(0.0),
      goal_align_rotation_speed_(0.0), goal_align_direction_set_(false) {

    // 初始化参数
    initializeParameters();
    
    // 初始化ROS通信
    initializePublishersSubscribers();
    
    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化MPC求解器
    MPCSolver::MPCParams mpc_params;
    // 可以从ROS参数服务器加载MPC参数
    private_nh_.param<double>("mpc/control_horizon", mpc_params.T, 0.5);
    private_nh_.param<int>("mpc/prediction_steps", mpc_params.N, 10);
    private_nh_.param<double>("mpc/max_linear_vel", mpc_params.v_max, 0.5);
    private_nh_.param<double>("mpc/max_angular_vel", mpc_params.omega_max, 1.0);
    
    // 加载控制平滑参数
    double smooth_v_weight = 0.3;
    double smooth_omega_weight = 0.5;
    private_nh_.param<double>("mpc/smooth_v_weight", smooth_v_weight, 0.3);
    private_nh_.param<double>("mpc/smooth_omega_weight", smooth_omega_weight, 0.5);
    mpc_params.S = Eigen::Matrix2d::Zero();
    mpc_params.S(0,0) = smooth_v_weight;
    mpc_params.S(1,1) = smooth_omega_weight;
    
    // 加载障碍物避障参数（改进TEB形式）
    private_nh_.param<double>("mpc/safe_distance", mpc_params.safe_distance, 0.4);
    private_nh_.param<double>("mpc/influence_distance", mpc_params.influence_distance, 1.2);
    private_nh_.param<double>("mpc/obstacle_weight", mpc_params.obstacle_weight, 8.0);
    private_nh_.param<bool>("mpc/use_time_weight", mpc_params.use_time_weight, true);
    private_nh_.param<int>("mpc/max_obstacles_consider", mpc_params.max_obstacles_consider, 10);
    private_nh_.param<double>("mpc/obstacle_influence_range", mpc_params.obstacle_influence_range, 5.0);
    private_nh_.param<double>("mpc/smooth_epsilon", mpc_params.smooth_epsilon, 0.01);
    
    mpc_solver_ = std::make_unique<MPCSolver>(mpc_params);
    
    // 初始化数据
    current_state_.fill(0.0);
    
    // 规划定时器
    planning_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_), 
                                     &LocalPlanner::planningTimerCallback, this);
    
    ROS_INFO("Local Planner initialized");
}

LocalPlanner::~LocalPlanner() {
    shutdown();
}

void LocalPlanner::initialize() {
    ROS_INFO("Local Planner running...");
}

void LocalPlanner::shutdown() {
    if (planning_timer_.isValid()) {
        planning_timer_.stop();
    }
}

void LocalPlanner::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    if (msg->poses.empty()) {
        ROS_WARN("[Local Planner] Empty global path received");
        return;
    }
    
    if (processGlobalPathData(msg)) {
        ref_path_set_ = true;
        ROS_INFO_THROTTLE(5.0, "[Local Planner] Global path updated, %zu points", global_path_.size());
    }
}

void LocalPlanner::currentStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (msg->data.size() >= 4) {
        current_state_[0] = msg->data[0];  // x
        current_state_[1] = msg->data[1];  // y
        current_state_[2] = msg->data[2];  // z
        current_state_[3] = msg->data[3];  // yaw
        
        // 动态更新机器人高度
        robot_height_ = msg->data[2];
        
        state_received_ = true;
    }
}

void LocalPlanner::obstacleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);

    obstacles_.clear();

    if (!msg->data.empty() && msg->data.size() % 3 == 0) {
        size_t num_obstacles = msg->data.size() / 3;

        // 使用set来实现去重（基于网格化后的坐标）
        std::set<std::pair<double, double>> unique_grid_positions;

        for (size_t i = 0; i < num_obstacles; ++i) {
            // 获取障碍物坐标并进行网格化处理
            double x = msg->data[3 * i];
            double y = msg->data[3 * i + 1];
            // z坐标暂时不使用，但保留接口
            // double z = msg->data[3 * i + 2];

            // 网格化处理：将坐标对齐到网格
            double grid_x = std::floor(x / obstacle_grid_size_) * obstacle_grid_size_;
            double grid_y = std::floor(y / obstacle_grid_size_) * obstacle_grid_size_;

            // 添加到去重集合
            unique_grid_positions.insert({grid_x, grid_y});
        }

        // 将去重后的障碍物添加到列表中
        // 使用固定半径，可以后续从参数中调整
        double obstacle_radius = 0.15;
        for (const auto& pos : unique_grid_positions) {
            obstacles_.emplace_back(pos.first, pos.second, obstacle_radius);
        }

        ROS_DEBUG_THROTTLE(1.0, "[Local Planner] Received %zu obstacles, after gridding and dedup: %zu",
                          num_obstacles, obstacles_.size());
    } else if (!msg->data.empty()) {
        ROS_WARN("[Local Planner] Invalid obstacle data received (size not divisible by 3)");
    }
}

void LocalPlanner::navigationStateCallback(const std_msgs::UInt8::ConstPtr& msg) {
    // Convert uint8_t to NavState enum
    if (msg->data <= static_cast<uint8_t>(NavState::ABORTED)) {
        NavState new_state = static_cast<NavState>(msg->data);

        // 检测状态变化
        if (new_state != nav_state_) {
            ROS_INFO("[Local Planner] Navigation state changed: %d -> %d",
                     static_cast<int>(nav_state_), static_cast<int>(new_state));

            // 进入 COMPLETED 或 ABORTED 状态时清空缓存
            if (new_state == NavState::COMPLETED || new_state == NavState::ABORTED) {
                std::lock_guard<std::mutex> lock(path_mutex_);
                global_path_.clear();
                local_plan_.clear();
                ref_path_set_ = false;

                // 发布零控制指令
                publishZeroControl();

                ROS_INFO("[Local Planner] Path cache cleared, zero control published");
            }
        }

        nav_state_ = new_state;

        if (nav_state_ == NavState::GOAL_ALIGN) {
            updateGoalAlignDirection();
        } else {
            goal_align_direction_set_ = false;
            goal_align_rotation_speed_ = 0.0;
        }
    } else {
        ROS_WARN("[Local Planner] Received invalid navigation state: %d", msg->data);
    }
}

void LocalPlanner::planningTimerCallback(const ros::TimerEvent& event) {
    // 根据导航状态执行不同逻辑
    switch (nav_state_) {
        case NavState::WAITING:
        case NavState::GLOBAL_PLANNING:
            // 等待或规划阶段，不执行局部规划
            return;

        case NavState::TRACKING:
            // 正常路径跟踪，执行NMPC规划
            if (ref_path_set_ && state_received_) {
                performPlanning();
            }
            break;

        case NavState::GOAL_ALIGN:
            // 姿态对齐阶段，仅执行纯旋转控制
            if (state_received_) {
                performPlanning();  // 内部会调用buildAlignmentPath
            }
            break;

        case NavState::COMPLETED:
        case NavState::ABORTED:
            // 任务完成或中止，持续发布零控制
            publishZeroControl();
            break;
    }
}

void LocalPlanner::performPlanning() {
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    std::lock_guard<std::mutex> obstacle_lock(obstacle_mutex_);

    if (nav_state_ == NavState::GOAL_ALIGN) {
        if (!goal_align_direction_set_) {
            updateGoalAlignDirection();
        }

        double rotation_speed = goal_align_rotation_speed_;
        if (std::abs(rotation_speed) < 1e-6) {
            ROS_INFO_THROTTLE(2.0, "[GOAL_ALIGN] Rotation skipped (already aligned)");
        } else {
            ROS_INFO_THROTTLE(1.0, "[GOAL_ALIGN] Rotating with preset omega=%.2f rad/s", rotation_speed);
        }

        local_plan_.clear();
        for (int i = 0; i < desired_path_size_; ++i) {
            local_plan_.push_back({0.0, rotation_speed});
        }
        publishLocalPlan();
        return;
    }

    // 正常路径跟踪阶段
    if (global_path_.empty()) {
        // 发布空的local_plan
        local_plan_.clear();
        publishLocalPlan();
        return;
    }

    try {
        // 获取当前状态
        auto current_mpc_state = getCurrentMPCState();

        // 转换全局路径为MPC格式
        std::vector<MPCSolver::State> mpc_path = convertToMPCPath();
        
        if (mpc_path.empty()) {
            ROS_WARN("[Local Planner] Empty MPC path generated");
            local_plan_.clear();
            publishLocalPlan();
            return;
        }

        if (!mpc_solver_enabled_) {
            generateFallbackTrackingPlan(current_mpc_state, mpc_path);
            publishLocalPlan();
            return;
        }
        
        // 调用MPC求解器，传入障碍物数据
        auto mpc_result = mpc_solver_->solve(current_mpc_state, mpc_path, obstacles_);
        
        if (mpc_result.success) {
            // 转换MPC结果为local_plan格式
            convertMPCResultToLocalPlan(mpc_result);
            
            // 发布轨迹路径 (nav_msgs/Path)
            publishLocalPath(mpc_result);
        } else {
            if (fallback_tracking_enabled_) {
                ROS_WARN_THROTTLE(2.0, "[Local Planner] MPC solve failed, using fallback tracking controls");
                generateFallbackTrackingPlan(current_mpc_state, mpc_path);
            } else {
                ROS_WARN_THROTTLE(2.0, "[Local Planner] MPC solve failed, using zero controls");
                local_plan_.clear();
                for (int i = 0; i < desired_path_size_; ++i) {
                    local_plan_.push_back({0.0, 0.0});
                }
            }
        }
        
        // 发布控制指令结果
        publishLocalPlan();
        
    } catch (const std::exception& e) {
        ROS_ERROR("[Local Planner] Planning failed: %s", e.what());
        
        // 发布安全的停止指令
        local_plan_.clear();
        for (int i = 0; i < desired_path_size_; ++i) {
            local_plan_.push_back({0.0, 0.0});
        }
        publishLocalPlan();
    }
}

void LocalPlanner::publishLocalPlan() {
    std_msgs::Float32MultiArray msg;
    msg.data.reserve(local_plan_.size() * 2);
    
    for (const auto& point : local_plan_) {
        msg.data.push_back(static_cast<float>(point[0]));
        msg.data.push_back(static_cast<float>(point[1]));
    }
    
    local_plan_pub_.publish(msg);
}

void LocalPlanner::publishLocalPath(const MPCSolver::MPCResult& result) {
    nav_msgs::Path local_path;
    local_path.header.stamp = ros::Time::now();
    local_path.header.frame_id = map_frame_;
    
    // 参考Python版本的逻辑，发布MPC求解得到的状态轨迹
    for (size_t i = 0; i < result.trajectory.size(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        
        // 设置位置
        pose_stamped.pose.position.x = result.trajectory[i].x;
        pose_stamped.pose.position.y = result.trajectory[i].y;
        pose_stamped.pose.position.z = robot_height_;
        
        // 设置方向 (从theta角度创建四元数)
        double theta = result.trajectory[i].theta;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = sin(theta / 2.0);
        pose_stamped.pose.orientation.w = cos(theta / 2.0);
        
        // 设置header
        pose_stamped.header.seq = static_cast<uint32_t>(i);
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = map_frame_;
        
        local_path.poses.push_back(pose_stamped);
    }
    
    local_path_pub_.publish(local_path);
}

bool LocalPlanner::processGlobalPathData(const nav_msgs::Path::ConstPtr& msg) {
    global_path_.clear();

    // 参考path2array.py的处理逻辑：从索引0开始，步长为1
    int start_idx = std::min(10, static_cast<int>(msg->poses.size()) - 1);
    int step = 1;

    size_t num_points = 0;
    for (int i = start_idx; i < static_cast<int>(msg->poses.size()); i += step) {
        if (num_points >= static_cast<size_t>(path_buffer_limit_)) {
            ROS_WARN("[Local Planner] Path size exceeds buffer limit");
            break;
        }

        const auto& pose = msg->poses[i].pose;

        PathPoint point;
        point.x = pose.position.x;
        point.y = pose.position.y;
        // 保留全局规划器提供的姿态，不重新计算
        point.theta = quaternionToYaw(pose.orientation);
        point.confidence = 1.0;  // 默认置信度

        global_path_.push_back(point);
        num_points++;
    }

    // 缓存目标点信息（路径末端）用于 GOAL_ALIGN 阶段
    if (!global_path_.empty()) {
        const auto& goal_point = global_path_.back();
        goal_x_ = goal_point.x;
        goal_y_ = goal_point.y;
        goal_yaw_ = goal_point.theta;

        ROS_INFO_THROTTLE(5.0, "[Local Planner] Goal cached: pos=(%.2f, %.2f), yaw=%.2f deg",
                         goal_x_, goal_y_, goal_yaw_ * 180.0 / M_PI);
    }

    ROS_INFO_THROTTLE(2.0, "[Local Planner] Processed %zu path points from nav_msgs/Path", global_path_.size());
    return !global_path_.empty();
}

std::vector<MPCSolver::State> LocalPlanner::convertToMPCPath() {
    std::vector<MPCSolver::State> mpc_path;
    int required_points = mpc_solver_->getParams().N;
    
    if (global_path_.empty()) {
        return mpc_path;
    }
    
    // 如果路径点不足，通过重复最后一个点来扩展
    int available_points = static_cast<int>(global_path_.size());
    mpc_path.reserve(required_points);
    
    // 先添加所有可用的路径点
    for (int i = 0; i < std::min(available_points, required_points); ++i) {
        const auto& point = global_path_[i];
        mpc_path.emplace_back(point.x, point.y, point.theta);
    }
    
    // 如果路径点不足，用最后一个点填充剩余位置
    if (available_points < required_points) {
        const auto& last_point = global_path_.back();
        for (int i = available_points; i < required_points; ++i) {
            mpc_path.emplace_back(last_point.x, last_point.y, last_point.theta);
        }
        
        ROS_WARN_THROTTLE(2.0, "[Local Planner] Path too short (%d points), extended to %d points by repeating last point", 
                          available_points, required_points);
    }
    
    return mpc_path;
}

MPCSolver::State LocalPlanner::getCurrentMPCState() {
    return MPCSolver::State(current_state_[0], current_state_[1], current_state_[3]);
}

void LocalPlanner::convertMPCResultToLocalPlan(const MPCSolver::MPCResult& result) {
    local_plan_.clear();

    // 使用MPC的控制输入结果
    for (const auto& control : result.controls) {
        double v = control.v;
        double omega = control.omega;

        // 在 GOAL_ALIGN 状态下，对角速度进行阈值限制
        if (nav_state_ == NavState::GOAL_ALIGN) {
            // 线速度应该为0或接近0（位置已到达）
            v = 0.0;

            // 角速度：如果非零但小于硬件响应阈值，则设置为最小值（保持符号）
            if (std::abs(omega) > 1e-4 && std::abs(omega) < min_angular_vel_) {
                omega = (omega > 0) ? min_angular_vel_ : -min_angular_vel_;
                ROS_DEBUG_THROTTLE(1.0, "[Local Planner] Angular velocity boosted to minimum: %.3f rad/s", omega);
            }
        }

        local_plan_.push_back({v, omega});
    }

    // 如果控制输入不够，填充零值
    while (local_plan_.size() < static_cast<size_t>(desired_path_size_)) {
        local_plan_.push_back({0.0, 0.0});
    }

    // 如果控制输入过多，截断
    if (local_plan_.size() > static_cast<size_t>(desired_path_size_)) {
        local_plan_.resize(desired_path_size_);
    }
}

void LocalPlanner::generateFallbackTrackingPlan(const MPCSolver::State& current_state,
                                                const std::vector<MPCSolver::State>& mpc_path) {
    local_plan_.clear();

    if (mpc_path.empty()) {
        for (int i = 0; i < desired_path_size_; ++i) {
            local_plan_.push_back({0.0, 0.0});
        }
        return;
    }

    const auto& target = mpc_path.front();
    const double dx = target.x - current_state.x;
    const double dy = target.y - current_state.y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_heading - current_state.theta);

    double v = std::min(fallback_max_linear_vel_, 0.6 * distance);
    v *= std::max(0.0, std::cos(heading_error));
    double omega = std::max(-fallback_max_angular_vel_,
                            std::min(fallback_max_angular_vel_, 1.5 * heading_error));

    if (distance < 0.15) {
        v = 0.0;
    }

    ROS_INFO_THROTTLE(1.0, "[Local Planner] Fallback cmd: v=%.3f omega=%.3f dist=%.3f heading_err=%.3f",
                      v, omega, distance, heading_error);

    for (int i = 0; i < desired_path_size_; ++i) {
        local_plan_.push_back({v, omega});
    }
}

double LocalPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double LocalPlanner::quaternionToYaw(const geometry_msgs::Quaternion& q) {
    // 计算yaw角度 (绕z轴旋转)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void LocalPlanner::initializeParameters() {
    private_nh_.param<std::string>("base_frame", base_frame_, "motion_link");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<double>("planning_frequency", planning_frequency_, 10.0);
    private_nh_.param<int>("desired_path_size", desired_path_size_, 10);
    private_nh_.param<double>("path_buffer_limit", path_buffer_limit_, 1000.0);
    private_nh_.param<double>("robot_height", robot_height_, 0.0);
    private_nh_.param<double>("obstacle_grid_size", obstacle_grid_size_, 0.3);
    private_nh_.param<double>("pos_tolerance", pos_tolerance_, 0.50);  // 位置容差
    private_nh_.param<double>("min_angular_vel", min_angular_vel_, 0.1);  // 最小角速度 (rad/s)
    private_nh_.param<double>("goal_align_angular_vel", goal_align_angular_vel_, 1.0);  // GOAL_ALIGN恒定角速度
    private_nh_.param<bool>("mpc/solver_enabled", mpc_solver_enabled_, true);
    private_nh_.param<bool>("mpc/fallback_tracking_enabled", fallback_tracking_enabled_, true);
    private_nh_.param<double>("mpc/fallback_max_linear_vel", fallback_max_linear_vel_, 0.25);
    private_nh_.param<double>("mpc/fallback_max_angular_vel", fallback_max_angular_vel_, 0.6);

    ROS_INFO("Local Planner Parameters:");
    ROS_INFO("  base_frame: %s", base_frame_.c_str());
    ROS_INFO("  map_frame: %s", map_frame_.c_str());
    ROS_INFO("  planning_frequency: %.1f Hz", planning_frequency_);
    ROS_INFO("  desired_path_size: %d", desired_path_size_);
    ROS_INFO("  robot_height: %.2f m", robot_height_);
    ROS_INFO("  pos_tolerance: %.2f m", pos_tolerance_);
    ROS_INFO("  min_angular_vel: %.2f rad/s", min_angular_vel_);
    ROS_INFO("  goal_align_angular_vel: %.2f rad/s", goal_align_angular_vel_);
    ROS_INFO("  mpc_solver_enabled: %s", mpc_solver_enabled_ ? "true" : "false");
    ROS_INFO("  fallback_tracking_enabled: %s", fallback_tracking_enabled_ ? "true" : "false");
    ROS_INFO("  fallback_max_linear_vel: %.2f m/s", fallback_max_linear_vel_);
    ROS_INFO("  fallback_max_angular_vel: %.2f rad/s", fallback_max_angular_vel_);
}

void LocalPlanner::initializePublishersSubscribers() {
    // 发布者
    local_plan_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/local_plan", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

    // 订阅者
    global_path_sub_ = nh_.subscribe("/path_smooth", 10,
                                    &LocalPlanner::globalPathCallback, this);
    curr_state_sub_ = nh_.subscribe("/curr_state", 10,
                                   &LocalPlanner::currentStateCallback, this);
    obstacle_sub_ = nh_.subscribe("/obs_raw", 10,
                                 &LocalPlanner::obstacleCallback, this);
    nav_state_sub_ = nh_.subscribe("/navigation_state", 10,
                                  &LocalPlanner::navigationStateCallback, this);
}

void LocalPlanner::updateGoalAlignDirection() {
    double yaw_error = 0.0;
    bool have_state = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_received_) {
            double current_yaw = current_state_[3];
            yaw_error = normalizeAngle(goal_yaw_ - current_yaw);
            have_state = true;
        }
    }

    double rotation_speed = goal_align_angular_vel_;
    rotation_speed = (rotation_speed >= 0.0) ? std::abs(rotation_speed) : -std::abs(rotation_speed);

    if (have_state) {
        if (std::abs(yaw_error) < 1e-3) {
            rotation_speed = 0.0;
        } else {
            double direction = (yaw_error >= 0.0) ? 1.0 : -1.0;
            rotation_speed = std::abs(goal_align_angular_vel_) * direction;
        }
    }

    if (rotation_speed != 0.0 && std::abs(rotation_speed) < min_angular_vel_) {
        rotation_speed = (rotation_speed > 0.0 ? 1.0 : -1.0) * min_angular_vel_;
    }

    goal_align_rotation_speed_ = rotation_speed;
    goal_align_direction_set_ = true;

    if (have_state) {
        ROS_INFO("[Local Planner] GOAL_ALIGN direction initialized: yaw_error=%.3f rad (%.1f deg), omega=%.2f",
                 yaw_error, yaw_error * 180.0 / M_PI, rotation_speed);
    } else {
        ROS_WARN("[Local Planner] GOAL_ALIGN direction initialized without current state, omega=%.2f",
                 rotation_speed);
    }
}

/**
 * @brief 构造用于姿态对齐的纯旋转参考路径
 * @return 由重复目标点组成的MPC路径（位置相同，姿态为目标姿态）
 */
std::vector<MPCSolver::State> LocalPlanner::buildAlignmentPath() {
    std::vector<MPCSolver::State> alignment_path;
    int required_points = mpc_solver_->getParams().N;

    // 获取当前姿态
    double current_yaw = current_state_[3];

    // 计算最短角度差（归一化到 [-π, π]）
    double yaw_error = normalizeAngle(goal_yaw_ - current_yaw);

    // 构造渐进的姿态路径，确保MPC沿最短路径旋转
    // 每个预测步骤逐渐逼近目标姿态
    for (int i = 0; i < required_points; ++i) {
        // 线性插值：从当前姿态逐步过渡到目标姿态
        double alpha = static_cast<double>(i + 1) / required_points;
        double intermediate_yaw = normalizeAngle(current_yaw + alpha * yaw_error);
        alignment_path.emplace_back(goal_x_, goal_y_, intermediate_yaw);
    }

    ROS_INFO_THROTTLE(2.0, "[Local Planner] Alignment path: current_yaw=%.2f, goal_yaw=%.2f, error=%.2f rad (%.1f deg)",
                     current_yaw, goal_yaw_, yaw_error, yaw_error * 180.0 / M_PI);

    return alignment_path;
}

/**
 * @brief 检查机器人是否接近目标点（在位置容差范围内）
 * @return true 如果距离目标点的距离小于位置容差
 */
bool LocalPlanner::isNearGoal() const {
    double dx = current_state_[0] - goal_x_;
    double dy = current_state_[1] - goal_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance < pos_tolerance_;
}

/**
 * @brief 发布零控制指令
 */
void LocalPlanner::publishZeroControl() {
    local_plan_.clear();
    for (int i = 0; i < desired_path_size_; ++i) {
        local_plan_.push_back({0.0, 0.0});
    }
    publishLocalPlan();
}

} // namespace nmpc_planner
