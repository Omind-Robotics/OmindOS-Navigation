#ifndef NMPC_PLANNER_LOCAL_PLANNER_HPP
#define NMPC_PLANNER_LOCAL_PLANNER_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <vector>
#include <array>
#include <mutex>
#include <cstdint>

#include "nmpc_planner/mpc_solver.hpp"

// Navigation state enumeration (shared with global planner)
enum class NavState : uint8_t {
    WAITING = 0,
    GLOBAL_PLANNING = 1,
    TRACKING = 2,
    GOAL_ALIGN = 3,
    COMPLETED = 4,
    ABORTED = 5
};

namespace nmpc_planner {

class LocalPlanner {
public:
    // 路径点结构
    struct PathPoint {
        double x, y, theta;
        double confidence;
        
        PathPoint() : x(0), y(0), theta(0), confidence(1.0) {}
        PathPoint(double x_, double y_, double theta_, double conf_ = 1.0) 
            : x(x_), y(y_), theta(theta_), confidence(conf_) {}
    };

public:
    LocalPlanner();
    ~LocalPlanner();
    
    // 主要接口
    void initialize();
    void shutdown();
    
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布者和订阅者
    ros::Publisher local_plan_pub_;      // 发布Float32MultiArray格式的控制指令
    ros::Publisher local_path_pub_;      // 发布Path格式的轨迹路径
    ros::Subscriber global_path_sub_;
    ros::Subscriber curr_state_sub_;
    ros::Subscriber obstacle_sub_;        // 订阅障碍物数据
    ros::Subscriber nav_state_sub_;       // 订阅导航状态 (TODO: will be used in later steps)
    
    // TF监听器
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // MPC求解器
    std::unique_ptr<MPCSolver> mpc_solver_;
    
    // 定时器
    ros::Timer planning_timer_;
    
    // 数据存储
    std::vector<PathPoint> global_path_;
    std::array<double, 4> current_state_;  // [x, y, z, yaw]
    std::vector<std::array<double, 2>> local_plan_;
    std::vector<MPCSolver::Obstacle> obstacles_;  // 障碍物列表
    
    // 互斥锁
    std::mutex path_mutex_;
    std::mutex state_mutex_;
    std::mutex obstacle_mutex_;
    
    // 参数
    std::string base_frame_;
    std::string map_frame_;
    double planning_frequency_;
    int desired_path_size_;
    double path_buffer_limit_;
    double robot_height_;  // 机器人高度
    double obstacle_grid_size_;  // 障碍物网格化大小
    double pos_tolerance_;  // 位置容差，用于判断是否进入姿态对齐阶段
    double min_angular_vel_;  // 最小角速度阈值（硬件响应下限）
    double goal_align_angular_vel_;  // GOAL_ALIGN 阶段的恒定角速度
    double goal_align_rotation_speed_;  // 实际执行时的恒定角速度（含方向）
    bool mpc_solver_enabled_;
    bool fallback_tracking_enabled_;  // NMPC失败时使用低速几何跟踪
    double fallback_max_linear_vel_;
    double fallback_max_angular_vel_;

    // 标志位
    bool ref_path_set_;
    bool state_received_;
    bool goal_align_direction_set_;

    // 导航状态
    NavState nav_state_;

    // 目标点缓存（用于GOAL_ALIGN阶段）
    double goal_x_;
    double goal_y_;
    double goal_yaw_;

    // 回调函数
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void currentStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void obstacleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void navigationStateCallback(const std_msgs::UInt8::ConstPtr& msg);  // TODO: implement logic in later steps
    void planningTimerCallback(const ros::TimerEvent& event);
    
    // 核心功能函数
    void performPlanning();
    void publishLocalPlan();
    void publishLocalPath(const MPCSolver::MPCResult& result);  // 发布路径轨迹
    
    // 辅助函数
    void initializeParameters();
    void initializePublishersSubscribers();
    std::vector<MPCSolver::State> convertToMPCPath();
    MPCSolver::State getCurrentMPCState();
    void convertMPCResultToLocalPlan(const MPCSolver::MPCResult& result);
    void generateFallbackTrackingPlan(const MPCSolver::State& current_state,
                                      const std::vector<MPCSolver::State>& mpc_path);
    
    // 路径处理函数
    bool processGlobalPathData(const nav_msgs::Path::ConstPtr& msg);
    double normalizeAngle(double angle);
    double quaternionToYaw(const geometry_msgs::Quaternion& q);

    // 姿态对齐相关函数
    std::vector<MPCSolver::State> buildAlignmentPath();
    bool isNearGoal() const;
    void publishZeroControl();
    void updateGoalAlignDirection();
};

} // namespace nmpc_planner

#endif // NMPC_PLANNER_LOCAL_PLANNER_HPP
