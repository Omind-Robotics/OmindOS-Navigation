#ifndef NMPC_PLANNER_CONTROLLER_NODE_HPP
#define NMPC_PLANNER_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <array>
#include <vector>
#include <atomic>
#include <thread>
#include <cstdint>

#include "nmpc_planner/keyboard_input.hpp"

// Navigation state enumeration (shared with global planner and local planner)
enum class NavState : uint8_t {
    WAITING = 0,
    GLOBAL_PLANNING = 1,
    TRACKING = 2,
    GOAL_ALIGN = 3,
    COMPLETED = 4,
    ABORTED = 5
};

namespace nmpc_planner {

class ControllerNode {
public:
    // 控制模式枚举
    enum Mode {
        MANUAL,
        AUTO
    };
    
    // 状态结构
    struct RobotState {
        double x, y, z;      // 位置
        double roll, pitch, yaw;  // 姿态
        
        RobotState() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    };

public:
    ControllerNode();
    ~ControllerNode();
    
    // 主要接口
    void run();
    void shutdown();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布者和订阅者
    ros::Publisher cmd_vel_pub_;
    ros::Publisher curr_state_pub_;
    ros::Subscriber local_plan_sub_;
    ros::Subscriber nav_state_sub_;       // 订阅导航状态 (TODO: will be used in later steps)

    // TF监听器
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // 定时器
    ros::Timer state_timer_;

    // 键盘输入处理
    std::unique_ptr<KeyboardInput> keyboard_;

    // 状态变量
    std::atomic<bool> running_;
    Mode current_mode_;
    RobotState current_state_;
    std::vector<std::array<double, 2>> local_plan_;
    std::array<double, 2> manual_cmd_;

    // 导航状态 (TODO: will be actively used in later steps)
    NavState nav_state_;
    
    // 参数
    std::string base_frame_;
    std::string map_frame_;
    double control_frequency_;
    int plan_size_;
    
    // 信号处理
    static std::atomic<bool> shutdown_requested_;
    static void signalHandler(int signum);
    
    // 回调函数
    void localPlanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void navigationStateCallback(const std_msgs::UInt8::ConstPtr& msg);  // TODO: implement logic in later steps
    void stateTimerCallback(const ros::TimerEvent& event);
    
    // 核心功能函数
    void controlLoop();
    void manualMode();
    void autoMode();
    
    // 辅助函数
    void publishControlCommand(const std::array<double, 2>& cmd);
    void publishCurrentState();
    bool updateCurrentState();
    void quaternionToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);
    double normalizeAngle(double angle);
    
    // 初始化函数
    void initializeParameters();
    void initializePublishersSubscribers();
    void setupSignalHandler();
};

} // namespace nmpc_planner

#endif // NMPC_PLANNER_CONTROLLER_NODE_HPP