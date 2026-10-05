#include "nmpc_planner/controller_node.hpp"
#include <csignal>
#include <iostream>
#include <cmath>

namespace nmpc_planner {

std::atomic<bool> ControllerNode::shutdown_requested_{false};

ControllerNode::ControllerNode()
    : nh_(), private_nh_("~"), running_(true), current_mode_(MANUAL),
      nav_state_(NavState::WAITING) {

    // 初始化参数
    initializeParameters();
    
    // 初始化ROS通信
    initializePublishersSubscribers();
    
    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化键盘输入
    keyboard_ = std::make_unique<KeyboardInput>();
    
    // 初始化控制指令
    manual_cmd_ = {0.0, 0.0};
    
    // 设置信号处理
    setupSignalHandler();
    
    // 状态定时器
    state_timer_ = nh_.createTimer(ros::Duration(0.01), 
                                  &ControllerNode::stateTimerCallback, this);
    
    ROS_INFO("NMPC Controller Node initialized");
    std::cout << keyboard_->getHelpText() << std::endl;
}

ControllerNode::~ControllerNode() {
    shutdown();
}

void ControllerNode::run() {
    ros::Rate rate(control_frequency_);
    
    while (ros::ok() && running_ && !shutdown_requested_) {
        ros::spinOnce();
        
        // 主控制循环
        controlLoop();
        
        rate.sleep();
    }
    
    // 发送停止指令
    publishControlCommand({0.0, 0.0});
    ROS_INFO("Controller node shutting down safely");
}

void ControllerNode::shutdown() {
    running_ = false;
    publishControlCommand({0.0, 0.0});
}

void ControllerNode::signalHandler(int signum) {
    if (signum == SIGINT) {
        ROS_INFO("Received Ctrl+C signal, shutting down safely...");
        shutdown_requested_ = true;
        ros::shutdown();
    }
}

void ControllerNode::controlLoop() {
    switch (current_mode_) {
        case MANUAL:
            manualMode();
            break;
        case AUTO:
            autoMode();
            break;
    }
}

void ControllerNode::manualMode() {
    char key = keyboard_->getKey(50.0);  // 50ms超时
    
    if (keyboard_->isExitKey(key)) {
        ROS_INFO("Manual mode received Ctrl+C, exiting program");
        running_ = false;
        return;
    }
    
    if (keyboard_->isModeSwitch(key)) {
        current_mode_ = AUTO;
        ROS_INFO("Switched to AUTO mode");
        return;
    }
    
    // 处理手动控制指令
    manual_cmd_ = keyboard_->processManualControl(key, manual_cmd_);
    publishControlCommand(manual_cmd_);
}

void ControllerNode::autoMode() {
    char key = keyboard_->getKey(50.0);  // 50ms超时
    
    if (key == 'q' || keyboard_->isExitKey(key)) {
        current_mode_ = MANUAL;
        ROS_INFO("Switched to MANUAL mode");
        return;
    }
    
    // 检查local_plan数据是否有效
    if (local_plan_.size() > 5) {
        // 获取路径规划输出，使用第一个点作为参考
        auto ref_inputs = local_plan_[0];
        publishControlCommand(ref_inputs);
    } else {
        // 如果没有有效规划数据，停止机器人
        publishControlCommand({0.0, 0.0});
    }
}

void ControllerNode::localPlanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    local_plan_.clear();

    // 解析local_plan数据
    for (size_t i = 0; i < msg->data.size() && i < static_cast<size_t>(plan_size_ * 2); i += 2) {
        if (i + 1 < msg->data.size()) {
            std::array<double, 2> point = {msg->data[i], msg->data[i + 1]};
            local_plan_.push_back(point);
        }
    }
}

void ControllerNode::navigationStateCallback(const std_msgs::UInt8::ConstPtr& msg) {
    // Convert uint8_t to NavState enum
    if (msg->data <= static_cast<uint8_t>(NavState::ABORTED)) {
        nav_state_ = static_cast<NavState>(msg->data);
        ROS_DEBUG_THROTTLE(2.0, "[Controller] Navigation state updated: %d", msg->data);

        // TODO: In later steps, this callback will:
        // - Only execute local planner output when in TRACKING/GOAL_ALIGN states
        // - Force zero velocity in COMPLETED/ABORTED states
        // - Switch back to MANUAL mode when COMPLETED
    } else {
        ROS_WARN("[Controller] Received invalid navigation state: %d", msg->data);
    }
}

void ControllerNode::stateTimerCallback(const ros::TimerEvent& event) {
    updateCurrentState();
    publishCurrentState();
}

void ControllerNode::publishControlCommand(const std::array<double, 2>& cmd) {
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = cmd[0];
    twist_msg.angular.z = cmd[1];
    twist_msg.linear.z = 0.0;
    
    cmd_vel_pub_.publish(twist_msg);
    
    // 调试输出
    static int count = 0;
    if (++count % 20 == 0) {  // 每秒输出一次 (假设50Hz)
        std::cout << "Control input: [" << cmd[0] << ", " << cmd[1] << "]" << std::endl;
    }
}

void ControllerNode::publishCurrentState() {
    std_msgs::Float32MultiArray state_msg;
    state_msg.data.resize(6);
    
    state_msg.data[0] = static_cast<float>(current_state_.x);
    state_msg.data[1] = static_cast<float>(current_state_.y);
    state_msg.data[2] = static_cast<float>(current_state_.z);
    state_msg.data[3] = static_cast<float>(normalizeAngle(current_state_.yaw));
    state_msg.data[4] = static_cast<float>(current_state_.roll);
    state_msg.data[5] = static_cast<float>(current_state_.pitch);
    
    curr_state_pub_.publish(state_msg);
}

bool ControllerNode::updateCurrentState() {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
            map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
        
        // 更新位置
        current_state_.x = transform.transform.translation.x;
        current_state_.y = transform.transform.translation.y;
        current_state_.z = transform.transform.translation.z;
        
        // 更新姿态
        quaternionToRPY(transform.transform.rotation, 
                       current_state_.roll, 
                       current_state_.pitch, 
                       current_state_.yaw);
        
        return true;
        
    } catch (tf2::TransformException& ex) {
        // 静默处理TF异常，避免日志污染
        return false;
    }
}

void ControllerNode::quaternionToRPY(const geometry_msgs::Quaternion& q, 
                                    double& roll, double& pitch, double& yaw) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    m.getRPY(roll, pitch, yaw);
}

double ControllerNode::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void ControllerNode::initializeParameters() {
    private_nh_.param<std::string>("base_frame", base_frame_, "motion_link");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<double>("control_frequency", control_frequency_, 50.0);
    private_nh_.param<int>("plan_size", plan_size_, 10);
    bool start_in_auto = false;
    private_nh_.param<bool>("start_in_auto", start_in_auto, false);
    if (start_in_auto) {
        current_mode_ = AUTO;
    }
    
    ROS_INFO("Parameters loaded:");
    ROS_INFO("  base_frame: %s", base_frame_.c_str());
    ROS_INFO("  map_frame: %s", map_frame_.c_str());
    ROS_INFO("  control_frequency: %.1f Hz", control_frequency_);
    ROS_INFO("  plan_size: %d", plan_size_);
    ROS_INFO("  start_in_auto: %s", start_in_auto ? "true" : "false");
}

void ControllerNode::initializePublishersSubscribers() {
    // 发布者
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    curr_state_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/curr_state", 10);

    // 订阅者
    local_plan_sub_ = nh_.subscribe("/local_plan", 10,
                                   &ControllerNode::localPlanCallback, this);
    nav_state_sub_ = nh_.subscribe("/navigation_state", 10,
                                  &ControllerNode::navigationStateCallback, this);
}

void ControllerNode::setupSignalHandler() {
    std::signal(SIGINT, signalHandler);
}

} // namespace nmpc_planner
