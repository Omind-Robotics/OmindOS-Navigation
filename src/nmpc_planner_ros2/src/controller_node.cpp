#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nmpc_planner/keyboard_input.hpp"

namespace nmpc_planner_ros2
{

enum class NavState : uint8_t
{
  WAITING = 0,
  GLOBAL_PLANNING = 1,
  TRACKING = 2,
  GOAL_ALIGN = 3,
  COMPLETED = 4,
  ABORTED = 5,
};

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("nmpc_controller_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
    plan_size_ = declare_parameter<int>("plan_size", 10);
    const bool start_in_auto = declare_parameter<bool>("start_in_auto", false);
    mode_ = start_in_auto ? Mode::AUTO : Mode::MANUAL;

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/curr_state", 10);
    local_plan_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/local_plan", 10,
      std::bind(&ControllerNode::local_plan_callback, this, std::placeholders::_1));
    nav_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/navigation_state", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ControllerNode::nav_state_callback, this, std::placeholders::_1));

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&ControllerNode::state_timer, this));
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&ControllerNode::control_timer, this));
    RCLCPP_INFO(
      get_logger(), "Original controller ready in %s mode (map=%s, base=%s)",
      mode_ == Mode::AUTO ? "AUTO" : "MANUAL", map_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "%s", keyboard_.getHelpText().c_str());
  }

  ~ControllerNode() override
  {
    publish_command({0.0, 0.0});
  }

private:
  enum class Mode {MANUAL, AUTO};

  struct RobotState
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
  };

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }

  void local_plan_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    local_plan_.clear();
    const std::size_t limit = std::min(
      msg->data.size(), static_cast<std::size_t>(plan_size_ * 2));
    for (std::size_t i = 0; i + 1 < limit; i += 2) {
      local_plan_.push_back({msg->data[i], msg->data[i + 1]});
    }
  }

  void nav_state_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data <= static_cast<uint8_t>(NavState::ABORTED)) {
      nav_state_ = static_cast<NavState>(msg->data);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid navigation state: %u", msg->data);
    }
  }

  void state_timer()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.02));
      current_state_.x = transform.transform.translation.x;
      current_state_.y = transform.transform.translation.y;
      current_state_.z = transform.transform.translation.z;
      tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
      tf2::Matrix3x3(q).getRPY(
        current_state_.roll, current_state_.pitch, current_state_.yaw);
    } catch (const tf2::TransformException &) {
      return;
    }

    std_msgs::msg::Float32MultiArray message;
    message.data = {
      static_cast<float>(current_state_.x),
      static_cast<float>(current_state_.y),
      static_cast<float>(current_state_.z),
      static_cast<float>(normalize_angle(current_state_.yaw)),
      static_cast<float>(current_state_.roll),
      static_cast<float>(current_state_.pitch)};
    state_pub_->publish(message);
  }

  void control_timer()
  {
    const char key = keyboard_.getKey(0.0);
    if (mode_ == Mode::MANUAL) {
      if (keyboard_.isModeSwitch(key)) {
        mode_ = Mode::AUTO;
        RCLCPP_INFO(get_logger(), "Switched to AUTO mode");
        return;
      }
      manual_command_ = keyboard_.processManualControl(key, manual_command_);
      publish_command(manual_command_);
      return;
    }

    if (key == 'q' || keyboard_.isExitKey(key)) {
      mode_ = Mode::MANUAL;
      manual_command_ = {0.0, 0.0};
      publish_command(manual_command_);
      RCLCPP_INFO(get_logger(), "Switched to MANUAL mode");
      return;
    }
    if (local_plan_.size() > 5) {
      publish_command(local_plan_.front());
    } else {
      publish_command({0.0, 0.0});
    }
  }

  void publish_command(const std::array<double, 2> & command)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = command[0];
    message.angular.z = command[1];
    cmd_vel_pub_->publish(message);
  }

  std::string base_frame_;
  std::string map_frame_;
  double control_frequency_{50.0};
  int plan_size_{10};
  Mode mode_{Mode::MANUAL};
  NavState nav_state_{NavState::WAITING};
  RobotState current_state_;
  std::vector<std::array<double, 2>> local_plan_;
  std::array<double, 2> manual_command_{{0.0, 0.0}};
  nmpc_planner::KeyboardInput keyboard_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr local_plan_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_state_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace nmpc_planner_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nmpc_planner_ros2::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
