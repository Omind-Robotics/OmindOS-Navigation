#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "nmpc_planner/mpc_solver.hpp"

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

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("nmpc_local_planner")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    planning_frequency_ = declare_parameter<double>("planning_frequency", 10.0);
    desired_path_size_ = declare_parameter<int>("desired_path_size", 10);
    path_buffer_limit_ = declare_parameter<int>("path_buffer_limit", 1000);
    robot_height_ = declare_parameter<double>("robot_height", 0.35);
    obstacle_grid_size_ = declare_parameter<double>("obstacle_grid_size", 0.2);
    pos_tolerance_ = declare_parameter<double>("pos_tolerance", 0.5);
    min_angular_vel_ = declare_parameter<double>("min_angular_vel", 1.0);
    goal_align_angular_vel_ = declare_parameter<double>("goal_align_angular_vel", 1.0);
    solver_enabled_ = declare_parameter<bool>("mpc.solver_enabled", true);
    fallback_enabled_ = declare_parameter<bool>("mpc.fallback_tracking_enabled", true);
    fallback_max_linear_vel_ =
      declare_parameter<double>("mpc.fallback_max_linear_vel", 0.35);
    fallback_max_angular_vel_ =
      declare_parameter<double>("mpc.fallback_max_angular_vel", 0.8);

    nmpc_planner::MPCSolver::MPCParams params;
    params.T = declare_parameter<double>("mpc.control_horizon", 0.5);
    params.N = declare_parameter<int>("mpc.prediction_steps", 10);
    params.v_max = declare_parameter<double>("mpc.max_linear_vel", 0.9);
    params.omega_max = declare_parameter<double>("mpc.max_angular_vel", 1.4);
    params.S = Eigen::Matrix2d::Zero();
    params.S(0, 0) = declare_parameter<double>("mpc.smooth_v_weight", 0.8);
    params.S(1, 1) = declare_parameter<double>("mpc.smooth_omega_weight", 0.3);
    params.safe_distance = declare_parameter<double>("mpc.safe_distance", 0.4);
    params.influence_distance = declare_parameter<double>("mpc.influence_distance", 1.2);
    params.obstacle_weight = declare_parameter<double>("mpc.obstacle_weight", 10.0);
    params.use_time_weight = declare_parameter<bool>("mpc.use_time_weight", true);
    params.max_obstacles_consider =
      declare_parameter<int>("mpc.max_obstacles_consider", 10);
    params.obstacle_influence_range =
      declare_parameter<double>("mpc.obstacle_influence_range", 8.0);
    params.smooth_epsilon = declare_parameter<double>("mpc.smooth_epsilon", 0.01);
    mpc_solver_ = std::make_unique<nmpc_planner::MPCSolver>(params);

    local_plan_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/local_plan", 10);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path_smooth", 10,
      std::bind(&LocalPlannerNode::global_path_callback, this, std::placeholders::_1));
    current_state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/curr_state", 10,
      std::bind(&LocalPlannerNode::current_state_callback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/obs_raw", 10,
      std::bind(&LocalPlannerNode::obstacle_callback, this, std::placeholders::_1));
    nav_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/navigation_state", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalPlannerNode::nav_state_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / planning_frequency_);
    timer_ = create_wall_timer(period, std::bind(&LocalPlannerNode::planning_timer, this));
    RCLCPP_INFO(
      get_logger(), "Original NMPC planner ready: N=%d, solver=%s, map=%s, base=%s",
      params.N, solver_enabled_ ? "enabled" : "disabled", map_frame_.c_str(), base_frame_.c_str());
  }

private:
  struct PathPoint
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
  };

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }

  static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Empty global path received");
      return;
    }

    global_path_.clear();
    const int start_index = std::min(10, static_cast<int>(msg->poses.size()) - 1);
    for (int i = start_index;
      i < static_cast<int>(msg->poses.size()) &&
      global_path_.size() < static_cast<std::size_t>(path_buffer_limit_); ++i)
    {
      const auto & pose = msg->poses[static_cast<std::size_t>(i)].pose;
      global_path_.push_back(
        {pose.position.x, pose.position.y, quaternion_to_yaw(pose.orientation)});
    }
    if (!global_path_.empty()) {
      const auto & goal = global_path_.back();
      goal_x_ = goal.x;
      goal_y_ = goal.y;
      goal_yaw_ = goal.theta;
      ref_path_set_ = true;
    }
    RCLCPP_INFO(
      get_logger(), "Global path updated: input=%zu, NMPC buffer=%zu",
      msg->poses.size(), global_path_.size());
  }

  void current_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_[0] = msg->data[0];
    current_state_[1] = msg->data[1];
    current_state_[2] = msg->data[2];
    current_state_[3] = msg->data[3];
    robot_height_ = msg->data[2];
    state_received_ = true;
  }

  void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      std::lock_guard<std::mutex> lock(obstacle_mutex_);
      obstacles_.clear();
      return;
    }
    if (msg->data.size() % 3 != 0) {
      RCLCPP_WARN(get_logger(), "Invalid /obs_raw length: %zu", msg->data.size());
      return;
    }

    std::set<std::pair<double, double>> unique_grid_positions;
    for (std::size_t i = 0; i < msg->data.size(); i += 3) {
      const double x = std::floor(msg->data[i] / obstacle_grid_size_) * obstacle_grid_size_;
      const double y = std::floor(msg->data[i + 1] / obstacle_grid_size_) * obstacle_grid_size_;
      unique_grid_positions.emplace(x, y);
    }
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_.clear();
    for (const auto & point : unique_grid_positions) {
      obstacles_.emplace_back(point.first, point.second, 0.15);
    }
  }

  void nav_state_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data > static_cast<uint8_t>(NavState::ABORTED)) {
      RCLCPP_WARN(get_logger(), "Invalid navigation state: %u", msg->data);
      return;
    }
    const auto next = static_cast<NavState>(msg->data);
    if (next != nav_state_) {
      RCLCPP_INFO(
        get_logger(), "Navigation state: %u -> %u",
        static_cast<unsigned>(nav_state_), static_cast<unsigned>(next));
    }
    nav_state_ = next;
    if (next == NavState::COMPLETED || next == NavState::ABORTED) {
      std::lock_guard<std::mutex> lock(path_mutex_);
      global_path_.clear();
      local_plan_.clear();
      ref_path_set_ = false;
      publish_zero_control();
    }
    if (next == NavState::GOAL_ALIGN) {
      update_goal_align_direction();
    } else {
      goal_align_direction_set_ = false;
      goal_align_rotation_speed_ = 0.0;
    }
  }

  void planning_timer()
  {
    switch (nav_state_) {
      case NavState::TRACKING:
        if (ref_path_set_ && state_received_) {perform_planning();}
        break;
      case NavState::GOAL_ALIGN:
        if (state_received_) {perform_planning();}
        break;
      case NavState::COMPLETED:
      case NavState::ABORTED:
        publish_zero_control();
        break;
      default:
        break;
    }
  }

  std::vector<nmpc_planner::MPCSolver::State> convert_to_mpc_path() const
  {
    std::vector<nmpc_planner::MPCSolver::State> path;
    const int required = mpc_solver_->getParams().N;
    if (global_path_.empty()) {
      return path;
    }
    path.reserve(static_cast<std::size_t>(required));
    for (int i = 0; i < std::min(required, static_cast<int>(global_path_.size())); ++i) {
      const auto & point = global_path_[static_cast<std::size_t>(i)];
      path.emplace_back(point.x, point.y, point.theta);
    }
    while (static_cast<int>(path.size()) < required) {
      const auto & point = global_path_.back();
      path.emplace_back(point.x, point.y, point.theta);
    }
    return path;
  }

  void perform_planning()
  {
    std::scoped_lock lock(path_mutex_, state_mutex_, obstacle_mutex_);
    if (nav_state_ == NavState::GOAL_ALIGN) {
      if (!goal_align_direction_set_) {
        set_goal_align_direction_unlocked();
      }
      local_plan_.assign(
        static_cast<std::size_t>(desired_path_size_), {0.0, goal_align_rotation_speed_});
      publish_local_plan();
      return;
    }
    if (global_path_.empty()) {
      publish_zero_control();
      return;
    }

    const nmpc_planner::MPCSolver::State current(
      current_state_[0], current_state_[1], current_state_[3]);
    const auto path = convert_to_mpc_path();
    if (!solver_enabled_) {
      generate_fallback(current, path);
      publish_local_plan();
      return;
    }

    const auto result = mpc_solver_->solve(current, path, obstacles_);
    if (result.success) {
      local_plan_.clear();
      for (const auto & control : result.controls) {
        local_plan_.push_back({control.v, control.omega});
      }
      local_plan_.resize(static_cast<std::size_t>(desired_path_size_), {0.0, 0.0});
      publish_local_path(result);
    } else if (fallback_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "MPC solve failed; using original fallback tracker");
      generate_fallback(current, path);
    } else {
      local_plan_.assign(static_cast<std::size_t>(desired_path_size_), {0.0, 0.0});
    }
    publish_local_plan();
  }

  void generate_fallback(
    const nmpc_planner::MPCSolver::State & current,
    const std::vector<nmpc_planner::MPCSolver::State> & path)
  {
    if (path.empty()) {
      local_plan_.assign(static_cast<std::size_t>(desired_path_size_), {0.0, 0.0});
      return;
    }
    const auto & target = path.front();
    const double dx = target.x - current.x;
    const double dy = target.y - current.y;
    const double distance = std::hypot(dx, dy);
    const double heading_error = normalize_angle(std::atan2(dy, dx) - current.theta);
    double v = std::min(fallback_max_linear_vel_, 0.6 * distance);
    v *= std::max(0.0, std::cos(heading_error));
    if (distance < 0.15) {v = 0.0;}
    const double omega = std::clamp(
      1.5 * heading_error, -fallback_max_angular_vel_, fallback_max_angular_vel_);
    local_plan_.assign(static_cast<std::size_t>(desired_path_size_), {v, omega});
  }

  void update_goal_align_direction()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    set_goal_align_direction_unlocked();
  }

  void set_goal_align_direction_unlocked()
  {
    double speed = 0.0;
    if (state_received_) {
      const double error = normalize_angle(goal_yaw_ - current_state_[3]);
      if (std::abs(error) >= 1e-3) {
        speed = std::copysign(std::abs(goal_align_angular_vel_), error);
      }
    }
    if (speed != 0.0 && std::abs(speed) < min_angular_vel_) {
      speed = std::copysign(min_angular_vel_, speed);
    }
    goal_align_rotation_speed_ = speed;
    goal_align_direction_set_ = true;
  }

  void publish_local_plan()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(local_plan_.size() * 2);
    for (const auto & control : local_plan_) {
      msg.data.push_back(static_cast<float>(control[0]));
      msg.data.push_back(static_cast<float>(control[1]));
    }
    local_plan_pub_->publish(msg);
  }

  void publish_zero_control()
  {
    local_plan_.assign(static_cast<std::size_t>(desired_path_size_), {0.0, 0.0});
    publish_local_plan();
  }

  void publish_local_path(const nmpc_planner::MPCSolver::MPCResult & result)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;
    for (const auto & state : result.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = state.x;
      pose.pose.position.y = state.y;
      pose.pose.position.z = robot_height_;
      pose.pose.orientation.z = std::sin(state.theta / 2.0);
      pose.pose.orientation.w = std::cos(state.theta / 2.0);
      path.poses.push_back(pose);
    }
    local_path_pub_->publish(path);
  }

  std::string base_frame_;
  std::string map_frame_;
  double planning_frequency_{10.0};
  int desired_path_size_{10};
  int path_buffer_limit_{1000};
  double robot_height_{0.35};
  double obstacle_grid_size_{0.2};
  double pos_tolerance_{0.5};
  double min_angular_vel_{1.0};
  double goal_align_angular_vel_{1.0};
  bool solver_enabled_{true};
  bool fallback_enabled_{true};
  double fallback_max_linear_vel_{0.35};
  double fallback_max_angular_vel_{0.8};

  bool ref_path_set_{false};
  bool state_received_{false};
  NavState nav_state_{NavState::WAITING};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  bool goal_align_direction_set_{false};
  double goal_align_rotation_speed_{0.0};

  std::vector<PathPoint> global_path_;
  std::array<double, 4> current_state_{{0.0, 0.0, 0.0, 0.0}};
  std::vector<std::array<double, 2>> local_plan_;
  std::vector<nmpc_planner::MPCSolver::Obstacle> obstacles_;
  std::mutex path_mutex_;
  std::mutex state_mutex_;
  std::mutex obstacle_mutex_;

  std::unique_ptr<nmpc_planner::MPCSolver> mpc_solver_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr local_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nmpc_planner_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nmpc_planner_ros2::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
