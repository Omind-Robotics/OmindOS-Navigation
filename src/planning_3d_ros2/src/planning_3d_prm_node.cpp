#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "planning_3d_ros2/edt_3d.hpp"
#include "planning_3d_ros2/nav_state.hpp"

using namespace std::chrono_literals;

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    const auto hx = std::hash<int>{}(key.x);
    const auto hy = std::hash<int>{}(key.y);
    const auto hz = std::hash<int>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

class Planning3DPrmNode final : public rclcpp::Node
{
public:
  Planning3DPrmNode()
  : Node("planning_3d_prm"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    read_parameters();
    create_interfaces();
    load_traversable_map();
    build_distance_field();
    build_prm();
    build_vertex_tree();
    publish_static_debug_data();

    nav_state_ = NavState::ABORTED;
    set_nav_state(NavState::WAITING);
    last_replan_time_ = now();
    update_timer_ = create_wall_timer(100ms, std::bind(&Planning3DPrmNode::update, this));
    RCLCPP_INFO(
      get_logger(), "ROS 2 3D PRM planner ready; TF start is %s -> %s",
      map_frame_.c_str(), robot_frame_.c_str());
  }

private:
  struct EdgeInfo
  {
    int target_id;
    double min_clearance;
    double avg_clearance;
  };

  struct Vertex
  {
    Eigen::Vector3d position;
    std::vector<EdgeInfo> edges;
  };

  struct EdgeCheckResult
  {
    bool free{false};
    double min_clearance{std::numeric_limits<double>::max()};
    double avg_clearance{0.0};
  };

  struct QueueItem
  {
    int id;
    double score;
    bool operator<(const QueueItem & other) const {return score > other.score;}
  };

  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    return declare_parameter<T>(name, default_value);
  }

  void read_parameters()
  {
    pcd_path_ = parameter<std::string>("pcd_path", "");
    map_frame_ = parameter<std::string>("map_frame", "map");
    robot_frame_ = parameter<std::string>("robot_frame", "base_link");
    edt_xy_expand_ = parameter<double>("edt_xy_expand", 1.0);
    edt_z_thickness_ = parameter<int>("edt_z_thickness", 3);
    robot_height_ = parameter<double>("robot_height", 1.0);
    enable_headroom_check_ = parameter<bool>("enable_headroom_check", true);
    safe_margin_ = parameter<double>("safe_margin", 0.4);
    voxel_leaf_ = parameter<double>("voxel_leaf", 0.4);
    use_clearance_penalty_ = parameter<bool>("use_clearance_penalty", true);
    clearance_penalty_weight_ = parameter<double>("clearance_penalty_weight", 22750.0);
    clearance_penalty_scale_ = parameter<double>("clearance_penalty_scale", 0.3);
    use_soft_penalty_ = parameter<bool>("use_soft_penalty", false);
    step_size_ = parameter<double>("step_size", 1.0);
    max_nodes_ = parameter<int>("max_nodes", 10000);
    k_neigh_ = parameter<int>("k_neigh", 50);
    const double max_slope_deg = parameter<double>("max_slope_deg", 45.0);
    max_slope_tan_ = std::tan(max_slope_deg * M_PI / 180.0);
    auto_replan_ = parameter<bool>("auto_replan", true);
    replan_frequency_ = parameter<double>("replan_frequency", 1.0);
    pos_tolerance_ = parameter<double>("pos_tolerance", 0.5);
    pos_tolerance_exit_ = parameter<double>("pos_tolerance_exit", 2.0);
    yaw_tolerance_ = parameter<double>("yaw_tolerance", 0.14);
    obstacle_block_radius_ = parameter<double>("obstacle_block_radius", 0.0);
    obstacle_timeout_ = parameter<double>("obstacle_timeout", 5.0);
    random_seed_ = parameter<int>("random_seed", 42);

    if (pcd_path_.empty()) {
      throw std::runtime_error("pcd_path is required");
    }
    if (voxel_leaf_ <= 0.0 || step_size_ <= 0.0 || max_nodes_ <= 0 || k_neigh_ <= 0) {
      throw std::runtime_error("voxel_leaf, step_size, max_nodes and k_neigh must be positive");
    }
    if (replan_frequency_ <= 0.0) {
      throw std::runtime_error("replan_frequency must be positive");
    }
    RCLCPP_INFO(
      get_logger(), "3D parameters: voxel=%.2f m safe=%.2f m slope=%.1f deg nodes=%d",
      voxel_leaf_, safe_margin_, max_slope_deg, max_nodes_);
  }

  void create_interfaces()
  {
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pcd_map", latched_qos);
    graph_pub_ = create_publisher<visualization_msgs::msg::Marker>("prm_graph", latched_qos);
    safe_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("safe_centers", latched_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", rclcpp::QoS(1).reliable());
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    nav_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "/navigation_state", latched_qos);
    nav_state_debug_pub_ = create_publisher<std_msgs::msg::String>(
      "/navigation_state_debug", latched_qos);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&Planning3DPrmNode::goal_callback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/obs_raw", 10,
      std::bind(&Planning3DPrmNode::obstacle_callback, this, std::placeholders::_1));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/navigation/cancel",
      std::bind(
        &Planning3DPrmNode::cancel_callback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  VoxelKey voxel_key(const Eigen::Vector3d & point) const
  {
    return VoxelKey{
      static_cast<int>(std::floor(point.x() / voxel_resolution_)),
      static_cast<int>(std::floor(point.y() / voxel_resolution_)),
      static_cast<int>(std::floor(point.z() / voxel_resolution_))};
  }

  Eigen::Vector3d voxel_center(const VoxelKey & key) const
  {
    return Eigen::Vector3d(
      (key.x + 0.5) * voxel_resolution_,
      (key.y + 0.5) * voxel_resolution_,
      (key.z + 0.5) * voxel_resolution_);
  }

  void load_traversable_map()
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) < 0 || cloud->empty()) {
      throw std::runtime_error("unable to load traversable PCD: " + pcd_path_);
    }
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    const float leaf = static_cast<float>(voxel_leaf_);
    filter.setLeafSize(leaf, leaf, leaf);
    filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filter.filter(*filtered_cloud_);
    if (filtered_cloud_->empty()) {
      throw std::runtime_error("traversable PCD became empty after voxel filtering");
    }

    pcl::toROSMsg(*filtered_cloud_, pcd_message_);
    pcd_message_.header.frame_id = map_frame_;
    voxel_resolution_ = voxel_leaf_;
    for (const auto & point : filtered_cloud_->points) {
      const Eigen::Vector3d position(point.x, point.y, point.z);
      if (position.allFinite()) {
        ++hit_count_[voxel_key(position)];
      }
    }
    raw_hit_count_ = hit_count_;
    free_centers_.reserve(hit_count_.size());
    for (const auto & item : hit_count_) {
      free_centers_.push_back(voxel_center(item.first));
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu PCD points; %zu points and %zu voxels after filtering",
      cloud->size(), filtered_cloud_->size(), hit_count_.size());
  }

  bool has_headroom(const VoxelKey & key) const
  {
    if (!enable_headroom_check_) {
      return true;
    }
    const int layers = static_cast<int>(std::ceil(robot_height_ / voxel_resolution_));
    for (int dz = 1; dz <= layers; ++dz) {
      const VoxelKey above{key.x, key.y, key.z + dz};
      if (raw_hit_count_.find(above) != raw_hit_count_.end()) {
        return false;
      }
    }
    return true;
  }

  void build_distance_field()
  {
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int min_z = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::lowest();
    int max_y = std::numeric_limits<int>::lowest();
    int max_z = std::numeric_limits<int>::lowest();
    for (const auto & item : hit_count_) {
      min_x = std::min(min_x, item.first.x);
      min_y = std::min(min_y, item.first.y);
      min_z = std::min(min_z, item.first.z);
      max_x = std::max(max_x, item.first.x);
      max_y = std::max(max_y, item.first.y);
      max_z = std::max(max_z, item.first.z);
    }
    const int expansion = static_cast<int>(std::ceil(edt_xy_expand_ / voxel_resolution_));
    min_x -= expansion;
    min_y -= expansion;
    max_x += expansion;
    max_y += expansion;
    const int nx = max_x - min_x + 1;
    const int ny = max_y - min_y + 1;
    const int nz = max_z - min_z + 1;
    const std::size_t total =
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
    if (total == 0 || total > 200000000ULL) {
      throw std::runtime_error("invalid or excessively large EDT volume");
    }
    const auto index = [=](const int x, const int y, const int z) {
        return ((z - min_z) * ny + (y - min_y)) * nx + (x - min_x);
      };
    std::vector<uint8_t> occupancy(total, 0U);
    for (const auto & item : hit_count_) {
      for (int dz = -edt_z_thickness_; dz <= edt_z_thickness_; ++dz) {
        const int z = item.first.z + dz;
        if (z >= min_z && z <= max_z) {
          occupancy[index(item.first.x, item.first.y, z)] = 1U;
        }
      }
    }
    std::vector<float> distances;
    EDT3D::compute(occupancy, nx, ny, nz, voxel_resolution_, distances);
    for (const auto & item : hit_count_) {
      float distance = distances[index(item.first.x, item.first.y, item.first.z)];
      if (!std::isfinite(distance) || distance > 1.0e6F) {
        distance = 0.5F;
      }
      voxel_margin_[item.first] = distance;
    }

    select_safe_centers(safe_margin_);
    if (safe_centers_.size() < 100U) {
      RCLCPP_WARN(
        get_logger(), "Only %zu safe voxels at %.2f m; retrying at half margin",
        safe_centers_.size(), safe_margin_);
      select_safe_centers(safe_margin_ * 0.5);
    }
    if (safe_centers_.empty()) {
      RCLCPP_WARN(get_logger(), "No eroded safe voxels; falling back to all traversable voxels");
      safe_centers_ = free_centers_;
    }
    RCLCPP_INFO(
      get_logger(), "3D EDT volume %d x %d x %d; selected %zu safe voxels",
      nx, ny, nz, safe_centers_.size());
  }

  void select_safe_centers(const double threshold)
  {
    safe_centers_.clear();
    for (const auto & item : voxel_margin_) {
      if (item.second >= threshold && has_headroom(item.first)) {
        safe_centers_.push_back(voxel_center(item.first));
      }
    }
  }

  double distance_to_boundary(const Eigen::Vector3d & point) const
  {
    const auto found = voxel_margin_.find(voxel_key(point));
    return found == voxel_margin_.end() ? 0.0 : found->second;
  }

  bool is_free(const Eigen::Vector3d & point) const
  {
    const auto key = voxel_key(point);
    if (hit_count_.find(key) == hit_count_.end()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    return dynamic_obstacles_.find(key) == dynamic_obstacles_.end();
  }

  bool slope_valid(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
  {
    const double dz = std::abs(b.z() - a.z());
    const double dxy = std::hypot(b.x() - a.x(), b.y() - a.y());
    return dxy < 1.0e-6 ? dz < 1.0e-6 : dz / dxy <= max_slope_tan_;
  }

  EdgeCheckResult check_edge(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
  {
    EdgeCheckResult result;
    const int samples = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.1)));
    double clearance_sum = 0.0;
    for (int i = 0; i <= samples; ++i) {
      const Eigen::Vector3d point = a + (b - a) * (static_cast<double>(i) / samples);
      const double clearance = distance_to_boundary(point);
      if (!is_free(point) || clearance < safe_margin_) {
        return result;
      }
      result.min_clearance = std::min(result.min_clearance, clearance);
      clearance_sum += clearance;
    }
    result.free = true;
    result.avg_clearance = clearance_sum / (samples + 1);
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_as_cloud() const
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(vertices_.size());
    for (const auto & vertex : vertices_) {
      cloud->push_back(pcl::PointXYZ(
        static_cast<float>(vertex.position.x()),
        static_cast<float>(vertex.position.y()),
        static_cast<float>(vertex.position.z())));
    }
    return cloud;
  }

  void build_prm()
  {
    RCLCPP_INFO(get_logger(), "Building 3D PRM graph...");
    const auto started = now();
    std::mt19937 random(static_cast<std::mt19937::result_type>(random_seed_));
    std::vector<Eigen::Vector3d> samples = safe_centers_;
    std::shuffle(samples.begin(), samples.end(), random);
    if (samples.size() > static_cast<std::size_t>(max_nodes_)) {
      samples.resize(static_cast<std::size_t>(max_nodes_));
    }
    vertices_.resize(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      vertices_[i].position = samples[i];
    }
    auto cloud = vertices_as_cloud();
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    std::size_t slope_rejected = 0;
    std::size_t clearance_rejected = 0;
    std::size_t accepted = 0;
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      std::vector<int> indices;
      std::vector<float> squared_distances;
      tree.radiusSearch(cloud->points[i], step_size_, indices, squared_distances);
      if (indices.size() > static_cast<std::size_t>(k_neigh_ + 1)) {
        indices.resize(static_cast<std::size_t>(k_neigh_ + 1));
      }
      for (const int index : indices) {
        if (index <= static_cast<int>(i)) {
          continue;
        }
        if (!slope_valid(vertices_[i].position, vertices_[index].position)) {
          ++slope_rejected;
          continue;
        }
        const auto result = check_edge(vertices_[i].position, vertices_[index].position);
        if (!result.free) {
          ++clearance_rejected;
          continue;
        }
        vertices_[i].edges.push_back(EdgeInfo{index, result.min_clearance, result.avg_clearance});
        vertices_[index].edges.push_back(
          EdgeInfo{static_cast<int>(i), result.min_clearance, result.avg_clearance});
        ++accepted;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "PRM built: %zu vertices, %zu edges, rejected slope=%zu clearance=%zu, %.2f s",
      vertices_.size(), accepted, slope_rejected, clearance_rejected, (now() - started).seconds());
  }

  void build_vertex_tree()
  {
    vertex_cloud_ = vertices_as_cloud();
    vertex_tree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
    vertex_tree_->setInputCloud(vertex_cloud_);
    vertex_blocked_.assign(vertices_.size(), false);
  }

  int closest_vertex(
    const Eigen::Vector3d & point, const bool skip_blocked,
    const double radius_multiplier) const
  {
    int best = -1;
    double best_distance = std::numeric_limits<double>::max();
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
      if (skip_blocked && vertex_blocked_[i]) {
        continue;
      }
      const double distance = (vertices_[i].position - point).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best = static_cast<int>(i);
      }
    }
    return best_distance < step_size_ * radius_multiplier ? best : -1;
  }

  nav_msgs::msg::Path compute_path(const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    int start_id = closest_vertex(start, true, 2.0);
    int goal_id = closest_vertex(goal, true, 2.0);
    if (start_id < 0) {start_id = closest_vertex(start, true, 3.0);}
    if (goal_id < 0) {goal_id = closest_vertex(goal, true, 3.0);}
    if (start_id < 0) {start_id = closest_vertex(start, false, 3.0);}
    if (goal_id < 0) {goal_id = closest_vertex(goal, false, 3.0);}
    if (start_id < 0 || goal_id < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot snap start/goal to the 3D PRM graph");
      return path;
    }
    if (start_id == goal_id) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = vertices_[start_id].position.x();
      pose.pose.position.y = vertices_[start_id].position.y();
      pose.pose.position.z = vertices_[start_id].position.z() + 0.35;
      pose.pose.orientation = goal_pose_.pose.orientation;
      path.poses.push_back(pose);
      return path;
    }

    std::priority_queue<QueueItem> frontier;
    std::vector<double> cost(vertices_.size(), std::numeric_limits<double>::max());
    std::vector<int> parent(vertices_.size(), -1);
    frontier.push(QueueItem{start_id, 0.0});
    cost[start_id] = 0.0;
    while (!frontier.empty()) {
      const int current = frontier.top().id;
      frontier.pop();
      if (current == goal_id) {
        break;
      }
      {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        if (vertex_blocked_[current]) {
          continue;
        }
      }
      for (const auto & edge : vertices_[current].edges) {
        {
          std::lock_guard<std::mutex> lock(obstacle_mutex_);
          if (vertex_blocked_[edge.target_id]) {
            continue;
          }
        }
        const double length =
          (vertices_[edge.target_id].position - vertices_[current].position).norm();
        double edge_cost = length;
        if (use_clearance_penalty_) {
          edge_cost *= 1.0 + clearance_penalty_weight_ *
            std::exp(-edge.min_clearance / clearance_penalty_scale_);
        } else if (use_soft_penalty_) {
          edge_cost *= 1.0 + 3.0 *
            std::exp(-distance_to_boundary(vertices_[edge.target_id].position) / 0.3);
        }
        const double candidate = cost[current] + edge_cost;
        if (candidate < cost[edge.target_id]) {
          cost[edge.target_id] = candidate;
          parent[edge.target_id] = current;
          const double heuristic = (vertices_[edge.target_id].position - goal).norm();
          frontier.push(QueueItem{edge.target_id, candidate + heuristic});
        }
      }
    }
    if (parent[goal_id] < 0) {
      return path;
    }
    std::vector<Eigen::Vector3d> points;
    for (int at = goal_id; at >= 0; at = parent[at]) {
      points.push_back(vertices_[at].position);
      if (at == start_id) {
        break;
      }
    }
    std::reverse(points.begin(), points.end());
    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x();
      pose.pose.position.y = points[i].y();
      pose.pose.position.z = points[i].z() + 0.35;
      pose.pose.orientation.w = 1.0;
      if (i + 3U >= points.size()) {
        pose.pose.orientation = goal_pose_.pose.orientation;
      }
      path.poses.push_back(pose);
    }
    if (!path.poses.empty()) {
      path.poses.push_back(path.poses.back());
    }
    return path;
  }

  Eigen::Vector3d snap_to_traversable(const Eigen::Vector3d & point) const
  {
    return *std::min_element(
      free_centers_.begin(), free_centers_.end(),
      [&point](const auto & a, const auto & b) {
        return (a - point).squaredNorm() < (b - point).squaredNorm();
      });
  }

  static Eigen::Vector3d position_of(const geometry_msgs::msg::PoseStamped & pose)
  {
    return Eigen::Vector3d(
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }

  void snap_pose(geometry_msgs::msg::PoseStamped & pose) const
  {
    const Eigen::Vector3d snapped = snap_to_traversable(position_of(pose));
    pose.pose.position.x = snapped.x();
    pose.pose.position.y = snapped.y();
    pose.pose.position.z = snapped.z();
  }

  bool current_pose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s -> %s: %s", map_frame_.c_str(), robot_frame_.c_str(), error.what());
      return false;
    }
  }

  static double yaw_of(const geometry_msgs::msg::Quaternion & quaternion)
  {
    return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  }

  static double normalized_angle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejected goal in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    if (nav_state_ == NavState::GLOBAL_PLANNING || nav_state_ == NavState::TRACKING ||
      nav_state_ == NavState::GOAL_ALIGN)
    {
      set_nav_state(NavState::ABORTED);
      set_nav_state(NavState::WAITING);
    }
    goal_pose_ = *message;
    goal_pose_.header.frame_id = map_frame_;
    snap_pose(goal_pose_);
    goal_yaw_ = yaw_of(goal_pose_.pose.orientation);
    has_goal_ = true;
    force_replan_ = true;
    last_path_.poses.clear();
    RCLCPP_INFO(
      get_logger(), "Accepted 3D goal (%.2f, %.2f, %.2f)",
      goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
    set_nav_state(NavState::GLOBAL_PLANNING);
  }

  void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() % 3U != 0U) {
      RCLCPP_WARN(get_logger(), "Rejected /obs_raw: data length is not a multiple of three");
      return;
    }
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    dynamic_obstacles_.clear();
    std::fill(vertex_blocked_.begin(), vertex_blocked_.end(), false);
    std::size_t blocked_count = 0;
    for (std::size_t i = 0; i < message->data.size(); i += 3U) {
      const Eigen::Vector3d point(message->data[i], message->data[i + 1U], message->data[i + 2U]);
      if (!point.allFinite()) {
        continue;
      }
      dynamic_obstacles_.insert(voxel_key(point));
      if (obstacle_block_radius_ <= 0.0 || !vertex_tree_) {
        continue;
      }
      pcl::PointXYZ query(
        static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()));
      std::vector<int> indices;
      std::vector<float> distances;
      vertex_tree_->radiusSearch(query, obstacle_block_radius_, indices, distances);
      for (const int index : indices) {
        if (!vertex_blocked_[index]) {
          vertex_blocked_[index] = true;
          ++blocked_count;
        }
      }
    }
    last_obstacle_time_ = now();
    if (nav_state_ == NavState::TRACKING && blocked_count > 0U) {
      force_replan_ = true;
    }
    RCLCPP_INFO(
      get_logger(), "Dynamic obstacles: %zu voxels, %zu PRM vertices blocked",
      dynamic_obstacles_.size(), blocked_count);
  }

  void cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    has_goal_ = false;
    force_replan_ = false;
    last_path_.poses.clear();
    set_nav_state(NavState::ABORTED);
    set_nav_state(NavState::WAITING);
    response->success = true;
    response->message = "Navigation cancelled; planner returned to WAITING";
  }

  void expire_dynamic_obstacles()
  {
    if (last_obstacle_time_.nanoseconds() == 0 || obstacle_timeout_ <= 0.0) {
      return;
    }
    if ((now() - last_obstacle_time_).seconds() <= obstacle_timeout_) {
      return;
    }
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    dynamic_obstacles_.clear();
    std::fill(vertex_blocked_.begin(), vertex_blocked_.end(), false);
    last_obstacle_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void perform_planning(const geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::PoseStamped snapped = pose;
    snap_pose(snapped);
    auto path = compute_path(position_of(snapped), position_of(goal_pose_));
    if (path.poses.empty()) {
      RCLCPP_ERROR(get_logger(), "No collision-free 3D path found");
      has_goal_ = false;
      set_nav_state(NavState::ABORTED);
      return;
    }
    last_path_ = std::move(path);
    last_replan_time_ = now();
    force_replan_ = false;
    path_pub_->publish(last_path_);
    RCLCPP_INFO(get_logger(), "Published 3D path with %zu poses", last_path_.poses.size());
    set_nav_state(NavState::TRACKING);
  }

  bool should_replan() const
  {
    return force_replan_ ||
           (now() - last_replan_time_).seconds() >= 1.0 / replan_frequency_;
  }

  void update()
  {
    pcd_message_.header.stamp = now();
    pcd_pub_->publish(pcd_message_);
    expire_dynamic_obstacles();
    geometry_msgs::msg::PoseStamped pose;
    if (!current_pose(pose)) {
      return;
    }
    current_pose_pub_->publish(pose);
    switch (nav_state_) {
      case NavState::GLOBAL_PLANNING:
        perform_planning(pose);
        break;
      case NavState::TRACKING:
        if (!last_path_.poses.empty()) {
          path_pub_->publish(last_path_);
        }
        if (auto_replan_ && should_replan()) {
          perform_planning(pose);
        } else if ((position_of(goal_pose_) - position_of(pose)).norm() < pos_tolerance_) {
          set_nav_state(NavState::GOAL_ALIGN);
        }
        break;
      case NavState::GOAL_ALIGN: {
          const double position_error = (position_of(goal_pose_) - position_of(pose)).norm();
          const double yaw_error = std::abs(normalized_angle(goal_yaw_ - yaw_of(pose.pose.orientation)));
          if (yaw_error < yaw_tolerance_) {
            has_goal_ = false;
            last_path_.poses.clear();
            set_nav_state(NavState::COMPLETED);
          } else if (position_error > pos_tolerance_exit_) {
            has_goal_ = false;
            last_path_.poses.clear();
            set_nav_state(NavState::ABORTED);
          }
          break;
        }
      default:
        break;
    }
  }

  void publish_static_debug_data()
  {
    pcd_message_.header.stamp = now();
    pcd_pub_->publish(pcd_message_);

    pcl::PointCloud<pcl::PointXYZRGB> safe_cloud;
    safe_cloud.reserve(safe_centers_.size());
    for (const auto & point : safe_centers_) {
      pcl::PointXYZRGB output;
      output.x = static_cast<float>(point.x());
      output.y = static_cast<float>(point.y());
      output.z = static_cast<float>(point.z());
      output.r = 0U;
      output.g = 255U;
      output.b = 0U;
      safe_cloud.push_back(output);
    }
    sensor_msgs::msg::PointCloud2 safe_message;
    pcl::toROSMsg(safe_cloud, safe_message);
    safe_message.header.frame_id = map_frame_;
    safe_message.header.stamp = now();
    safe_pub_->publish(safe_message);

    visualization_msgs::msg::Marker graph;
    graph.header.frame_id = map_frame_;
    graph.header.stamp = now();
    graph.ns = "prm";
    graph.id = 0;
    graph.type = visualization_msgs::msg::Marker::LINE_LIST;
    graph.action = visualization_msgs::msg::Marker::ADD;
    graph.pose.orientation.w = 1.0;
    graph.scale.x = 0.05;
    graph.color.r = 1.0F;
    graph.color.g = 0.7F;
    graph.color.a = 0.6F;
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
      for (const auto & edge : vertices_[i].edges) {
        if (edge.target_id < static_cast<int>(i)) {
          continue;
        }
        geometry_msgs::msg::Point first;
        first.x = vertices_[i].position.x();
        first.y = vertices_[i].position.y();
        first.z = vertices_[i].position.z();
        geometry_msgs::msg::Point second;
        second.x = vertices_[edge.target_id].position.x();
        second.y = vertices_[edge.target_id].position.y();
        second.z = vertices_[edge.target_id].position.z();
        graph.points.push_back(first);
        graph.points.push_back(second);
      }
    }
    graph_pub_->publish(graph);
  }

  void set_nav_state(const NavState state)
  {
    if (nav_state_ == state) {
      return;
    }
    nav_state_ = state;
    std_msgs::msg::UInt8 numeric;
    numeric.data = static_cast<uint8_t>(state);
    nav_state_pub_->publish(numeric);
    std_msgs::msg::String text;
    text.data = nav_state_to_string(state);
    nav_state_debug_pub_->publish(text);
    RCLCPP_INFO(get_logger(), "Navigation state: %s", text.data.c_str());
  }

  std::string pcd_path_;
  std::string map_frame_;
  std::string robot_frame_;
  double edt_xy_expand_{1.0};
  int edt_z_thickness_{3};
  double robot_height_{1.0};
  bool enable_headroom_check_{true};
  double safe_margin_{0.4};
  double voxel_leaf_{0.4};
  double voxel_resolution_{0.4};
  bool use_clearance_penalty_{true};
  double clearance_penalty_weight_{22750.0};
  double clearance_penalty_scale_{0.3};
  bool use_soft_penalty_{false};
  double step_size_{1.0};
  int max_nodes_{10000};
  int k_neigh_{50};
  double max_slope_tan_{1.0};
  bool auto_replan_{true};
  double replan_frequency_{1.0};
  double pos_tolerance_{0.5};
  double pos_tolerance_exit_{2.0};
  double yaw_tolerance_{0.14};
  double obstacle_block_radius_{0.0};
  double obstacle_timeout_{5.0};
  int random_seed_{42};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};

  NavState nav_state_{NavState::WAITING};
  bool has_goal_{false};
  bool force_replan_{false};
  double goal_yaw_{0.0};
  geometry_msgs::msg::PoseStamped goal_pose_;
  nav_msgs::msg::Path last_path_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertex_cloud_;
  std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> vertex_tree_;
  sensor_msgs::msg::PointCloud2 pcd_message_;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> hit_count_;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> raw_hit_count_;
  std::unordered_map<VoxelKey, double, VoxelKeyHash> voxel_margin_;
  std::vector<Eigen::Vector3d> free_centers_;
  std::vector<Eigen::Vector3d> safe_centers_;
  std::vector<Vertex> vertices_;

  mutable std::mutex obstacle_mutex_;
  std::unordered_set<VoxelKey, VoxelKeyHash> dynamic_obstacles_;
  std::vector<bool> vertex_blocked_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr safe_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr nav_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_state_debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Planning3DPrmNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("planning_3d_prm"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
