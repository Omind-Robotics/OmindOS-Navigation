#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kEpsilon = 1.0e-6;

double distance_3d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

geometry_msgs::msg::Point bezier(
  const std::vector<geometry_msgs::msg::Point> & control_points, const double t)
{
  auto work = control_points;
  for (std::size_t level = 1; level < work.size(); ++level) {
    for (std::size_t i = 0; i + level < work.size(); ++i) {
      work[i].x = (1.0 - t) * work[i].x + t * work[i + 1U].x;
      work[i].y = (1.0 - t) * work[i].y + t * work[i + 1U].y;
      work[i].z = (1.0 - t) * work[i].z + t * work[i + 1U].z;
    }
  }
  return work.front();
}
}  // namespace

class BezierPathOptimizerNode final : public rclcpp::Node
{
public:
  BezierPathOptimizerNode()
  : Node("bezier_path_optimizer")
  {
    sample_rate_ = declare_parameter<int>("sample_rate", 20);
    q1_scale_ = declare_parameter<double>("q1_scale", 0.3);
    q1_max_ratio_ = declare_parameter<double>("q1_max_ratio", 1.0);
    enable_orientation_ = declare_parameter<bool>("enable_orientation", true);
    end_strategy_ = declare_parameter<std::string>("end_strategy", "keep_original");
    validation_pcd_path_ = declare_parameter<std::string>("validation_pcd_path", "");
    path_height_offset_ = declare_parameter<double>("path_height_offset", 0.35);
    max_traversable_distance_ = declare_parameter<double>("max_traversable_distance", 0.45);
    if (sample_rate_ < 1 || q1_scale_ < 0.0 || q1_max_ratio_ < 0.0) {
      throw std::runtime_error("invalid Bezier sampling parameters");
    }
    if (!validation_pcd_path_.empty()) {
      load_validation_map();
    }
    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path_smooth", path_qos);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", rclcpp::QoS(1).reliable(),
      std::bind(&BezierPathOptimizerNode::path_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "ROS 2 3D Bezier path optimizer ready");
  }

private:
  geometry_msgs::msg::Point compute_q1(
    const geometry_msgs::msg::Point & p4,
    const geometry_msgs::msg::Point & p5,
    const geometry_msgs::msg::Point & q2) const
  {
    geometry_msgs::msg::Point direction;
    direction.x = p5.x - p4.x;
    direction.y = p5.y - p4.y;
    direction.z = p5.z - p4.z;
    const double original_length =
      std::hypot(std::hypot(direction.x, direction.y), direction.z);
    const double length = std::max(original_length, kEpsilon);
    double extension = q1_scale_ * length;
    extension = std::min(extension, q1_max_ratio_ * length);

    geometry_msgs::msg::Point q1;
    q1.x = p5.x + extension * direction.x / length;
    q1.y = p5.y + extension * direction.y / length;
    q1.z = p5.z + extension * direction.z / length;
    if (distance_3d(q1, p5) >= distance_3d(q1, q2)) {
      const double extra = distance_3d(q1, p5) - distance_3d(q1, q2) + 0.01;
      q1.x += extra * direction.x / length;
      q1.y += extra * direction.y / length;
      q1.z += extra * direction.z / length;
    }
    return q1;
  }

  void set_tangent_orientation(
    geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Point & from,
    const geometry_msgs::msg::Point & to) const
  {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double yaw = std::hypot(dx, dy) < kEpsilon ? 0.0 : std::atan2(dy, dx);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (message->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty planned path");
      return;
    }
    if (have_last_stamp_ &&
      message->header.stamp.sec == last_stamp_sec_ &&
      message->header.stamp.nanosec == last_stamp_nanosec_)
    {
      return;
    }
    have_last_stamp_ = true;
    last_stamp_sec_ = message->header.stamp.sec;
    last_stamp_nanosec_ = message->header.stamp.nanosec;
    std::vector<geometry_msgs::msg::Point> raw;
    raw.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      raw.push_back(pose.pose.position);
    }

    std::vector<geometry_msgs::msg::Point> smoothed;
    std::size_t segment_start = 0U;
    std::size_t point_count = raw.size();
    while (segment_start + 5U < point_count) {
      const std::vector<geometry_msgs::msg::Point> controls(
        raw.begin() + static_cast<std::ptrdiff_t>(segment_start),
        raw.begin() + static_cast<std::ptrdiff_t>(segment_start + 6U));
      for (int sample = 0; sample <= sample_rate_; ++sample) {
        smoothed.push_back(bezier(controls, static_cast<double>(sample) / sample_rate_));
      }
      const auto q1 = compute_q1(
        controls[4], controls[5],
        segment_start + 6U < point_count ? raw[segment_start + 6U] : raw.back());
      raw.insert(raw.begin() + static_cast<std::ptrdiff_t>(segment_start + 6U), q1);
      point_count = raw.size();
      segment_start += 6U;
    }
    if (segment_start < point_count) {
      const std::vector<geometry_msgs::msg::Point> controls(
        raw.begin() + static_cast<std::ptrdiff_t>(segment_start), raw.end());
      for (int sample = 0; sample <= sample_rate_; ++sample) {
        smoothed.push_back(
          controls.size() == 1U ? controls.front() :
          bezier(controls, static_cast<double>(sample) / sample_rate_));
      }
    }

    nav_msgs::msg::Path output;
    output.header = message->header;
    output.poses.reserve(smoothed.size());
    for (std::size_t i = 0; i < smoothed.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position = smoothed[i];
      if (!enable_orientation_) {
        pose.pose.orientation.w = 1.0;
      } else if (i + 1U < smoothed.size()) {
        set_tangent_orientation(pose, smoothed[i], smoothed[i + 1U]);
      } else if (end_strategy_ == "keep_original") {
        pose.pose.orientation = message->poses.back().pose.orientation;
      } else if (smoothed.size() > 1U) {
        set_tangent_orientation(pose, smoothed[i - 1U], smoothed[i]);
      } else {
        pose.pose.orientation.w = 1.0;
      }
      output.poses.push_back(pose);
    }
    if (!smooth_path_is_traversable(output)) {
      RCLCPP_ERROR(
        get_logger(), "Bezier path left the traversable PCD; publishing the raw safe path");
      path_pub_->publish(*message);
      return;
    }
    path_pub_->publish(output);
    RCLCPP_INFO(
      get_logger(), "Smoothed 3D path from %zu to %zu poses",
      message->poses.size(), output.poses.size());
  }

  void load_validation_map()
  {
    auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(validation_pcd_path_, *raw) < 0 || raw->empty()) {
      throw std::runtime_error("unable to load validation PCD: " + validation_pcd_path_);
    }
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(raw);
    filter.setLeafSize(0.2F, 0.2F, 0.2F);
    validation_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filter.filter(*validation_cloud_);
    validation_tree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
    validation_tree_->setInputCloud(validation_cloud_);
    RCLCPP_INFO(
      get_logger(), "Loaded %zu points for smooth-path traversability validation",
      validation_cloud_->size());
  }

  bool smooth_path_is_traversable(const nav_msgs::msg::Path & path) const
  {
    if (!validation_tree_) {
      return true;
    }
    const float distance_limit = static_cast<float>(
      max_traversable_distance_ * max_traversable_distance_);
    for (const auto & pose : path.poses) {
      pcl::PointXYZ query(
        static_cast<float>(pose.pose.position.x),
        static_cast<float>(pose.pose.position.y),
        static_cast<float>(pose.pose.position.z - path_height_offset_));
      std::vector<int> index(1);
      std::vector<float> squared_distance(1);
      if (validation_tree_->nearestKSearch(query, 1, index, squared_distance) != 1 ||
        squared_distance[0] > distance_limit)
      {
        return false;
      }
    }
    return true;
  }

  int sample_rate_{20};
  double q1_scale_{0.3};
  double q1_max_ratio_{1.0};
  bool enable_orientation_{true};
  std::string end_strategy_{"keep_original"};
  std::string validation_pcd_path_;
  double path_height_offset_{0.35};
  double max_traversable_distance_{0.45};
  bool have_last_stamp_{false};
  int32_t last_stamp_sec_{0};
  uint32_t last_stamp_nanosec_{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr validation_cloud_;
  std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> validation_tree_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BezierPathOptimizerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bezier_path_optimizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
