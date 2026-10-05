#include "rviz_3d_nav_goal_tool_ros2/goal_3d_tool.hpp"

#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace nav3d_rviz_plugins
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/goal_pose",
    "The PoseStamped topic on which to publish 3D navigation goals.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

Goal3DTool::~Goal3DTool() = default;

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {qos_profile_ = profile;});
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  const auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = node->get_clock();
}

void Goal3DTool::onPoseSet(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.z = std::sin(yaw * 0.5);
  goal.pose.orientation.w = std::cos(yaw * 0.5);

  RVIZ_COMMON_LOG_INFO_STREAM(
    "3D navigation goal: frame=" << goal.header.frame_id <<
      " xyz=(" << x << ", " << y << ", " << z << ") yaw=" << yaw);
  publisher_->publish(goal);
}

}  // namespace nav3d_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nav3d_rviz_plugins::Goal3DTool, rviz_common::Tool)
