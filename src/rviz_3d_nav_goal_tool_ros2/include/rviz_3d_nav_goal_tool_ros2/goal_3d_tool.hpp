#ifndef RVIZ_3D_NAV_GOAL_TOOL_ROS2__GOAL_3D_TOOL_HPP_
#define RVIZ_3D_NAV_GOAL_TOOL_ROS2__GOAL_3D_TOOL_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rviz_3d_nav_goal_tool_ros2/pose_3d_tool.hpp"

namespace rviz_common::properties
{
class QosProfileProperty;
class StringProperty;
}

namespace nav3d_rviz_plugins
{

class Goal3DTool : public Pose3DTool
{
  Q_OBJECT

public:
  Goal3DTool();
  ~Goal3DTool() override;
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double yaw) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;
  rclcpp::QoS qos_profile_{5};
};

}  // namespace nav3d_rviz_plugins

#endif  // RVIZ_3D_NAV_GOAL_TOOL_ROS2__GOAL_3D_TOOL_HPP_
