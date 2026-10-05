#ifndef RVIZ_3D_NAV_GOAL_TOOL_ROS2__POSE_3D_TOOL_HPP_
#define RVIZ_3D_NAV_GOAL_TOOL_ROS2__POSE_3D_TOOL_HPP_

#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace rviz_common
{
class ViewportMouseEvent;
}

namespace rviz_rendering
{
class Arrow;
}

namespace nav3d_rviz_plugins
{

class Pose3DTool : public rviz_common::Tool
{
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double yaw) = 0;

private:
  enum class State
  {
    Position,
    Orientation,
    Height
  };

  void clearHeightArrows();
  void updateArrowOrientation();
  void updateHeightArrows();

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> height_arrows_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
  State state_{State::Position};
  Ogre::Vector3 position_;
  double yaw_{0.0};
  double initial_z_{0.0};
  double previous_mouse_y_{0.0};
};

}  // namespace nav3d_rviz_plugins

#endif  // RVIZ_3D_NAV_GOAL_TOOL_ROS2__POSE_3D_TOOL_HPP_
