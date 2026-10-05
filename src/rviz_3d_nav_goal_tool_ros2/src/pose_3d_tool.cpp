#include "rviz_3d_nav_goal_tool_ros2/pose_3d_tool.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <OgreQuaternion.h>
#include <OgreSceneNode.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"

namespace nav3d_rviz_plugins
{

Pose3DTool::Pose3DTool()
: rviz_common::Tool()
{
  projection_finder_ =
    std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

Pose3DTool::~Pose3DTool() = default;

void Pose3DTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 2.0F, 0.2F, 0.5F, 0.35F);
  arrow_->setColor(0.0F, 1.0F, 0.0F, 1.0F);
  arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate()
{
  setStatus(
    "Left-drag: set XY/yaw. Hold right button while dragging: set Z.");
  state_ = State::Position;
}

void Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
  clearHeightArrows();
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  const auto projection =
    projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  int flags = 0;

  if (event.leftDown() && projection.first) {
    position_ = projection.second;
    arrow_->setPosition(position_);
    state_ = State::Orientation;
    flags |= Render;
  } else if (event.type == QEvent::MouseMove && event.left()) {
    if (state_ == State::Orientation && projection.first) {
      yaw_ = std::atan2(
        projection.second.y - position_.y,
        projection.second.x - position_.x);
      updateArrowOrientation();
      if (event.right()) {
        state_ = State::Height;
        initial_z_ = position_.z;
        previous_mouse_y_ = event.y;
      }
      flags |= Render;
    } else if (state_ == State::Height) {
      constexpr double height_pixels_per_meter = 50.0;
      const double mouse_y = event.y;
      position_.z -= (mouse_y - previous_mouse_y_) / height_pixels_per_meter;
      previous_mouse_y_ = mouse_y;
      arrow_->setPosition(position_);
      updateHeightArrows();
      flags |= Render;
    }
  } else if (event.leftUp() &&
    (state_ == State::Orientation || state_ == State::Height))
  {
    clearHeightArrows();
    onPoseSet(position_.x, position_.y, position_.z, yaw_);
    flags |= Finished | Render;
  }

  return flags;
}

void Pose3DTool::clearHeightArrows()
{
  height_arrows_.clear();
}

void Pose3DTool::updateArrowOrientation()
{
  arrow_->getSceneNode()->setVisible(true);
  const Ogre::Quaternion arrow_axis(
    Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
  arrow_->setOrientation(
    Ogre::Quaternion(Ogre::Radian(yaw_), Ogre::Vector3::UNIT_Z) * arrow_axis);
}

void Pose3DTool::updateHeightArrows()
{
  constexpr double height_interval = 0.5;
  clearHeightArrows();
  const int count = static_cast<int>(
    std::ceil(std::abs(initial_z_ - position_.z) / height_interval));
  const double direction = initial_z_ > position_.z ? -1.0 : 1.0;
  const Ogre::Quaternion arrow_axis(
    Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

  for (int index = 0; index < count; ++index) {
    auto marker = std::make_shared<rviz_rendering::Arrow>(
      scene_manager_, nullptr, 0.5F, 0.1F, 0.0F, 0.1F);
    marker->setColor(0.0F, 1.0F, 0.0F, 1.0F);
    Ogre::Vector3 marker_position = position_;
    marker_position.z = initial_z_ + direction * index * height_interval;
    marker->setPosition(marker_position);
    marker->setOrientation(
      Ogre::Quaternion(Ogre::Radian(yaw_), Ogre::Vector3::UNIT_Z) *
      arrow_axis);
    marker->getSceneNode()->setVisible(true);
    height_arrows_.push_back(marker);
  }
}

}  // namespace nav3d_rviz_plugins
