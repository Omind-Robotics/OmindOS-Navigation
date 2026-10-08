# OmindOS Navigation

**OmindOS Navigation v0.1** is the ROS 2 Humble navigation stack maintained for the OmindOS robotics platform.

## v0.1 scope
- Livox / LiDAR sensor integration
- FAST-LIO ROS 2 localization and mapping
- 3D PRM planning and EDT environment representation
- Bezier path optimization
- NMPC local planning/control with the existing fallback tracker
- RViz2 3D navigation goal tools
- ROS 2 native validation stack

## Platform boundary
The navigation core is platform-independent. Existing Go2-specific files are retained only as legacy/platform-adapter material and are not the default OmindOS target.

## Main ROS 2 packages
- nav3d_native
- FAST_LIO_ROS2
- livox_ros_driver2
- planning_3d_ros2
- nmpc_planner_ros2
- rviz_3d_nav_goal_tool_ros2
- nav3d_hardware_bringup (legacy/platform integration)

See docs/OMINDOS_V0_1.md.

## Unified wheeled / quadruped preview

[Published 0.2.0-preview.1: download and installation](https://github.com/maganrobotics-boop/OmindOS-Navigation-Release/releases/tag/v0.2.0-preview.1)
— [发行记录](docs/UNIFIED_RELEASE.zh-CN.md).

The development preview uses one navigation core and selectable `wheeled_diff`,
`wheeled_omni`, and `quadruped` body-velocity profiles. See
[统一平台预览说明（中文）](docs/UNIFIED_PLATFORM_PREVIEW.zh-CN.md).
It adds a guarded simulation command boundary; it is not a hardware release,
a quadruped gait controller, or evidence that the C++ hardware stack is validated.
Existing v0.1 tags and published assets are unchanged.

## Unified customer release work

The next customer version keeps one core for quadruped, differential, and
omnidirectional bases, with quadruped integration first. See
[客户版接入检查与交付范围](docs/UNIFIED_CUSTOMER_RELEASE.zh-CN.md).
The configuration checker and a pinned MEVIUS2 headless locomotion validation
are available now. See [MEVIUS2 adaptation and simulation evidence](docs/MEVIUS2_INTEGRATION.zh-CN.md).
Customer setup follows [URDF, motor model, then other parameters](docs/CUSTOMER_ROBOT_CONFIGURATION.zh-CN.md),
with a per-robot configuration record and update history.
The [robot parameter API](docs/ROBOT_PARAMETER_API.zh-CN.md) accepts URDF and joint
drive parameters, validates and versions them, and supports retrieval by profile ID.
It currently registers configuration only; controller activation is pending.
The customer quadruped is designed with reference to MEVIUS2; the original
model/policy passed a short stand/walk/stop simulation. ROS 2 locomotion integration,
the customer-specific model and physical acceptance remain pending.
This does not promote the preview to a hardware release.
