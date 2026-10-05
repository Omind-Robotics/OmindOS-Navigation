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
