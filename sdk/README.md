# OmindOS Navigation SDK v0.1

This directory defines the public integration boundary for OmindOS Navigation. Robot applications should integrate through ROS 2 topics/TF rather than depend on internal planner or localization implementation details.

## Stable v0.1 integration concepts

Inputs:
- sensor point cloud / LiDAR data
- IMU data
- odometry or platform state where required
- navigation goal

Outputs:
- localization / TF
- map products
- planned path
- navigation state
- standardized motion command

Robot-specific conversion belongs in a platform adapter. Go2-specific code is legacy integration material and is not the OmindOS default interface.

See INTERFACES.md.
