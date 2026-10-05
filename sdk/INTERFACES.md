# OmindOS Navigation v0.1 interface boundary

## Frames
Platform adapters should provide a consistent base frame and sensor transforms. The navigation core must not require a vendor-specific robot frame name.

## Goals
Use standard ROS 2 pose/path semantics wherever possible. 3D goal selection is provided by the RViz2 3D goal tool.

## Localization
FAST-LIO provides the LiDAR-inertial localization chain. Platform adapters are responsible for composing the estimator pose into the robot base frame.

## Planning
The 3D planning layer publishes a path independently of robot-vendor SDKs. Bézier optimization and local planning consume this generic path representation.

## Motion
The final navigation layer should expose a standardized motion command. A platform adapter converts that command to the target robot's native API.

## Status
Navigation state must remain observable independently of the platform adapter so applications can detect running, completed and failed missions.

## Compatibility rule
No OmindOS core package may import a Go2/Unitree SDK directly. Vendor dependencies belong under a platform adapter.
