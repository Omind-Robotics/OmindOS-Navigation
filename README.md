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

### Robot tuning workbench

The [local 3D tuning workbench](docs/ROBOT_WORKBENCH.zh-CN.md) loads URDF,
edits robot/drive parameters, computes forward kinematics and fixed-base static
gravity loads, previews velocity-limited joint trajectories, and saves versioned
profiles. Start with `python3 scripts/robot_workbench.py --store ./robot-configurations`.
This is a kinematic preview, not collision, balance, full dynamics or hardware acceptance.

## ROS 2 profile preview integration

The workbench can now explicitly load a saved profile into a ROS 2 joint-preview node,
run a bounded kinematic trajectory, stop, and render the node's reported positions.
See [ROS 2 参数加载与工作台联动](docs/ROS2_PROFILE_RUNTIME.zh-CN.md).
The loaded target is `ros2_joint_preview`; this does not activate a hardware driver
or validate quadruped gait, balance, or physical stopping.

## 四足闭环控制与 RS04 驱动（开发预览）

新增 MEVIUS2 姿态/关节反馈策略、浮动机身 MuJoCo 验证及 ROS 2 运动接口。默认采用 [通用关节目标角度接口](docs/JOINT_POSITION_INTERFACE.zh-CN.md)，由仿真关节执行闭环；原 ROBSTRIDE04 SocketCAN 扭矩驱动保留为显式对照测试。用户已确认控制板为 [DM-MC02](docs/DM_MC02_INTEGRATION.zh-CN.md)，板卡适配尚未实现，RS04 实机驱动及客户整机步态、平衡仍待验收。本增量在开发 PR #4 中；不代表当前公开下载包已经包含四足控制。见 [接入说明](docs/QUADRUPED_RS04_RUNTIME.zh-CN.md) 与 [通用接口验证记录](docs/validation/joint-position-20261008.md)。
