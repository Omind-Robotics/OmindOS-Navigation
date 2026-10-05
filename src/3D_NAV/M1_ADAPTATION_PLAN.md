# 智身 M1 适配 3D_NAV 方案

## 1. 结论先行

可以适配，但不要直接把 `M1` 当成当前 `Go2` 的等价替身。

当前仓库已经具备完整的上层导航链路：

- 全局规划：`planning_3d`
- 路径平滑：`bezier_path_optimizer`
- 局部规划/控制：`nmpc_planner`
- 在线障碍：`obstacle_processor`

真正需要替换的是下面三层：

1. `底盘控制桥接层`
2. `机器人状态/TF桥接层`
3. `传感器与定位接入层`

其中第 1 层必须做，第 2 层强烈建议做，第 3 层决定这个项目能不能“完整跑通”。

## 2. 当前项目对机器人底层的真实要求

从本仓库现状看，上层导航对底盘和传感器的要求并不复杂，但接口要求很明确。

### 2.1 控制接口

当前局部控制最终只输出一个 ROS1 话题：

- `/cmd_vel`：`geometry_msgs/Twist`

现有 `Go2` 适配层只是一个桥接器：

- 订阅 `/cmd_vel`
- 转成底盘 SDK 控制命令
- 急停时停止

也就是说，`M1` 只要能稳定吃下 `/cmd_vel`，规划器本身不用重写。

### 2.2 TF / 位姿接口

当前项目要求至少存在：

- `map -> base_link`

局部规划和全局规划都依赖这条 TF。

### 2.3 点云与障碍物接口

当前障碍物模块默认依赖：

- 输入点云：`/points_e1r_front`
- 地图点云：`/map`

但它支持 remap，所以不是硬编码死要求，只要你能提供实时点云即可。

## 3. 智身官方资料给出的关键信息

基于 `zsibot` 官方仓库，目前和本项目最相关的是两块：

### 3.1 `genisom_robot_sdk`

SDK 公开了高层控制与状态接口，关键点是：

- 开发环境主推 `Ubuntu 22.04 + GCC 11.4 + C++`
- 默认连机器人地址：`192.168.234.1:8082`
- 支持 `WebSocket` 和 `UDP`
- `Move(left_right, forward_back, yaw)` 控制范围是 `[-1.0, 1.0]`
- `Move` 是按速度档位限幅，不是直接吃 `m/s`
- 支持 `TakeControl()` / `ReleaseControl()`
- 支持 `SoftEmergencyStop()`
- 支持 `StandUp()`
- 支持 `SetMcConfig(true)` 打开 50Hz 运动状态上报
- 支持 `SetImuConfig(freq)` 打开 IMU 上报
- 支持 `SetSpeedReportConfig(true, freq)` 打开速度上报

这意味着：

- `M1` 最适合接成一个 C++ ROS 节点
- 该节点负责把 ROS 的速度指令换算成 SDK 百分比指令
- 同时把 SDK 回调反向发布成 ROS 状态/TF

### 3.2 `genisom_roamerx_open`

智身官方开源导航栈本身是：

- `ROS2 Humble`
- 带 `robot-forward`
- 支持 `UDP / LCM`
- 控制器类型出现了 `RL_TRACK_VELOCITY`

这说明智身自己的官方思路也是：

- 上层导航输出高层速度
- 下层由运动控制器 / RL 控制器负责跟踪速度

对本项目的意义是：

- 你不需要把本仓库改成 RL 项目
- 只需要把本仓库的 `/cmd_vel` 接到 M1 的“速度跟踪入口”

## 4. 最大的不兼容点

这是实际落地前必须先承认的部分。

### 4.1 ROS 版本不一致

本仓库是：

- `ROS1 Noetic`

智身官方开源导航栈是：

- `ROS2 Humble`

所以 `genisom_roamerx_open` 不能直接拉进本工作区复用。

### 4.2 开发环境不一致

本仓库当前文档按 `Noetic` 工作流组织，通常对应 `Ubuntu 20.04`。

智身 SDK 文档主推：

- `Ubuntu 22.04`

这会带来两个风险：

1. 官方预编译 `.so` 在你当前系统上可能直接能用，也可能有 ABI/依赖问题
2. 即使能编译，通过也不代表长期稳定

### 4.3 控制语义不一致

当前 `nmpc_planner` 输出的是物理量语义：

- `linear.x`：前进速度，单位接近 `m/s`
- `linear.y`：侧向速度
- `angular.z`：角速度，单位 `rad/s`

而 `M1 SDK Move()` 接的是归一化速度百分比：

- `left_right in [-1, 1]`
- `forward_back in [-1, 1]`
- `yaw in [-1, 1]`

所以中间必须做标定和限幅映射，不能直接透传。

## 5. 推荐适配架构

推荐采用“最小改造、保留现有导航栈”的方案。

### 5.1 方案总览

```text
planning_3d
  -> bezier_path_optimizer
  -> nmpc_planner
  -> /cmd_vel
  -> m1_base_bridge
  -> genisom_robot_sdk
  -> M1
```

同时补一条状态链：

```text
genisom_robot_sdk callbacks
  -> m1_state_bridge
  -> /odom, /imu, /diagnostics, TF
```

再根据传感器情况补定位链：

```text
LiDAR/IMU
  -> 定位/SLAM
  -> map -> base_link
```

### 5.2 为什么不建议直接复用官方 ROS2 导航栈

因为你的目标不是“跑智身官方导航”，而是“跑本机这个 3D_NAV 项目”。

直接接官方 ROS2 栈会引入：

- ROS1/ROS2 混合部署
- `ros1_bridge`
- `robot-forward`
- 官方消息定义与当前消息/TF 风格差异

工程成本明显高于直接写一个 ROS1 bridge。

## 6. 需要新增的适配模块

## 6.1 `m1_base_bridge`：底盘控制桥

建议新增一个 ROS1 C++ 包，例如：

- `src/3D_NAV/m1_base_controller/`

节点职责：

1. 启动时连接 SDK
2. 调用 `TakeControl()`
3. 确保机器人 `StandUp()`
4. 订阅 `/cmd_vel`
5. 按参数将 ROS 速度映射到 `Move(left_right, forward_back, yaw)`
6. 在控制超时或急停时下发零速
7. 收到 `/emergency_stop` 时调用 `SoftEmergencyStop(true)`
8. 退出前调用 `ReleaseControl()`

### 建议的速度映射

假设参数如下：

- `max_forward_vel_mps`
- `max_lateral_vel_mps`
- `max_yaw_vel_radps`

则：

```text
forward_back = clamp(cmd.linear.x / max_forward_vel_mps, -1.0, 1.0)
left_right   = clamp(cmd.linear.y / max_lateral_vel_mps, -1.0, 1.0)
yaw          = clamp(cmd.angular.z / max_yaw_vel_radps, -1.0, 1.0)
```

注意：

- `SDK Move()` 会维持 1 秒，所以 bridge 必须持续发送
- 控制循环建议 `20Hz ~ 50Hz`
- 超时建议设置成 `0.2s ~ 0.5s`，不要等满 1 秒

## 6.2 `m1_state_bridge`：状态与 TF 桥

建议和 `m1_base_bridge` 做成同一包内两个节点，或一个节点内双线程。

节点职责：

1. 打开 `SetMcConfig(true)`，接收 50Hz `MotionData`
2. 打开 `SetImuConfig(freq)`，发布 IMU
3. 打开 `SetSpeedReportConfig(true, freq)`，发布速度
4. 接收 `OnRobotStateData()`，发布状态诊断
5. 监听 `OnControlLost()` / `OnControlAvailable()`，做安全降级

建议输出：

- `/odom`
- `/imu/data_raw`
- `/m1/robot_state`
- `/m1/faults`
- `odom -> base_link` 或 `body -> base_link`

说明：

- 如果你后面采用外部定位输出 `map -> base_link`，那这里至少也要提供稳定的机器人本体系 TF
- 如果不做这层，调试和故障定位会很困难

## 6.3 `m1_bringup.launch`

建议增加一个单独 launch，把和 M1 有关的桥接统一起来：

- `m1_base_bridge`
- `m1_state_bridge`
- 静态 TF
- 参数文件

## 7. 传感器与定位接入方案

这里决定是否能把 3D_NAV “完整跑通”。

### 7.1 情况 A：M1 上有可用 3D LiDAR + IMU，且能给 ROS1 点云

这是最理想的情况。

可以保留本仓库主链，只改输入层：

1. 替换 `livox_ros_driver2` 为实际传感器驱动
2. 保证存在与真实安装位姿一致的雷达到机体 TF
3. 让定位模块最终发布 `map -> base_link`
4. 将障碍物模块输入 remap 到实际点云话题

如果点云和 IMU 不是 MID360 语义，则：

- 现有 `FAST_LIO`/`open3d_loc` 未必能直接用
- 需要替换成兼容你传感器的 SLAM/Localization

### 7.2 情况 B：M1 只有机身状态，没有可用实时点云

这种情况下，不能说“完整跑通本项目”。

原因是：

- `obstacle_processor` 需要实时点云
- 在线定位链也需要地图定位来源

能做的只是一种降级联调：

1. 用静态 TF 或其他定位来源先喂 `map -> base_link`
2. 打通 `/cmd_vel -> M1`
3. 跑通规划和跟踪
4. 暂时关闭或替代障碍物模块

这只能算“控制链打通”，不算完整实机导航。

### 7.3 情况 C：M1 使用的是官方 RL / 运动控制后端

这种情况下也不用改上层规划。

处理方式仍然是：

- 本项目输出 `/cmd_vel`
- 适配桥将 `/cmd_vel` 送给 M1 的速度控制入口

也就是说，RL 后端只是 `base controller`，不是替代本仓库的规划器。

## 8. 推荐实施顺序

建议按下面顺序做，不要一开始就全链路联调。

### 第 1 步：单独验证 SDK

目标：

- 能连上 `192.168.234.1:8082`
- 能 `TakeControl`
- 能 `StandUp`
- 能 `Move`
- 能收到 `MotionData / RobotStateData`

如果这一步不稳，后面都不用谈。

### 第 2 步：只打通 `/cmd_vel`

目标：

- 写 `m1_base_bridge`
- 手工 `rostopic pub /cmd_vel` 让 M1 运动
- 验证急停和超时停机

这一阶段不用碰规划器。

### 第 3 步：补状态与 TF

目标：

- 发布 `/odom`
- 发布 `/imu`
- 发布状态诊断
- 验证 `base_link` 姿态方向是否正确

### 第 4 步：接入定位

目标：

- 稳定提供 `map -> base_link`

这是 3D_NAV 真正开始能动起来的前提。

### 第 5 步：接入障碍物点云

目标：

- 给 `obstacle_processor` 稳定输入实时点云
- 确认 `/obs_raw` 正常输出

### 第 6 步：跑完整导航链

顺序保持本项目已有流程：

1. 定位
2. `planning_3d`
3. `bezier_path_optimizer`
4. `nmpc_planner`
5. `obstacle_processor`
6. `m1_base_bridge`

## 9. 需要改哪些文件

如果按推荐方案落地，现有项目不需要大改算法包，主要是新增与少量 launch 调整。

### 新增

- `src/3D_NAV/m1_base_controller/`
- `src/3D_NAV/m1_base_controller/src/m1_base_bridge.cpp`
- `src/3D_NAV/m1_base_controller/src/m1_state_bridge.cpp`
- `src/3D_NAV/m1_base_controller/launch/m1_bringup.launch`
- `src/3D_NAV/m1_base_controller/config/m1_params.yaml`

### 可能需要改的现有文件

- `obstacle_processor/launch/obstacle_processor.launch`
  - 改点云话题 remap 和雷达 frame
- `local_planner/nmpc_planner/launch/nmpc_controller.launch`
  - 如 M1 不能稳定侧移，可把 `linear.y` 约束收紧
- `localization/*`
  - 根据你实际传感器替换定位链

## 10. 最关键的参数表

建议把这些参数显式写到 `m1_params.yaml`：

- `robot_ip`
- `robot_port`
- `transport_protocol`
- `connect_timeout_ms`
- `auto_reconnect`
- `control_frequency`
- `cmd_timeout_sec`
- `max_forward_vel_mps`
- `max_lateral_vel_mps`
- `max_yaw_vel_radps`
- `publish_imu`
- `imu_frequency`
- `publish_speed`
- `speed_frequency`
- `publish_mc`
- `body_to_base_link`
- `lidar_to_base_link`

## 11. 风险与现实判断

### 11.1 最大风险

不是控制接口，而是定位与传感器。

只要 M1 能提供：

- 稳定底盘控制
- 稳定机体姿态
- 稳定点云/IMU

这个项目就能接。

反过来，如果没有可用实时点云和定位来源，那么最多只能打通速度控制链，不能完整跑通 3D_NAV。

### 11.2 第二风险

是 SDK 的系统兼容性。

如果 `genisom_robot_sdk` 在当前 `ROS1 Noetic` 机器上因为系统/ABI 不稳定，推荐采用双机或双环境部署：

1. `M1 SDK bridge` 跑在官方推荐环境
2. `3D_NAV` 继续跑在当前环境
3. 两边通过 ROS 网络或自定义桥接交换少量消息

只桥接这些就够：

- `/cmd_vel`
- `/odom`
- `/imu/data_raw`
- `/tf`
- `/emergency_stop`
- 点云话题

## 12. 推荐路线

如果目标是“尽快跑通本项目”，推荐路线只有一条：

1. 不迁移现有规划算法
2. 不直接套官方 ROS2 导航栈
3. 新写一个 `ROS1 + C++ + genisom_robot_sdk` 的 M1 bridge
4. 先打通 `/cmd_vel`
5. 再补 TF/状态
6. 最后解决点云和定位

一句话总结：

> 对 M1 的正确适配方式不是“把 3D_NAV 改成智身官方栈”，而是“把 M1 改造成这个 3D_NAV 期望的底层机器人接口”。
