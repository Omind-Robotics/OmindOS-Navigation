# ROS 2 接口契约

## 核心话题

| 话题 | 类型 | 生产者 | 消费者 | 约束 |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 仿真或激光前端 | mapper、localizer | `frame_id=base_scan` 或提供等价外参 |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | 仅测试仿真 | mapper、supervisor | 只用于演示建图和量化验收 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | 仿真或底盘里程计 | localizer | 位姿在 `odom` 坐标系连续 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | mapper | localizer、planner、supervisor | frame 为 `map`，transient-local |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | localizer | planner、controller、supervisor | 位姿在 `map` 坐标系 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 用户或 supervisor | planner | 目标在 `map` 坐标系 |
| `/plan` | `nav_msgs/msg/Path` | planner | controller、supervisor | 路径在 `map` 坐标系，transient-local |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | controller | 仿真或底盘桥 | 当前算法使用 `linear.x`、`angular.z` |
| `/nav3d/demo_result` | `std_msgs/msg/String` | supervisor | 可选监控 | JSON 状态，仅测试使用 |

话题可使用 ROS 2 标准 remapping 适配现有系统，例如：

```bash
ros2 run nav3d_native localizer --ros-args   -r /scan:=/lidar/scan   -r /wheel/odom:=/base/odom
```

## TF2 契约

| 变换 | 发布者 | 要求 |
|---|---|---|
| `map -> odom` | `nav3d_localizer` | 扫描匹配得到的全局校正 |
| `odom -> base_link` | 仿真或底盘里程计 | 连续、时间戳与扫描兼容 |
| `base_link -> base_scan` | 仿真或机器人描述/静态发布器 | 正确的激光外参 |

实机系统必须形成单一连通树，且不能重复发布相同父子变换。

## 参数

| 节点 | 参数 | 默认值 | 说明 |
|---|---|---|---|
| mapper | `pose_topic` | `/ground_truth/odom` | 建图使用的全局位姿来源 |
| mapper | `map_output` | `/tmp/nav3d_demo_map` | 不含扩展名的地图输出路径 |
| simulator | `mapping_phase_seconds` | `5.0` | 演示扫描覆盖阶段时长 |
| simulator | `scan_rate` | `10.0` | 仿真扫描/里程计频率 Hz |
| supervisor | `mapping_phase_seconds` | `5.0` | 发送导航目标前的建图时长 |
| supervisor | `result_file` | `/tmp/nav3d_demo_result.json` | 验收 JSON 路径 |

算法常量目前位于 `algorithms.py` 和节点实现中，后续实机标定时再分组参数化；当前值由确定性回归
测试锁定，避免未经验证的配置漂移。

## 实机底盘桥要求

底盘桥只消费 `/cmd_vel`，不应把厂商 SDK 引入算法包。最少需要：

1. 对线速度、角速度和加速度限幅；
2. 命令超时自动发送零速度；
3. 退出、异常和急停时发送零速度；
4. 发布 `/wheel/odom` 和 `odom -> base_link`，或接入独立里程计来源；
5. 提供独立使能/急停门控。

在运动测试前可检查：

```bash
ros2 topic info /cmd_vel
ros2 topic echo --once /localization/pose
ros2 run tf2_ros tf2_echo map base_link
```
