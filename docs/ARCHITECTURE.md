# 原生 ROS 2 架构

## 目标与边界

当前默认运行链是一个独立的 ROS 2 Humble 算法栈，不依赖 ROS 1、`roscore`、catkin、
rosbridge 或 ROS 1 API 兼容层。阶段目标是在无实机条件下，把建图、定位、全局规划和速度控制
组成可重复验收的闭环。旧工程保留在 `src/3D_NAV`，但由 `COLCON_IGNORE` 从默认 colcon
工作区隔离。

本阶段使用二维占据栅格验证算法和 ROS 2 接口。3D 激光/LIO、底盘 SDK、点云地形语义等实机
前端属于下一阶段接入边界，不会混入当前算法核心。

## 数据流

```text
                         原生 ROS 2 话题 / TF

simulator or hardware
  ├─ /scan ----------------------┬──────────────> mapper
  ├─ mapping pose --------------┘                    │
  ├─ /wheel/odom --------┐                            └─ /map + PGM/YAML
  ├─ odom -> base_link TF│
  └─ /cmd_vel <----------┼------- controller <--------- /plan
                         │                                  ^
/map + /scan + wheel odom└----> localizer ------------------┤
                              ├─ /localization/pose         │
                              └─ map -> odom TF             │
                                                           │
/map + localized pose + /goal_pose ----> planner ----------┘
```

Docker 验收时还会启动 `demo_supervisor`。它发送目标、订阅地图/定位/路径/仿真真值，并把量化
结果写入 `demo_result.json`。真值只用于测试判定，定位、规划和控制节点不订阅真值。

## 算法分层

`src/nav3d_native/nav3d_native/algorithms.py` 不依赖 ROS 消息，可直接单元测试：

| 模块 | 实现 |
|---|---|
| 坐标变换 | 严格 SE(2) 组合、求逆和坐标系变换 |
| 建图 | log-odds 占据栅格、Bresenham 射线更新 |
| 定位 | 里程计预测 + 相关扫描匹配 + `map -> odom` 平滑校正 |
| 规划 | 障碍膨胀、八邻域 A*、端点安全释放、路径简化 |
| 控制 | Pure Pursuit 跟踪，输出线速度和角速度 |

ROS 适配层由六个 `rclpy` 节点组成：

| 节点 | 责任 |
|---|---|
| `nav3d_mapper` | 将位姿和 `LaserScan` 融合为 `OccupancyGrid`，保存 PGM/YAML |
| `nav3d_localizer` | 扫描匹配定位，发布定位位姿和 TF2 校正 |
| `nav3d_planner` | 在膨胀后的地图上执行 A* 并发布 `nav_msgs/Path` |
| `nav3d_controller` | 跟踪路径并发布 `/cmd_vel`，退出时发布零速度 |
| `nav3d_simulator` | 无实机确定性传感器和差速运动学闭环，仅用于验收 |
| `nav3d_demo_supervisor` | 发送目标、检查指标并输出 PASS/FAIL 证据 |

## 坐标系约定

```text
map --(localizer)--> odom --(simulator or base driver)--> base_link
                                                     └--> base_scan (static)
```

- `map`：全局地图坐标系。
- `odom`：连续但允许漂移的局部里程计坐标系。
- `base_link`：机器人基座。
- `base_scan`：激光坐标系。

实机接入时不得同时由多个节点发布同一条 TF。定位节点负责 `map -> odom`，底盘或里程计节点负责
`odom -> base_link`，传感器外参使用静态 TF。

## QoS 与进程边界

`/map` 和 `/plan` 使用 reliable + transient-local，后启动的定位、规划或控制节点仍可收到最新值。
高频扫描、里程计、定位和速度命令使用普通深度队列。所有核心节点均可独立启动，也可以由 launch
组合；容器中没有 ROS 1 守护进程或桥接进程。
