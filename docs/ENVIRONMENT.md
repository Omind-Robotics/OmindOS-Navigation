# 原生 ROS 2 环境

## 支持基线

- Ubuntu 22.04 / ROS 2 Humble
- Docker Engine 24+ 与 Compose v2
- `amd64`、`arm64`（算法包为纯 Python/rclpy，无架构相关二进制）
- 默认 DDS：Fast DDS

默认构建和运行链只有 ROS 2：`ament_python + colcon + rclpy`。不启动 `roscore`，不使用
`rosbridge`，也不包含伪装 ROS 1 API 的兼容头。原 ROS 1 工程保留在 `src/3D_NAV`，由
`COLCON_IGNORE` 隔离，便于后续逐个迁移实机驱动。

## Docker 一键验收

```bash
docker compose build
docker compose run --rm nav3d
```

验收结果写入 `artifacts/`：

- `demo_result.json`：建图、定位误差、规划路径和到点结果
- `demo_map.pgm` / `demo_map.yaml`：容器内生成的地图
- `demo.log`：完整 ROS 2 节点日志

## 本机 Humble

```bash
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh
source install/setup.bash
./scripts/validate_native.sh
```

## 实机接入边界

仿真节点只负责提供标准接口。接入实机时替换它即可：

| 输入/输出 | 类型 | 说明 |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 激光数据；3D 雷达可先投影或增加 PointCloud2 前端 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | 连续局部里程计 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令 |
| `odom -> base_link` | TF2 | 局部连续坐标变换 |

建图时 `pose_topic` 默认使用仿真的 `/ground_truth/odom`；实机应改为 LIO/SLAM 输出。定位、规划、
控制节点本身不订阅真值话题。
