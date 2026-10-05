# 部署与验收

## Docker（推荐）

要求 Docker Engine 和 Compose v2。镜像基于官方 ROS 2 Humble ros-base，容器内完成 colcon
构建、算法测试和闭环仿真。

```bash
git clone -b ros2-native-rebuild-20260817 https://github.com/Chen-lvping/Nav_3d.git
cd Nav_3d
docker compose build
docker compose run --rm nav3d
cat artifacts/demo_result.json
```

返回码为 0 且 JSON 中 `status` 为 `PASS` 才视为通过。输出文件：

```text
artifacts/
  demo_result.json   # 量化验收结果
  demo_map.pgm       # 运行时生成的占据栅格
  demo_map.yaml      # ROS 地图元数据
  demo.log           # 所有 ROS 2 节点日志
```

Compose 默认使用 host networking 和 `ROS_DOMAIN_ID=42`，便于 ROS 2 DDS 在 Linux 主机上
工作。并行运行多个工程时可修改域：

```bash
ROS_DOMAIN_ID=73 docker compose run --rm nav3d
```

## Ubuntu 22.04 / ROS 2 Humble 本机运行

```bash
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh
source install/setup.bash
./scripts/validate_native.sh artifacts-host
```

只启动演示而不重复测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_demo.sh   map_output:=/tmp/nav3d_map   result_file:=/tmp/nav3d_result.json
```

## 分阶段启动

```bash
ros2 launch nav3d_native mapping.launch.py
ros2 launch nav3d_native localization.launch.py
ros2 launch nav3d_native navigation.launch.py
```

这些 launch 文件只启动算法节点。实机或外部仿真必须按 `INTERFACES.md` 提供扫描、里程计、TF
和目标。建图节点默认位姿话题是 `/ground_truth/odom`，实机使用时应通过 ROS 2 参数覆盖：

```bash
ros2 run nav3d_native mapper --ros-args   -p pose_topic:=/lio/odom   -p map_output:=/data/site_a
```

## 验收规则

`demo_supervisor` 最长等待 45 秒，并同时要求：

- 占据单元不少于 150；
- 自由单元不少于 2500；
- 路径不少于 3 个位姿；
- 定位平面误差小于 0.65 m；
- 机器人终点误差小于 0.45 m。

2026-08-17 的最终 Docker 回归结果为：6/6 单元测试通过、定位误差 0.050 m、终点误差
0.384 m、路径 29 个位姿，状态 `PASS`。数值会随调度有小幅变化，应以阈值和退出码为准。

## ROS 1 旧工程

默认 `docker/Dockerfile` 和 `scripts/build_workspace.sh` 只构建 ROS 2 包。若确实需要历史 ROS 1
环境，可参考 `docker/Dockerfile.ros1-legacy` 和 `scripts/build_legacy_ros1.sh`；它们不属于本分支
的原生 ROS 2 验收链。
