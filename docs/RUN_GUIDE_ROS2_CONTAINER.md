# Nav_3d ROS 2 容器运行手册

本文只适用于当前工程：

```text
/home/nuc/Nav_3d-ros2-native-rebuild-20260817
```

使用已经载入的镜像：

```text
nav3d:ros2-humble-native-20260817-82dfe36
```

常驻容器名为 `Nav_3d_ros2`。当前仿真阶段使用专用 Docker bridge 网络
`nav_3d_ros2_isolated` 和 `ROS_DOMAIN_ID=142`，避免 ROS 2 DDS 影响宿主机或其他容器。

## 1. 容器日常操作

查看状态：

```bash
docker ps -a --filter 'name=^/Nav_3d_ros2$'
```

启动和停止：

```bash
docker start Nav_3d_ros2
docker stop Nav_3d_ros2
```

进入容器时必须经过镜像入口脚本，以自动加载 ROS 2 和镜像内工作空间：

```bash
docker exec -it Nav_3d_ros2 /ros_entrypoint_nav3d.sh bash
```

普通的 `docker exec -it Nav_3d_ros2 bash` 不会重新执行入口脚本，因此可能找不到
`ros2` 命令。

## 2. 当前容器配置

当前容器已经创建，无需重复执行本节命令。只有明确需要重建该容器时才使用：

```bash
docker network inspect nav_3d_ros2_isolated >/dev/null 2>&1 || \
  docker network create --driver bridge \
    --label project=Nav_3d_ros2 nav_3d_ros2_isolated

docker run -d \
  --name Nav_3d_ros2 \
  --restart unless-stopped \
  --init \
  --network nav_3d_ros2_isolated \
  --user 1000:1000 \
  --workdir /workspace/Nav_3d \
  -e ROS_DOMAIN_ID=142 \
  -e HOME=/tmp/nav3d_home \
  -e ROS_LOG_DIR=/tmp/nav3d_ros_logs \
  --mount type=bind,src=/home/nuc/Nav_3d-ros2-native-rebuild-20260817,dst=/workspace/Nav_3d \
  --mount type=bind,src=/home/nuc/Nav_3d-ros2-native-rebuild-20260817/artifacts,dst=/artifacts \
  nav3d:ros2-humble-native-20260817-82dfe36 \
  bash -lc 'mkdir -p /tmp/nav3d_home /tmp/nav3d_ros_logs && exec sleep infinity'
```

不要把当前源码覆盖挂载到 `/opt/nav3d_ws`。该目录保留镜像内提交 `82dfe36` 的已构建
基线，当前源码固定挂载到 `/workspace/Nav_3d`。

## 3. 构建当前源码

进入容器后执行：

```bash
cd /workspace/Nav_3d
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh
source install/setup.bash
```

构建脚本只选择 `nav3d_native`。历史 ROS 1 工程 `src/3D_NAV` 由其中的
`COLCON_IGNORE` 隔离，不进入 ROS 2 构建。

## 4. 自动验收

进入容器后执行：

```bash
cd /workspace/Nav_3d
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/validate_native.sh /artifacts/source-current
```

验收依次运行 Flake8、单元测试和完整仿真闭环。成功时命令返回 0，且：

```bash
python3 -m json.tool /artifacts/source-current/demo_result.json
```

应显示 `"status": "PASS"`。验收阈值为：

- 占用单元不少于 150；
- 自由单元不少于 2500；
- 路径不少于 3 个位姿；
- 定位误差小于 0.65 m；
- 终点误差小于 0.45 m。

输出文件位于宿主机：

```text
/home/nuc/Nav_3d-ros2-native-rebuild-20260817/artifacts/source-current/
  demo_result.json
  demo_map.pgm
  demo_map.yaml
  demo.log
```

## 5. 手动启动闭环

进入容器并加载当前工作空间后执行：

```bash
cd /workspace/Nav_3d
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav3d_native demo.launch.py \
  map_output:=/artifacts/manual/demo_map \
  result_file:=/artifacts/manual/demo_result.json
```

使用 `Ctrl+C` 结束。节点应全部报告 `process has finished cleanly`。

## 6. 运行时接口检查

在另一个容器终端中执行：

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /scan
ros2 topic info --verbose /map
ros2 topic info --verbose /plan
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

预期核心节点：

```text
/nav3d_simulator
/nav3d_mapper
/nav3d_localizer
/nav3d_planner
/nav3d_controller
/nav3d_demo_supervisor
```

预期核心接口：

```text
/scan                  sensor_msgs/msg/LaserScan
/wheel/odom            nav_msgs/msg/Odometry
/map                   nav_msgs/msg/OccupancyGrid
/localization/pose     geometry_msgs/msg/PoseStamped
/goal_pose             geometry_msgs/msg/PoseStamped
/plan                  nav_msgs/msg/Path
/cmd_vel               geometry_msgs/msg/Twist
```

## 7. 故障检查

先查看验收日志：

```bash
tail -200 /artifacts/source-current/demo.log
colcon test-result --verbose
```

再检查是否有残留进程：

```bash
docker top Nav_3d_ros2 -eo pid,ppid,user,comm,args
```

空闲状态下只应有 `docker-init` 和 `sleep infinity`。不要使用范围不明确的进程清理命令，
也不要停止其他容器。

## 8. 实机边界

当前专用 bridge 网络用于隔离仿真。接入 MID360 或 Go2 前，需要单独设计实机网络、设备映射
和安全门控；不要直接把当前容器切换到 host 网络并启动 `/cmd_vel`。实机阶段至少需要确认：

- ROS 2 MID360 驱动和 LIO 输出；
- `PointCloud2` 到 `LaserScan` 的投影；
- 时间同步和完整 TF 树；
- 地图加载和初始定位；
- Go2 ROS 2 底盘桥、速度限幅、watchdog、enable 和急停。

## 9. 已验证的只读实机入口

实机依赖只存在于项目派生镜像：

```text
nav3d:ros2-hardware-dev-82dfe36
```

不要在主机安装 Livox 或 Unitree SDK。MID360 标准点云及隔离扫描入口为：

```bash
source /workspace/Nav_3d/install/setup.bash
ros2 launch nav3d_hardware_bringup mid360_scan.launch.py
```

该入口发布 `/livox/lidar`、`/livox/imu_raw`、`/livox/imu` 和
`/livox/scan`，不会发布导航使用的 `/scan`。其中 `/livox/imu_raw` 保留 MID360
协议中的 g 单位；`/livox/imu` 经项目适配器转换为 ROS 要求的 m/s²，并使用
`orientation_covariance[0] = -1` 表示雷达没有输出姿态估计。Go2 只读里程计入口为：

```bash
source /workspace/Nav_3d/install/setup.bash
ros2 launch nav3d_hardware_bringup go2_odom_raw.launch.py
```

该入口只订阅 Unitree `rt/lf/sportmodestate` 并发布 `/go2/odom_raw`。它不会创建
`SportClient`，不会发布 TF，也不会发布或订阅 `/cmd_vel`。已有同安装实机录包确认
`velocity[0]` 的前后符号和 `yaw_speed` 的逆/顺时针符号符合 ROS REP-103；重新创建只读
订阅器也不会让位置清零。但横向速度符号、`position` 的外部参考系以及机器人断电/服务
生命周期后的重置语义仍未知，因此不得把 `/go2/odom_raw` 重映射到 `/wheel/odom`，也
不得让它发布导航 TF。

只读检查：

```bash
ros2 topic hz /go2/odom_raw --qos-reliability best_effort
ros2 topic info --verbose /go2/odom_raw
ros2 topic list -t
```

验收记录位于：

```text
/workspace/Nav_3d/artifacts/hardware-mid360/RESULT.md
/workspace/Nav_3d/artifacts/hardware-go2/RESULT.md
/workspace/Nav_3d/artifacts/hardware-go2-direction-reset-audit/RESULT.md
```

## 10. MID360 与 Go2 静态联合建图检查

本检查只验证实机数据链和地图落盘，不启动定位、规划或控制。它使用：

```text
/livox/lidar -> /livox/scan -> nav3d mapper
rt/lf/sportmodestate -> /go2/odom_raw -> nav3d mapper
nav3d mapper -> /hardware/map
```

启动入口：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Nav_3d/install/setup.bash
ros2 launch nav3d_hardware_bringup static_mapping_probe.launch.py \
  map_output:=/artifacts/hardware-static-map/static_map
```

该入口不会创建 `/cmd_vel`，也不会发布导航用的 `/scan`、`/map`、
`/wheel/odom` 或动态里程计 TF。它会发布已验收的静态传感器链
`base_link -> body -> livox_frame -> imu_link`。实机网络容器必须使用项目派生镜像并单独临时创建；不要改变常驻
`Nav_3d_ros2` 的原始镜像和隔离网络。结束时发送 `Ctrl+C`，应看到四个进程均
`finished cleanly`。

应用已验收外参后的结果：`/livox/scan` 位于 `base_link`，30 帧平均每帧
510.37/720 个有效波束；一次静态地图包含 1038 个已观测栅格和 176 个占用栅格。
记录与地图位于：

```text
/workspace/Nav_3d/artifacts/hardware-static-map/RESULT.md
/workspace/Nav_3d/artifacts/hardware-static-map/static_map.pgm
/workspace/Nav_3d/artifacts/hardware-static-map/static_map.yaml
```

此结果只证明外参后的传感器投影与静态数据链正确，不能证明动态建图或导航闭环正确。
测试 mapper 仍直接组合雷达平面距离与 Go2 原始位置/偏航角。尽管 Go2 前后和旋转符号
已由历史实机录包确认，其 `position` 外部参考系与机器人生命周期重置语义仍未知，不得
将 `/go2/odom_raw` 作为导航位姿输入；动态阶段应先接入 ROS 2 LIO。

## 11. 坐标与外参核对现状

静止状态已确认：

- Unitree `[w, x, y, z]` 四元数换算出的 RPY 与设备 RPY 字段一致；
- Go2 原始里程计时间戳单调、静止位置稳定；
- MID360 原始加速度确为 g，标准 `/livox/imu` 已转换为 m/s²；
- 一次候选地面拟合给出的雷达离地距离约 0.363 m、法向偏离雷达 Z 轴约 3.89°。

后续只读测试已确认重启 DDS 订阅器不会让 Go2 `position` 清零，但仍不能确定断电或
运动服务生命周期变化时的重置行为。雷达外参已在用户确认安装一致后，
从 `/home/nuc/Nav_test/data/calibration/quadruped/latest/` 的生产标定复制到当前工程；运行时
不依赖 `Nav_test`。ROS 1 的 `mid360_g1.yaml` 中 `extrinsic_T/R` 仍只表示雷达与雷达 IMU
的内部外参，不能替代机身安装外参。

当前唯一静态链及数值为：

```text
base_link -> body:      xyz=[0, 0, 0], quaternion=[0, 0, 0, 1]
body -> livox_frame:    xyz=[0.2734, 0, 0.13228413]
                        quaternion xyzw=[0.00268398, 0.09500185,
                                         -0.00025614, 0.99547345]
livox_frame -> imu_link: xyz=[0.011, 0.02329, -0.04412]
                         quaternion=[0, 0, 0, 1]
```

`/livox/scan` 会先把点云变换到 `base_link`，再用 `[-0.18, 0.30] m` 高度切片投影；
该下限比原 `-0.25 m` 更能排除站立地面回波。

详细数据：

```text
/workspace/Nav_3d/artifacts/hardware-coordinate-audit/RESULT.md
/workspace/Nav_3d/artifacts/hardware-extrinsics/RESULT.md
/workspace/Nav_3d/artifacts/hardware-go2-direction-reset-audit/RESULT.md
```

## 12. Go2 H4 命令门禁 dry-run

该入口只用于隔离 ROS 域中的控制接口验收，不连接 Go2，不导入 Unitree SDK：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Nav_3d/install/setup.bash
ros2 launch nav3d_hardware_bringup go2_command_gate_dry_run.launch.py
```

输入为 `/cmd_vel`、`/nav/control_enabled` 和 `/nav/emergency_stop`；诊断输出为：

```text
/nav/h4_gate/status
/nav/h4_gate/input_valid
/nav/h4_gate/zero_output
```

运动阶段锁在源码中固定为关闭，不是 ROS 参数。即使三个输入都新鲜、命令合法，状态也
只能到 `MOTION_STAGE_LOCKED`；`/nav/h4_gate/zero_output` 的六个分量永远为零。
命令或安全心跳超过 0.15 秒、急停、未 enable、非有限值、非平面自由度、线速度超过
0.05 m/s 或角速度超过 0.30 rad/s 都会失败关闭。

隔离 ROS 图的 6 阶段验收已通过，共检查 90 条零输出，最大绝对分量为 0；Ctrl+C 干净
退出且无 ROS 节点残留。详细记录：

```text
/workspace/Nav_3d/artifacts/hardware-go2-h4-gate/RESULT.md
```

此入口不是实机底盘桥，不能通过改参数解锁运动。

## 13. Go2 遥控器优先安全监督器（只读）

该入口只订阅 Unitree `rt/lf/lowstate` 并发布 ROS 2 安全状态；源码常量
`MOTION_STAGE_UNLOCKED=False` 不可通过 ROS 参数覆盖：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Nav_3d/install/setup.bash
ros2 launch nav3d_hardware_bringup \
  go2_safety_supervisor_read_only.launch.py
```

它要求新进程先看到遥控器完全松开/摇杆中立，再接受新的 START 上升沿；许可后任何其他
遥控输入都锁存人工接管。LowState 超过 0.25 秒、操作者请求超过 0.30 秒都失败关闭。
发布接口为：

```text
/nav/safety/lowstate_ok
/nav/safety/remote_online
/nav/safety/remote_takeover
/nav/safety/remote_soft_estop_input
/nav/safety/remote_motion_permit
/nav/emergency_stop
/nav/control_enabled
/nav/safety/status
```

节点没有 `SportClient`、Twist 或 `/cmd_vel` 接口。当前静止只读实机验收中，8 秒状态频率
约 20 Hz，唯一状态为 `REMOTE_START_REQUIRED`；`control_enabled` 始终 false，
`emergency_stop` 始终 true，ROS 图没有 `/cmd_vel`，退出干净。记录位于：

```text
/workspace/Nav_3d/artifacts/hardware-go2-safety-supervisor/RESULT.md
```

不要为了看到其他状态而随意按 START 或操作摇杆。实机 START、零命令 SDK 边界、支架测试
和低速运动分别需要现场安全条件与明确授权。

## 14. Go2 零命令单次验收结果

在用户确认机器人已可靠固定后，当前 ROS 2 节点完成了一次授权范围内的实机调用：固定
`0.50 s @ 20 Hz`，共 10 次字面量 `Move(0.0,0.0,0.0)`，随后调用一次
`StopMove`。零 Move 均返回 0，前后状态保持静止；但 `StopMove` 返回厂家状态 `-1`，
因此整体结论是 **FAIL**。

固定版本官方 SDK 没有 `-1` 的含义映射或恢复流程。不得重试、不得改用其他运动/模式 API、
不得发送非零速度，也不得进入低速测试。独立 5 秒只读复查确认机器人仍为
`mode=0/gait=0`，最大线速度仅 `8.47e-8 m/s`。完整记录：

```text
/workspace/Nav_3d/artifacts/hardware-go2-zero-command/RESULT.md
```

注意：该次 `ros2 launch` 正确报告子进程退出码 2，但 launch 外层 shell 返回了 0；验收时
必须以 `GO2 ZERO VELOCITY: PASS/FAIL` 子进程标记和结果文件为准，不能只看外层退出码。
当前入口保持留档，但在 `-1` 原因明确并重新审批前禁止再次运行确认令牌。

后续离线调查确认：本地固定 SDK 与 Unitree 官方当前 master 是同一提交，Sport 客户端和
服务端 API 版本均为 `1.0.0.1`；`-1` 直接来自机器人响应状态，官方公开错误码没有该项。
同机保留证据中，两次 idle/零命令情形均返回 `-1`，而 8 份真实非零运动日志均返回 0；
这只支持“可能没有活动运动可停止”的假设，不能当作官方定义。调查记录与可直接提交给
Unitree 支持的问题模板位于：

```text
/workspace/Nav_3d/artifacts/hardware-go2-zero-command/STOPMOVE_INVESTIGATION.md
```

## 15. ROS 2 MID360 + FAST-LIO 实机建图启动

该链路采用 HKU-MARS 官方 FAST_LIO ROS2 分支的固定提交，只启动 MID360、SI 单位 IMU
适配、LIO、TF、二维扫描投影和 `nav3d_native` 建图器，不启动 Unitree 控制 SDK，也不创建
`/cmd_vel`：

首次或源码变化后，在项目硬件镜像容器中构建：

```bash
cd /workspace/Nav_3d
./scripts/build_hardware_lio_workspace.sh
```

该脚本不安装 apt/pip 包，也不需要 `pcl_ros`。

```bash
source /opt/ros/humble/setup.bash
source /workspace/Nav_3d/install/setup.bash
source /workspace/Nav_3d/install_lio/setup.bash
ros2 launch nav3d_hardware_bringup hardware_mapping.launch.py
```

只需要 LIO 而不需要占用栅格时，改为：

```bash
ros2 launch nav3d_hardware_bringup mid360_lio.launch.py
```

必须在项目专用的 host-network 硬件容器 `Nav_3d_ros2` 中执行。
实机验收通过的接口为 `/livox/lidar`、`/livox/imu`、`/lio/odom`、
`/lio/cloud_registered_body`、`/livox/scan` 和 `/hardware/map`。固定 30 秒 LIO 首末漂移
为 3.231 mm / 0.010799 deg；20 秒建图得到 190 个占用格和 910 个自由格。完整记录：

```text
/workspace/Nav_3d/artifacts/hardware-fast-lio-static/RESULT.md
```

## 16. ROS 1 实机 3D 导航链的 ROS 2 复现

当前实机依赖已固化为镜像：

```text
nav_3d_ros2:v1
image id: ae3c22a320aa
```

当前 host-network 实机常驻容器为 `Nav_3d_ros2`，使用镜像 `nav_3d_ros2:v1`。

本链路严格沿用 ROS 1 的运行模型：

```text
MID360 -> FAST-LIO -> map 到 lio_odom 的已知起点静态 TF
-> 3D PRM -> 3D Bezier -> CasADi NMPC -> /cmd_vel
-> Go2 SportClient bridge
```

没有加入点云地图重定位。机器人必须放回建图轨迹的起始位置，并保持与建图开始时相同的
朝向；静态 TF 不是全局定位。按当前使用要求，不启动动态障碍物处理，规划空间直接采用：

```text
/workspace/Nav_3d/src/data/traversable_cloud/traversable_areas.pcd
```

实机现场首次验证发现原点附近仅形成 3 节点孤立路网，因此实机一键入口改用保留原文件的
补点版本：

```text
/workspace/Nav_3d/src/data/traversable_cloud/traversable_areas_start_patch.pcd
```

该文件在起点附近增加 `x=-1.0..1.4 m`、`y=-0.8..1.4 m`、`z=-0.25 m` 的水平点云，
用于把起点连接到主路网；原始 `traversable_areas.pcd` 未覆盖。

### 16.1 构建

CasADi 3.5.5 已位于镜像 `/opt/casadi`，无需重复安装。构建命令：

```bash
docker exec -it Nav_3d_ros2 bash
cd /workspace/Nav_3d
export ROS_DISTRO=humble
export CASADI_ROOT=/opt/casadi
export CASADI_LIB_PATH=/opt/casadi/lib
export LD_LIBRARY_PATH=/opt/casadi/lib:${LD_LIBRARY_PATH:-}
./scripts/build_hardware_lio_workspace.sh
```

该步骤不需要安装 `pcl_ros`。

### 16.2 当前容器已有 LIO 时启动导航前端

确认机器人已经在旧地图起点和原朝向后执行：

```bash
docker exec -it Nav_3d_ros2 bash
source /opt/ros/humble/setup.bash
source /workspace/Nav_3d/install/setup.bash
source /workspace/Nav_3d/install_lio/setup.bash
export ROS_DOMAIN_ID=142
export LD_LIBRARY_PATH=/opt/casadi/lib:${LD_LIBRARY_PATH:-}

ros2 launch nav3d_hardware_bringup \
  hardware_navigation_known_start.launch.py \
  start_lio:=false \
  start_controller:=false
```

`start_lio:=false` 是因为当前容器已运行 MID360/FAST-LIO，避免重复启动驱动。
`start_controller:=false` 保证没有 `/cmd_vel`。新建的干净实机容器尚未运行 LIO 时，改成
`start_lio:=true`。

目标接口为 ROS 2 标准话题 `/goal_pose`。例如：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: map},
  pose: {
    position: {x: 0.777727608, y: 1.473814282, z: 3.057909784},
    orientation: {
      x: -0.220542395, y: -0.020329797,
      z: 0.971484583, w: 0.084649015
    }
  }
}"
```

上面的目标只是现有建图轨迹终点，用于验证链路。预期接口：

```text
/planned_path       3D PRM 原始路径
/path_smooth        3D Bezier 路径
/local_plan         NMPC 控制序列
/navigation_state   0..5 状态机
```

### 16.3 控制器与底盘边界

ROS 2 NMPC 控制器保留旧版行为：默认手动、零速度，在交互终端按 `i` 才切到自动，按 `q`
返回手动。只有现场运动条件重新批准后才可启动：

```bash
ros2 run nmpc_planner_ros2 nmpc_controller_node --ros-args \
  --params-file \
  /workspace/Nav_3d/install_lio/nmpc_planner_ros2/share/nmpc_planner_ros2/config/nmpc.yaml
```

旧版 Go2 `/cmd_vel` 桥也已原生移植为
`nav3d_hardware_bringup/go2_base_controller_ros2`。通用 launch 默认仍不启动底盘桥；第 18 节的
实机一键脚本在现场确认后才显式启动它。`StopMove=-1` 按当前现场决定不作为启动阻断项，
但桥退出时仍先连续发送 10 次零速度。

### 16.4 已完成的无运动验收

- 3D PCD：71797 点，过滤后 598 个体素，355 个安全体素；
- PRM：355 个顶点、2137 条边；
- 轨迹终点规划：原始路径 21 点，Bezier 路径 105 点；
- NMPC：输出 10 组控制量和 11 点预测轨迹；
- 默认控制器：151 条 `/cmd_vel` 样本最大绝对值为 0；
- 单元测试：28/28 通过；
- 没有启动障碍物处理，没有地图重定位，没有调用 Go2 运动 SDK。

## 17. ROS 1 离线模拟导航的 ROS 2 复现

原 ROS 1 `nav_bringup/offline_navigation_sim.launch` 中的运动学虚拟底盘已移植到 ROS 2。
它发布 `map -> base_link`、`/odom`、`/simulated_path` 和 `/tracking_error`，不会启动
MID360、FAST-LIO、Go2 SDK 或实机底盘桥。

仿真控制器的输出固定重映射为 `/sim/cmd_vel`。实机桥监听的 `/cmd_vel` 在该启动入口中
不存在，即使误用了实机容器也不会把模拟速度送到底盘。

### 17.1 启动

在主机工程根目录只执行一条命令：

```bash
cd /home/nuc/Nav_3d-ros2-native-rebuild-20260817
./scripts/run_offline_navigation_rviz.sh
```

脚本自动完成现有 `Nav_3d_ros2` 容器检查、X11 授权复制、DDS 域 179、ROS 2
overlay 加载，以及算法和 RViz2 的统一启动。不要再逐行执行 `docker exec/export/source`。

默认 `motion_mode:=path`，虚拟底盘沿 `/path_smooth` 的 XYZ 轨迹运动，适合当前包含高度
变化的三维地图。它是轨迹回放/运动学仿真，不模拟足端接触、步态和动力学。

### 17.2 在 RViz2 中操作

RViz2 会自动显示：

```text
Traversable PCD  可通行点云
Planned Path     橙色 3D PRM 原始路径
Smooth Path      绿色 Bezier 平滑路径
Local Path       蓝色局部参考轨迹
Simulated Path   紫色虚拟底盘实际轨迹
Simulated Robot  黄色虚拟机器人箭头
```

工具栏中的 `3D Nav Goal` 直接发布 ROS 2 `/goal_pose`：按住左键拖动设置 XY 和航向；左键
保持按下时再按住右键并上下移动设置 Z；释放左键后发布目标。规划路线和虚拟机器人运动会
直接显示在同一个 RViz2 窗口中，不需要第二个终端发布目标。

停止时在启动终端按 `Ctrl+C`。若终端异常退出但窗口仍在，执行：

```bash
./scripts/stop_offline_navigation_rviz.sh
```

`/navigation_state` 最终应为 `4`（`COMPLETED`）。规划和仿真接口为：

```text
/planned_path       3D PRM 原始路径
/path_smooth        3D Bezier 平滑路径
/local_path         局部参考轨迹
/sim/cmd_vel        仅供虚拟底盘使用的控制量
/simulated_path     虚拟底盘实际轨迹
/tracking_error     [最近点误差, 最近点索引, 路径完成比例]
```

原 ROS 1 的平面控制模式也保留：

```bash
NAV3D_MOTION_MODE=cmd_vel ./scripts/run_offline_navigation_rviz.sh
```

该模式按 `/sim/cmd_vel` 积分 XY 和航向，并从三维参考轨迹更新 Z。复杂曲线下原 NMPC 的
QRSQP 可能因数值收敛失败转入原有 fallback tracker；默认三维 `path` 模式不依赖该求解
结果完成运动。

## 18. 实机导航一键启动

本入口只使用现有常驻容器 `Nav_3d_ros2`，不会创建新容器，不会修改主机 ROS
环境，也不会启动障碍物处理或地图重定位。容器中已有 MID360/FAST-LIO，因此入口固定
使用 `start_lio:=false`，避免重复启动硬件驱动。

### 18.1 现场前提

- 机器狗放在旧地图的建图起点，并保持建图开始时的朝向；
- 测试区域无障碍物，人员与线缆离开运动范围；
- 遥控器在线且随时可以接管或急停；
- `enp86s0` 已连接 Go2，地址 `192.168.123.161` 可达；
- 当前系统已有持续更新的 `/lio/odom`。

本工程没有全局地图定位。若起点或朝向不一致，即使 RViz 能规划，实际路线也会整体错位。

### 18.2 启动和操作

在主机任意终端只复制下面这一条命令：

```bash
/home/nuc/Nav_3d-ros2-native-rebuild-20260817/scripts/run_hardware_navigation_rviz.sh
```

脚本会自动检查现有容器、Go2 网口、Go2 地址、实时 LIO 里程计和 RViz 显示权限。检查通过
后，按提示输入：

```text
START_REAL_NAVIGATION
```

随后 RViz2 打开。工具栏选择 `3D Nav Goal`，在可通行点云上设置目标；目标一经发布，控制器
已处于自动模式，机器狗可能立即开始运动。

本实机入口按当前要求对底盘有效指令进行固定速度整形：

```text
非零前进/后退：±0.40 m/s
非零转向：      ±0.40 rad/s
侧向速度：       0.00 m/s
停止指令：       0
```

这不是只设上限：低于机器狗有效动作阈值的非零控制量也会提升到 `0.4`。极小数值噪声
（绝对值不超过 `1e-4`）仍按零处理。

### 18.3 停止

首选用机器狗遥控器急停。启动终端按 `Ctrl+C` 时，Go2 桥会先发送连续零速度再退出。
若启动终端已丢失，执行：

```bash
/home/nuc/Nav_3d-ros2-native-rebuild-20260817/scripts/stop_hardware_navigation.sh
```

`StopMove` 返回 `-1` 会被记录但不阻断退出，连续零速度仍会执行。
