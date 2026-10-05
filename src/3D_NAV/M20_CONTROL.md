# M20Pro 导航与控制

本流程按独立终端启动以下链路：

```text
传感器通信
-> FAST-LIO + Open3D 定位
-> planning_3d 全局规划
-> Bezier 路径平滑
-> NMPC 局部规划与控制
-> obstacle_processor 动态避障
-> ROS1 /cmd_vel 到 M20Pro /NAV_CMD
```

前雷达和融合点云方案二选一，不要同时启动两套定位或规划节点。

## 0. 地图选择

| 方案 | 定位原始地图 | 规划可通行地图 | 在线点云 |
| --- | --- | --- | --- |
| 前雷达（当前新地图，推荐） | `scans.pcd` | `map_process_test_front/traversable_areas.pcd` | `/m20/lidar/front` |
| 融合点云 | `replay_fused/scans.pcd` | `map_process_test_fused/traversable_areas.pcd` | `/m20/lidar/fused` |

### 0.1 处理重新建图的前雷达数据

当前这轮前雷达建图使用以下两个配套文件：

```text
/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd
/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

两个文件必须来自同一轮建图。不要把新的 `scans.pcd` 与旧的
`mapping_trajectory_replay_front.txt` 混用。

建图结束且 FAST-LIO 已完成 PCD 保存后，执行：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch map_process_test map_process_custom.launch \
  point_cloud_file:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  trajectory_file:=/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt \
  output_file:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd \
  config_file:=/home/arts-inpector/ART_3dnav/src/3D_NAV/map_process_test/config/default_params.yaml \
  trajectory_format:=1
```

`map_process_test` 处理完成后会自动退出。确认日志出现
`Processing Complete`，然后检查新可通行地图：

```bash
ls -lh /home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd
```

后续前雷达定位和全局规划使用下文的当前新地图路径。

## 离线规控仿真（不接入 M20Pro）

已有可通行区域时，可直接使用离线运动学底盘验证完整的
`全局规划 -> 平滑 -> NMPC 跟踪` 链路。这个启动文件只订阅 ROS1
默认以 `/path_smooth` 作为回放轨迹，也保留 `/cmd_vel` 平面模拟模式，并在本机发布
`map -> base_link` 和 `/odom`；
不会启动传感器通信、ROS1/ROS2 bridge 或 M20Pro motion adapter。

当前离线地图的默认路径已经配置为：

```text
/home/arts-inpector/ART_3dnav/src/data/offline_mapping/m20_front_clean/traversable/traversable_areas.pcd
```

启动时必须将 `initial_x`、`initial_y`、`initial_z` 放在可通行区域上：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch nav_bringup offline_navigation_sim.launch \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

RViz 打开后，先用 `2D Pose Estimate` 发布 `/initialpose` 微调 XY 和航向，
再使用工具栏中的 `3D Nav Goal` 在可通行点云上给出目标点。随后将看到：

```text
/planning_3d_PRM_node/planned_path  全局轨迹
/path_smooth                        平滑轨迹
/local_path                         NMPC 局部轨迹
map -> base_link                    动态跟踪的仿真底盘
```

默认的 `motion_mode:=path` 会沿 `/path_smooth` 的 XYZ 路径回放 `base_link`，
因此在楼梯和坡道处也会更新 Z，适合纯 RViz 的轨迹演示。它不模拟接触、步态或动力学。
若需要恢复仅通过 NMPC `/cmd_vel` 积分的平地模式，启动时指定：

```bash
roslaunch nav_bringup offline_navigation_sim.launch motion_mode:=cmd_vel
```

初始 Z 由启动参数保留，RViz 的 `2D Pose Estimate` 默认不会把它重置为 0。

## 1. 传感器通信

终端 1：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh start  # reads exported M20_SSH_PASSWORD when required
./scripts/m20_sensor_transport.sh status
```

确认 `/IMU`、`/m20/lidar/front` 和 `/m20/lidar/rear` 都有发布者。

## 2. 定位

以下两条命令只执行一条。

### 2.1 前雷达定位（推荐）

终端 2：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch fast_lio localization_m20_nav.launch \
  start_lidar_fusion:=false \
  lidar_topic:=/m20/lidar/front \
  map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  use_map_registration:=true \
  rviz:=false
```

### 2.2 融合点云定位

终端 2：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch fast_lio localization_m20_nav.launch \
  start_lidar_fusion:=true \
  lidar_topic:=/m20/lidar/fused \
  map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/replay_fused/scans.pcd \
  use_map_registration:=true \
  rviz:=false
```

融合定位会同时启动前后雷达融合节点，不需要另外发布 `/m20/lidar/fused`。

## 3. 全局规划

以下两条命令只执行与定位方案匹配的一条。该 launch 同时启动 RViz。

### 3.1 前雷达可通行地图

终端 3：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch planning_3d 3d_planning.launch \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd
```

### 3.2 融合点云可通行地图

终端 3：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch planning_3d 3d_planning.launch \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_fused/traversable_areas.pcd
```

## 4. 路径平滑

终端 4：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bezier_path_optimizer bezier_path_optimizer.launch
```

## 5. 局部规划与控制

终端 5：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch nmpc_planner nmpc_controller.launch \
  start_in_auto:=false \
  start_obstacle_processor:=false
```

NMPC 默认保持手动模式。不要按 `i`，直到定位、规划、避障和 M20Pro 底层通信
全部检查完成。

局部规划器发布 `/local_plan`，`nmpc_controller_node` 在自动模式下读取该规划结果，
并发布 ROS1 `/cmd_vel`。底盘链路不会直接订阅 `/local_plan`，完整控制链为：

```text
/local_plan
-> nmpc_controller_node
-> ROS1 /cmd_vel
-> /m20_cmd/ros_bridge
-> ROS2 /cmd_vel
-> m20_real_motion_adapter
-> M20Pro /NAV_CMD
```

## 6. 动态避障

以下两条命令只执行与定位方案匹配的一条。

### 6.1 前雷达避障

终端 6：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch obstacle_processor obstacle_processor.launch \
  lidar_topic:=/m20/lidar/front \
  lidar_frame_id:=base_link \
  world_map_topic:=/planning_3d_PRM_node/pcd_map
```

### 6.2 融合点云避障

终端 6：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch obstacle_processor obstacle_processor.launch \
  lidar_topic:=/m20/lidar/fused \
  lidar_frame_id:=base_link \
  world_map_topic:=/planning_3d_PRM_node/pcd_map
```

### 6.3 组合启动选项：定位 + 全局/局部规划（不含避障）

如果当前不启用动态避障，可以用 `bringup_m20_nav.launch` 一次启动定位、
`planning_3d`、Bezier 路径平滑、NMPC 局部规划与控制，以及 RViz。

这个选项替代第 2、3、4、5 节的独立终端，不要再重复启动这些节点，并跳过第 6.1、
6.2 节的 `obstacle_processor`。传感器通信仍需先按第 1 节启动；ROS1/ROS2 控制 bridge 和
M20Pro motion adapter 仍按第 7、8 节单独启动。

前雷达方案（当前新地图，推荐）：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch nav_bringup bringup_m20_nav.launch \
  m20_start_lidar_fusion:=false \
  m20_lidar_topic:=/m20/lidar/front \
  map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd \
  use_map_registration:=true \
  start_in_auto:=false \
  start_rviz:=true
```

融合点云方案：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch nav_bringup bringup_m20_nav.launch \
  m20_start_lidar_fusion:=true \
  m20_lidar_topic:=/m20/lidar/fused \
  map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/replay_fused/scans.pcd \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_fused/traversable_areas.pcd \
  use_map_registration:=true \
  start_in_auto:=false \
  start_rviz:=true
```

该 launch 固定不启动 `obstacle_processor`。NMPC 默认保持手动模式；第 7、8 节底层
通信检查完成并下发目标后，在这个 launch 所在终端按 `i` 进入自动模式。

## 7. ROS1 `/cmd_vel` 桥接到 ROS2

传感器容器只桥接传感器话题。这里单独启动一个只包含 `/cmd_vel` 的 bridge，
避免重复桥接 `/IMU`。

### 7.1 通过遥控器切换底盘模式

确认机器周围安全，通过遥控器将 M20Pro 切换到导航模式。实机验证表明，狗端会随
遥控器模式自动切换相关服务，不需要通过 SSH 手动停止 `planner.service` 或
`global_planner.service`。

切换后必须确认底盘进入已验证的 RL 导航状态：

```text
state=17
gait=4097
```

可以通过正在运行的传感器容器读取底盘反馈：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
docker exec m20_sensor_pipeline_safe bash -lc \
  'source /opt/ros/foxy/setup.bash; \
   source /opt/m20_real_ws/install/setup.bash; \
   timeout 3 ros2 topic echo --qos-reliability reliable /MOTION_INFO'
```

如果不是 `state=17, gait=4097`，不要启动运动 adapter，也不要让 NMPC 进入自动模式。

### 7.2 启动独立 `/cmd_vel` bridge

终端 7：

```bash
source /opt/ros/noetic/setup.bash

rosparam set /m20_cmd_vel_bridge_topics \
  '[{topic: /cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10}]'
rosparam set /m20_cmd_vel_bridge_services_1_to_2 '[]'
rosparam set /m20_cmd_vel_bridge_services_2_to_1 '[]'

cd /home/arts-inpector/m20pro_ros2_mock
ROS_IP=10.21.31.50 \
IMAGE_NAME=m20pro-sensor-pipeline:foxy-noetic \
CONTAINER_NAME=m20_cmd_vel_bridge \
./scripts/run_bridge_container.sh bash -lc \
  'source /opt/ros1_bridge_ws/install/setup.bash && exec ros2 run ros1_bridge parameter_bridge /m20_cmd_vel_bridge_topics /m20_cmd_vel_bridge_services_1_to_2 /m20_cmd_vel_bridge_services_2_to_1 __ns:=/m20_cmd'
```

该终端保持运行。`__ns:=/m20_cmd` 是必须项：传感器 bridge 已使用节点名
`/ros_bridge`；如果省略命名空间，新 bridge 会因同名节点注册而关闭传感器 bridge。

启动后检查：

```bash
rosnode list | grep -E '^/m20_cmd/ros_bridge$|^/ros_bridge$'
rostopic info /cmd_vel
```

应同时看到传感器 `/ros_bridge` 和控制 `/m20_cmd/ros_bridge`，且 `/cmd_vel` 的
Subscribers 中包含 `/m20_cmd/ros_bridge`。

## 8. M20Pro 底层运动通信

确认机器周围安全、遥控器可随时急停后再执行。本命令使用已经通过实机
`/cmd_vel` 前进测试的 `m20_real_motion_adapter`，把 ROS2 `/cmd_vel` 转成
`/NAV_CMD`。实测可用步态是 `4097`，不要使用旧值 `12290`。这里关闭
`auto_start`，只允许 adapter 在已经确认的 `state=17, gait=4097` 下转发速度。

终端 8：

```bash
cd /home/arts-inpector/m20pro_ros2_mock

ROS_IP=10.21.31.50 \
IMAGE_NAME=m20pro-sensor-pipeline:foxy-noetic \
CONTAINER_NAME=m20_motion_adapter \
./scripts/run_bridge_container.sh \
  start_m20_motion_adapter --ros-args \
  -p enable_motion:=true \
  -p auto_start:=false \
  -p default_gait:=4097 \
  -p cmd_timeout_sec:=0.3 \
  -p publish_rate_hz:=20.0 \
  -p max_x_vel:=0.35 \
  -p max_y_vel:=0.0 \
  -p max_yaw_vel:=0.8
```

`max_x_vel` 和 `max_yaw_vel` 是首次闭环导航的底盘侧限速；NMPC 即使输出更大速度，
adapter 也会限幅。横移暂时禁用，因为当前 NMPC controller 只使用前向速度和偏航角速度。

看到日志包含以下内容后，才能继续：

```text
M20 real motion adapter started in enabled mode
M20 feedback connected: state=17, gait=4097
```

在另一个已 source ROS1 的终端确认 bridge 已订阅：

```bash
rostopic info /cmd_vel
```

再确认 ROS2 侧 `/NAV_CMD` 同时存在一个 adapter 发布者和一个狗端订阅者：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
docker exec m20_sensor_pipeline_safe bash -lc \
  'source /opt/ros/foxy/setup.bash; \
   source /opt/m20_real_ws/install/setup.bash; \
   ros2 topic info /NAV_CMD'
```

在终端 5 仍处于手动模式时，`/NAV_CMD` 应持续为零。若发布者/订阅者数量不对，
或零速度检查不通过，不要按 `i`。

## 9. 下发目标并进入自动模式

1. 在终端 3 的 RViz 中使用 `3D Nav Goal` 设置目标。
2. 终端 5 保持手动模式，依次确认规划链路有输出：

```bash
rostopic echo -n 1 /planning_3d_PRM_node/planned_path
rostopic echo -n 1 /path_smooth
rostopic echo -n 1 /local_plan
rostopic echo -n 1 /obs_raw
```

3. 确认 `/local_plan` 有效、底盘反馈为 `state=17, gait=4097`、控制 bridge 正常，
   并且手动模式下 `/cmd_vel` 与 `/NAV_CMD` 都是零。
4. 回到终端 5，按 `i` 进入自动模式。此时 `nmpc_controller_node` 开始把
   `/local_plan` 转成 `/cmd_vel`，M20Pro 开始跟踪路径。
5. 在另一个 ROS1 终端检查 NMPC 实际输出：

```bash
rostopic echo /cmd_vel
```

6. 必要时在传感器容器内检查底盘收到的 `/NAV_CMD` 和实际运动反馈：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
docker exec m20_sensor_pipeline_safe bash -lc \
  'source /opt/ros/foxy/setup.bash; \
   source /opt/m20_real_ws/install/setup.bash; \
   ros2 topic echo --qos-reliability reliable /NAV_CMD'
```

7. 自动模式下按 `q` 回到手动模式，再按 `s` 将速度清零。

注意：手动模式下 `q` 是左前移动，不是停止；手动模式必须按 `s` 停止。

## 10. 停止顺序

1. 终端 5：自动模式按 `q` 回手动，再按 `s` 停止。
2. 终端 8：按 `Ctrl+C` 停止 motion adapter，它会发送一次零速度。
3. 终端 7：按 `Ctrl+C` 停止 `/cmd_vel` bridge。
4. 如果按独立终端启动，依次在终端 6、5、4、3、2 按 `Ctrl+C`；如果使用第 6.3 节
   组合启动选项，只需在组合 launch 终端按 `Ctrl+C`。
5. 确认 NUC 不再发布控制命令：

```bash
source /opt/ros/noetic/setup.bash
rostopic info /cmd_vel
docker ps --format '{{.Names}}' | grep -E 'm20_motion_adapter|m20_cmd_vel_bridge' || true
```

6. 使用遥控器退出导航模式并恢复遥控接管。狗端相关服务会随遥控器模式自动切换，
   不要手动执行 `systemctl start/stop planner.service global_planner.service`。
7. 最后停止传感器通信：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh stop
```
