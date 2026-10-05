# M20Pro 前雷达全流程精简指令（无避障）

本流程固定使用前雷达 `/m20/lidar/front`，适用于从建图、提取可通行区域到实机导航控制的完整流程。

需要全程使用 M20 标准楼梯步态 `0x1003`（十进制 `4099`）时，使用 [M20_STAIR.md](./M20_STAIR.md)，不要启动本文档的平地 Motion Adapter。

导航使用 `bringup_m20_nav.launch` 一键启动定位、全局规划、Bezier 路径平滑、NMPC 和 RViz。该 launch 不启动 `obstacle_processor`，因此测试区域必须保持清空，并确保遥控器可随时接管或急停。

如果当前地图已经生成且配套正确，可跳过第 2、3 节，直接从第 4 节开始导航。

## 1. 启动传感器通信

终端 1，全程保持运行：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh start  # reads exported M20_SSH_PASSWORD when required
./scripts/m20_sensor_transport.sh status
```

## 2. 前雷达建图

终端 2：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch fast_lio mapping_m20.launch \
  start_lidar_fusion:=false \
  lidar_topic:=/m20/lidar/front
```

建图完成后先停稳 M20，再在终端 2 按 `Ctrl+C`，等待 PCD 保存完成。确认同一轮建图的两个文件都已生成：

```bash
ls -lh \
  /home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  /home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

## 3. 提取可通行区域

终端 3：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

mkdir -p /home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front

roslaunch map_process_test map_process_custom.launch \
  point_cloud_file:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  trajectory_file:=/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt \
  output_file:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd \
  config_file:=/home/arts-inpector/ART_3dnav/src/3D_NAV/map_process_test/config/default_params.yaml \
  trajectory_format:=1
```

点云和轨迹必须来自同一轮建图。等待日志出现 `Processing Complete`，然后检查输出：

```bash
ls -lh /home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd
```

## 4. 一键启动导航

终端 2 可重新使用：

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

该终端保持运行。不要再单独启动定位、全局规划、Bezier、NMPC 或避障节点。

## 5. 切换并检查底盘状态

使用遥控器将 M20Pro 切换到导航模式，然后执行：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
docker exec m20_sensor_pipeline_safe bash -lc \
  'source /opt/ros/foxy/setup.bash; \
   source /opt/m20_real_ws/install/setup.bash; \
   timeout 3 ros2 topic echo --qos-reliability reliable /MOTION_INFO'
```

必须确认：

```text
state=17
gait=4097
```

状态不正确时，不要启动运动 adapter，也不要按 `i` 进入自动模式。

## 6. 启动 `/cmd_vel` Bridge

终端 4，保持运行：

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

## 7. 启动 Motion Adapter

终端 5，保持运行：

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

确认日志包含：

```text
M20 real motion adapter started in enabled mode
M20 feedback connected: state=17, gait=4097
```

## 8. 下发目标并进入自动模式

1. 在 RViz 中使用 `3D Nav Goal` 下发目标。
2. 新开一个已 source ROS1 的终端，确认规划链路和 bridge：

```bash
source /opt/ros/noetic/setup.bash

rostopic echo -n 1 /planning_3d_PRM_node/planned_path
rostopic echo -n 1 /path_smooth
rostopic echo -n 1 /local_plan
rostopic info /cmd_vel
```

3. 确认 `/local_plan` 有效，且 `/cmd_vel` 的订阅者包含 `/m20_cmd/ros_bridge`。
4. 回到第 4 节的一键启动终端，按 `i` 进入自动模式。
5. 需要退出自动模式时，先按 `q` 回到手动模式，再按 `s` 清零速度。

注意：手动模式下 `q` 是左前移动，不是停止；手动模式必须按 `s` 停止。

## 9. 停止顺序

1. 一键启动终端：自动模式下先按 `q`，再按 `s`。
2. 终端 5：按 `Ctrl+C` 停止 motion adapter。
3. 终端 4：按 `Ctrl+C` 停止 `/cmd_vel` bridge。
4. 一键启动终端：按 `Ctrl+C` 停止导航节点。
5. 使用遥控器退出导航模式并恢复遥控接管。
6. 最后停止传感器通信：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh stop
```
