# M20Pro 前雷达楼梯导航指令（标准楼梯步态 4099）

本流程沿用 `M20_FRONT_LIDAR_QUICKSTART.md` 的建图、规划、Bezier 平滑和 NMPC，只替换实机运动前的步态检查与 Motion Adapter 启动方式。

底层控制链路固定为：

```text
NUC /cmd_vel
-> ROS1/ROS2 parameter_bridge
-> M20 stair motion adapter
-> /NAV_CMD
-> M20Pro（state=17, gait=4099）
```

楼梯版本使用云深处 M20 标准楼梯步态 `0x1003`（十进制 `4099`）。Motion Adapter 只有在 `/MOTION_INFO` 同时满足 `state=17` 和 `gait=4099` 时才会转发速度；步态被切走后会停止发送 `/NAV_CMD`，不影响现有平地版本。

## 1. 启动传感器通信

终端 1，全程保持运行：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh start  # reads exported M20_SSH_PASSWORD when required
./scripts/m20_sensor_transport.sh status
```

## 2. 建图与提取可通行区域（已有地图可跳过）

终端 2，前雷达建图：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch fast_lio mapping_m20.launch \
  start_lidar_fusion:=false \
  lidar_topic:=/m20/lidar/front
```

建图结束后停稳 M20，按 `Ctrl+C` 等待 PCD 保存，再确认同一轮点云和轨迹均已生成：

```bash
ls -lh \
  /home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  /home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

终端 3，提取可通行区域：

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

等待日志出现 `Processing Complete`，然后检查输出：

```bash
ls -lh /home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd
```

## 3. 启动导航

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

## 4. 切换并检查楼梯步态

先用遥控器切到楼梯步态，再切换到导航模式。开始底层通信前必须确认 M20 反馈为 `state=17, gait=4099`：

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
gait=4099
```

任一值不正确时，不要启动 Motion Adapter，也不要按 `i` 进入自动模式。

## 5. 启动 `/cmd_vel` Bridge

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

## 6. 启动楼梯 Motion Adapter

终端 5，保持运行：

```bash
cd /home/arts-inpector/m20pro_ros2_mock

ROS_IP=10.21.31.50 \
IMAGE_NAME=m20pro-sensor-pipeline:foxy-noetic \
./scripts/start_m20_stair_motion_adapter.sh
```

该入口固定使用以下底层参数，并将规划器发布的 `/cmd_vel` 三个速度分量原样转发，
通信层不再进行速度限幅：

```text
default_gait=4099
cmd_timeout_sec=0.3
publish_rate_hz=20.0
```

`cmd_timeout_sec` 只负责在 `/cmd_vel` 超过 0.3 秒未更新时发送零速度，防止通信中断后
继续执行旧指令，不会改变正常收到的速度值。

确认日志包含：

```text
M20 real motion adapter started in enabled mode
M20 feedback connected: state=17, gait=4099
```

## 7. 下发目标并进入自动模式

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
4. 再次确认 `/MOTION_INFO` 为 `state=17, gait=4099`。
5. 回到第 3 节的一键启动终端，按 `i` 进入自动模式。
6. 需要退出自动模式时，先按 `q` 回到手动模式，再按 `s` 清零速度。

注意：手动模式下 `q` 是左前移动，不是停止；手动模式必须按 `s` 停止。

## 8. 无动作时的检查顺序

依次检查，前一项不正确时不要继续：

```bash
rostopic echo /cmd_vel
rostopic info /cmd_vel
```

```bash
docker exec m20_sensor_pipeline_safe bash -lc \
  'source /opt/ros/foxy/setup.bash; \
   source /opt/m20_real_ws/install/setup.bash; \
   timeout 3 ros2 topic echo --qos-reliability reliable /MOTION_INFO'
```

判断标准：

- `/cmd_vel` 必须持续出现非零速度。
- `/cmd_vel` 必须存在 `/m20_cmd/ros_bridge` 订阅者。
- `/MOTION_INFO` 必须保持 `state=17, gait=4099`；否则楼梯 adapter 会停止转发。
- 云深处文档给出的标准楼梯步态 X 方向有效非零速度下限为 `0.15 m/s`。若 `/cmd_vel.linear.x` 长时间低于该值，底层可能不执行。

## 9. 停止顺序

1. 一键启动终端：自动模式下先按 `q`，再按 `s`。
2. 终端 5：按 `Ctrl+C` 停止楼梯 Motion Adapter；退出时会发送零速度。
3. 终端 4：按 `Ctrl+C` 停止 `/cmd_vel` bridge。
4. 一键启动终端：按 `Ctrl+C` 停止导航节点。
5. 使用遥控器退出导航模式并恢复遥控接管。
6. 最后停止传感器通信：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh stop
```

楼梯版本不要与平地 Motion Adapter 同时运行；二者默认使用同一个 `m20_motion_adapter` 容器名，Docker 会阻止重复启动。
