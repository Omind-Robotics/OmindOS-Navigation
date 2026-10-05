# 3D_NAV 简洁命令文档

适用工作空间：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 1. 编译

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 2. 建图（MID360 + FAST-LIO）

```bash
roslaunch fast_lio mapping_mid360_g1.launch
```

输出：

- 地图：`src/data/point_cloud/scans.pcd`
- 轨迹：`src/data/trace_data/mapping_trajectory.txt`

## 3. 提取可通行区域

```bash
roslaunch map_process map_process_custom.launch \
  point_cloud_file:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  trajectory_file:=/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt \
  output_file:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd \
  config_file:=/home/arts-inpector/ART_3dnav/src/3D_NAV/map_process/config/mid360_traversable_params.yaml \
  trajectory_format:=1
```

## 4. 启动导航链路

### 4.1 仿真 / 只发布静态地图

```bash
roslaunch map_publisher map_publisher.launch \
  map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd
```

### 4.2 实机定位（MID360）

```bash
roslaunch fast_lio localization_mid360_nav.launch
```

### 4.3 全局规划

```bash
roslaunch planning_3d 3d_planning.launch \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd
```

默认读取新目录里的可通行区域：

```bash
roslaunch planning_3d 3d_planning.launch \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd
```

### 4.4 路径平滑

```bash
roslaunch bezier_path_optimizer bezier_path_optimizer.launch
```

### 4.5 局部规划 / 控制

```bash
roslaunch nmpc_planner nmpc_controller.launch
```

说明：启动后默认手动模式，在该终端按 `i` 切自动，按 `q` 回手动。

### 4.6 障碍物处理

```bash
roslaunch obstacle_processor obstacle_processor.launch
```

## 5. Go2 底盘接入

连通性检查：

```bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
python3 loco/sdk_probe.py --iface enp86s0 --peer 192.168.123.161
```

启动控制桥：

```bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
GO2_IFACE=enp86s0 GO2_PEER=192.168.123.161 ./loco/go2_base_control.sh start
```

停止：

```bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
./loco/go2_base_control.sh stop
```

## 6. 常用排查

```bash
rostopic echo /planning_3d_PRM_node/planned_path
rostopic echo /path_smooth
rostopic echo /local_plan
rostopic echo /cmd_vel
rostopic echo /navigation_state
rostopic echo /obs_raw
```

## 7. 最小顺序

```text
编译
-> 建图
-> map_process 提取 traversable_areas
-> localization / map_publisher
-> planning_3d
-> bezier_path_optimizer
-> nmpc_planner
-> obstacle_processor
-> go2_base_controller（实机时）
```
