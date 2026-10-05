# 3D_NAV 运行指令手册

这份文档按当前仓库里的实际 `launch` 和 `README` 整理，目标是给出一条从“建图”开始，到“可通行区域提取”，再到“规划/控制运行”的完整命令链路。

适用工作空间：

```bash
/home/arts-inpector/ART_3dnav
```

## 0. 先说结论

这个项目不是“在线边建图边直接导航”的一体化流程，而是分成两段：

1. 先建图，拿到原始场景点云地图。
2. 建图时同步记录机器人实际通行轨迹。
3. 再用原始地图 + 轨迹文本，离线提取可通行区域。
4. 在线导航时，全局规划器读取的是“可通行区域点云”，不是原始障碍物地图。

所以真正的主流程是：

```text
FAST-LIO / 其他SLAM建图
-> 原始地图 .pcd
-> 同步记录轨迹 .txt
-> map_process 提取 traversable_areas.pcd
-> planning_3d
-> bezier_path_optimizer
-> nmpc_planner
-> obstacle_processor
-> /cmd_vel
```

## 1. 编译工作空间

每次新开终端，先执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
```

首次编译或代码更新后执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
catkin_make
source devel/setup.bash
```

如果你还没有装好定位相关依赖，至少要确认这些依赖可用：

```bash
sudo apt update
sudo apt install -y libeigen3-dev libc++-dev libc++abi-dev
```

## 2. 第一步：建图

### 2.1 MID360 + FAST-LIO 建图

终端 1：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch fast_lio mapping_mid360_g1.launch
```

说明：

- 这个 launch 会启动 `livox_ros_driver2` 和 `fast_lio`。
- 当前 `mapping_mid360_g1.launch` 里 `pcd_save_en` 默认就是 `true`。
- 当前 `mapping_mid360_g1.launch` 里 `record_trajectory` 默认也是 `true`，会同步订阅 `/Odometry_loc` 记录轨迹。
- 建图结束后按 `Ctrl+C` 退出，FAST-LIO 会把累计点云保存到：

```bash
/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd
```

同步记录的轨迹会保存到：

```bash
/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

轨迹格式是：

```text
timestamp x y z qx qy qz qw
```

后面运行 `map_process` 时使用 `trajectory_format:=1`。

如果你用 rosbag 而不是直连雷达：

终端 1：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch fast_lio mapping_mid360_g1.launch start_driver:=false
```

终端 2：

```bash
rosbag play your_mapping.bag --clock
```

如果想自定义轨迹输出位置：

```bash
roslaunch fast_lio mapping_mid360_g1.launch \
  trajectory_output_file:=/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

如果临时不想记录轨迹：

```bash
roslaunch fast_lio mapping_mid360_g1.launch record_trajectory:=false
```

### 2.2 地图后处理

建图出来的原始地图建议至少做两件事：

- 裁掉明显无关区域。
- 用 CloudCompare 做地面对齐、下采样、去掉人影或漂浮噪声。

建议把处理后的地图放到一个固定位置，例如：

```bash
/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd
```

## 3. 第二步：确认轨迹文本

`map_process` 不能只靠地图运行，它还需要一份轨迹文本文件。现在建图 launch 已经会自动记录这份文件。

当前仓库支持 3 种轨迹格式：

1. `timestamp x y z qx qy qz qw`
2. `x y z param1 param2 param3`
3. 多列文本，但位置必须在第 5 到第 7 列

注意：

- `mapping_mid360_g1.launch` 默认输出的是格式 1。
- 如果你使用自己准备的轨迹文件，只需要确认它的路径和格式编号。

默认轨迹文件是：

```bash
/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

## 4. 第三步：提取可通行区域

这是把“原始场景地图”转成“规划器可用地图”的关键步骤。

### 4.1 推荐命令

终端执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash

roslaunch map_process map_process_custom.launch \
  point_cloud_file:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  trajectory_file:=/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt \
  output_file:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd \
  config_file:=/home/arts-inpector/ART_3dnav/src/3D_NAV/map_process/config/mid360_traversable_params.yaml \
  trajectory_format:=1
```

如果你的轨迹是其他来源，再按实际格式把 `trajectory_format` 改成 `2` 或 `3`。

### 4.2 结果文件

提取完成后，重点看这几个文件：

- 主结果：

```bash
/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd
```

- 调试种子点：

```bash
/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/seed_points.pcd
```

- 如果 `save_individual_regions: true`，还会额外输出多个 `region_*.pcd`。

### 4.3 如果你已经生成过可通行区域

默认可通行区域文件是：

```bash
/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd
```

旧点云文件已清理；如果该文件不存在，需要先重新运行 `map_process`。

## 5. 第四步：启动导航链路

在线导航至少需要 5 个部分：

1. 定位或地图发布
2. 全局规划
3. 路径平滑
4. 局部规划/控制
5. 障碍物处理

### 5.1 定位或地图发布

#### 方案 A：仿真或只需要给障碍物模块发布静态地图

终端 1：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch map_publisher map_publisher.launch \
  map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd
```

#### 方案 B：实机 MID360，使用当前仓库已有的 FAST-LIO 导航定位桥接

终端 1：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch fast_lio localization_mid360_nav.launch
```

说明：

- 这个 launch 会启动 `mapping_mid360_g1.launch`，但关闭 `pcd_save`。
- 它也会关闭 `record_trajectory`，避免在线导航定位时覆盖建图轨迹。
- 它会补两条静态 TF：
  - `map -> camera_init`
  - `body -> base_link`

### 5.2 启动全局规划器

终端 2：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch planning_3d 3d_planning.launch \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd
```

如果你想用自己新生成的可通行区域文件，把 `pcd_path` 换成你在第 4 步输出的路径。

### 5.3 启动贝塞尔平滑

终端 3：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch bezier_path_optimizer bezier_path_optimizer.launch
```

### 5.4 启动局部规划与控制

终端 4：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch nmpc_planner nmpc_controller.launch
```

说明：

- 启动后默认是手动模式。
- 在运行这个终端里按 `i` 切自动模式。
- 按 `q` 返回手动模式。

### 5.5 启动障碍物处理

终端 5：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav
source devel/setup.bash
roslaunch obstacle_processor obstacle_processor.launch
```

## 6. 第五步：给目标点并开始导航

启动完上面几个终端后：

1. 打开 RViz。
2. 使用 `3D Nav Goal` 工具。
3. 往 `/planning_3d_PRM_node/goal_pose` 发目标点。

启动 `planning_3d` 时会自动打开 RViz，默认配置在：

```bash
/home/arts-inpector/ART_3dnav/src/3D_NAV/global_planner/planning_3d/rviz/default.rviz
```

关键话题：

- 原始全局路径：`/planning_3d_PRM_node/planned_path`
- 平滑路径：`/path_smooth`
- 局部控制序列：`/local_plan`
- 速度输出：`/cmd_vel`
- 障碍物数组：`/obs_raw`
- 导航状态：`/navigation_state`
- 导航状态文字：`/navigation_state_debug`

## 7. 如果是实机 Go2，最后一步接底盘

如果你最终要把 `/cmd_vel` 送到底盘，还需要开 `go2_base_controller`。

建议先做一次只读连通性探测，确认 DDS/SDK 已经打通：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
python3 loco/sdk_probe.py --iface enp86s0 --peer 192.168.123.161
```

如果输出里 `ServiceList code` 不是 `0`，或者 `LowState messages` 一直是 `0`，先不要测 `/cmd_vel`，优先检查网线、网口 IP、机器人是否在线，以及到 `GO2_PEER` 的路由是否走了正确网卡。

终端 6：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
./loco/go2_base_control.sh start
```

如果需要指定 DDS 网卡和 peer：

```bash
source /opt/ros/noetic/setup.bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
GO2_IFACE=enp86s0 GO2_PEER=192.168.123.161 ./loco/go2_base_control.sh start
```

`go2_base_control.sh` 会自动检查 `GO2_PEER` 的主机路由是否走 `GO2_IFACE`。如果机器同时连了 Wi-Fi 和机器狗网口，它会优先把这条静态 peer 路由修正到指定网卡，避免 DDS/SDK 请求误走其他网卡。
它还会在启动前清理旧的 PID 文件对应进程，避免重复启动多个 `go2_base_controller` 实例同时消费 `/cmd_vel`。

启动后建议先做一个小步测试：

```bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
python3 loco/test_cmd_vel_forward.py --distance 0.3 --speed 0.1
```

确认底盘响应后，再让完整导航链路去驱动 `/cmd_vel`。

停止：

```bash
cd /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller
./loco/go2_base_control.sh stop
```

## 8. 最常用的排查命令

### 8.1 看 TF 和节点关系

```bash
rqt_graph
```

### 8.2 看全局路径有没有出来

```bash
rostopic echo /planning_3d_PRM_node/planned_path
```

### 8.3 看平滑路径有没有出来

```bash
rostopic echo /path_smooth
```

### 8.4 看局部控制有没有出来

```bash
rostopic echo /local_plan
```

### 8.5 看速度指令有没有发出

```bash
rostopic echo /cmd_vel
```

如果 `/cmd_vel` 正常但 Go2 不动，再补两条：

```bash
rostopic info /cmd_vel
python3 /home/arts-inpector/ART_3dnav/src/3D_NAV/go2_base_controller/loco/sdk_probe.py --iface enp86s0 --peer 192.168.123.161
```

判断原则：

- `rostopic info /cmd_vel` 里应该能看到 `go2_base_controller_ros1` 是订阅者
- `sdk_probe.py` 里 `ServiceList code` 应该是 `0`
- 如果控制器日志持续出现 `SLOW Move()`，大概率是 DDS/SDK 或网卡路由没有通到机器狗

### 8.6 看导航状态

```bash
rostopic echo /navigation_state
rostopic echo /navigation_state_debug
```

### 8.7 看障碍物输出

```bash
rostopic echo /obs_raw
```

## 9. 一份最小可执行顺序

如果你已经有：

- 原始地图 `src/data/point_cloud/scans.pcd`
- 轨迹 `src/data/trace_data/mapping_trajectory.txt`
- 或者已经有可通行区域 `src/data/traversable_cloud/traversable_areas.pcd`

那么最小执行顺序如下。

### 9.1 先做可通行区域提取

```bash
roslaunch map_process map_process_custom.launch \
  point_cloud_file:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
  trajectory_file:=/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt \
  output_file:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd \
  config_file:=/home/arts-inpector/ART_3dnav/src/3D_NAV/map_process/config/mid360_traversable_params.yaml \
  trajectory_format:=1
```

### 9.2 再启动导航

```bash
roslaunch fast_lio localization_mid360_nav.launch
```

```bash
roslaunch planning_3d 3d_planning.launch \
  pcd_path:=/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd
```

```bash
roslaunch bezier_path_optimizer bezier_path_optimizer.launch
```

```bash
roslaunch nmpc_planner nmpc_controller.launch
```

```bash
roslaunch obstacle_processor obstacle_processor.launch
```

## 10. 当前仓库里最容易卡住的地方

1. `map_process` 必须要有轨迹 txt，只有地图不够；现在建图 launch 会默认生成 `src/data/trace_data/mapping_trajectory.txt`。
2. 全局规划器读的是“可通行区域点云”，不是原始地图点云。
3. 文档和实际 `launch` 名称有少量不一致时，以仓库里的实际文件为准。
4. `nmpc_planner` 默认先手动模式，要在终端按 `i` 切自动。
5. 真机联调时，除了 ROS 节点，还要确认 `map -> base_link` 和雷达 TF 都是通的。

## 11. 本文档对应的主要入口文件

- 项目总入口：`src/3D_NAV/README.md`
- 架构说明：`src/3D_NAV/ARCHITECTURE.md`
- 定位模块：`src/3D_NAV/localization/README.MD`
- 可通行区域提取：`src/3D_NAV/map_process/README.md`
- 全局规划：`src/3D_NAV/global_planner/ReadMe.md`
- 局部规划：`src/3D_NAV/local_planner/README.md`
