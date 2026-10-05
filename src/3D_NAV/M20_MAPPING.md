# M20 融合建图与录包

本流程用于外出采集。FAST-LIO 默认使用前、后雷达同步后的
`/m20/lidar/fused` 建图，同时用 rosbag 保存前雷达、后雷达、融合点云和 IMU，
便于回来后重新测试。

## 1. 启动传感器通信

在 NUC 终端执行：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh start  # reads exported M20_SSH_PASSWORD when required
./scripts/m20_sensor_transport.sh status
```

## 2. 启动融合建图

新开终端执行：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch fast_lio mapping_m20.launch
```

该 launch 已默认执行以下操作：

- 启动前、后雷达融合节点，发布 `/m20/lidar/fused`。
- FAST-LIO 使用 `/m20/lidar/fused` 和 `/IMU` 建图。
- 启动 RViz。
- 保存点云和轨迹。

开始移动前，确认融合点云约为 10 Hz：

```bash
rostopic hz /m20/lidar/fused
```

## 3. 启动 rosbag 录制

前一步启动建图后，所需话题已经存在。再新开一个终端，直接执行：

```bash
cd /home/arts-inpector/ART_3dnav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
BAG_DIR=/home/arts-inpector/ART_3dnav/src/data/bags
mkdir -p "${BAG_DIR}"
rosbag record --lz4 --split --size=4096 --min-space=10G \
  --buffsize=1024 --repeat-latched \
  -O "${BAG_DIR}/m20_mapping_$(date +%Y%m%d_%H%M%S)" \
  /m20/lidar/front /m20/lidar/rear /m20/lidar/fused /IMU \
  /tf /tf_static /Odometry_loc /rosout_agg
```

这条命令记录前雷达、后雷达、融合点云、IMU、TF 和 FAST-LIO 里程计。数据使用
LZ4 压缩、每 4096 MB 自动分包，并在磁盘剩余 10 GB 时停止。文件保存在
`src/data/bags/m20_mapping_<时间>*.bag`，预计每分钟占用约 `2.1-2.4 GB`。

需要写入外接盘时，只修改命令中的 `BAG_DIR`：

```bash
BAG_DIR=/media/arts-inpector/<外接盘目录>
```

## 4. 建图结束

1. 停稳 M20。
2. 先在 rosbag 终端按 `Ctrl+C`，等待索引写完。
3. 再在 FAST-LIO 终端按 `Ctrl+C`，等待 PCD 保存完成。
4. 最后停止通信：

```bash
cd /home/arts-inpector/m20pro_ros2_mock
./scripts/m20_sensor_transport.sh stop
```

建图输出：

```text
/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd
/home/arts-inpector/ART_3dnav/src/data/trace_data/mapping_trajectory.txt
```

## 5. 提取前雷达可通行区域

当前这轮 `scans.pcd` 是前雷达建图结果。确认 FAST-LIO 已经退出且
PCD 保存完成后，使用 `map_process_test` 算法处理：

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

点云与轨迹必须来自同一轮建图。处理完成后日志会出现
`Processing Complete`，节点随后自动退出。检查输出：

```bash
ls -lh /home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd
```

全局规划使用该文件时，需要重启 `planning_3d` 以重新加载 PCD 并构建 PRM。

## 6. 回来后回放

直接使用包内融合点云测试：

```bash
roslaunch fast_lio mapping_m20.launch \
  start_lidar_fusion:=false lidar_topic:=/m20/lidar/fused \
  rviz:=true pcd_save_en:=false record_trajectory:=false

rosbag play --delay=2 <bag文件> --topics /m20/lidar/fused /IMU
```

使用前、后原始点云重新融合测试：

```bash
roslaunch fast_lio mapping_m20.launch \
  rviz:=true pcd_save_en:=false record_trajectory:=false

rosbag play --delay=2 <bag文件> --topics \
  /m20/lidar/front /m20/lidar/rear /IMU
```

回放时不要播放包内的 `/Odometry_loc` 和 `/tf`，否则会与新运行的 FAST-LIO 输出
冲突。
