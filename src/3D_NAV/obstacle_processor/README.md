# Obstacle Processor

3D 障碍物处理功能包：基于激光雷达点云与全局地图，检测并膨胀障碍物，输出可视化点云和数组数据。

## 功能概述
- 订阅激光雷达点云(`/points_e1r_front`)与世界点云地图(`/map`)，支持 remap。
- 自定义体素降采样与手动 PassThrough 滤波，减少点数并限制处理范围。
- 构建局部地图并缓存：机器人移动超过 `cacheThreshold` 才重建。
- 基于局部地图的 2D 网格索引+分层高度聚类，加速高度差障碍物判定；索引缺失时回退到直接邻域搜索。
- TF 批量变换到 `map_frame_id`，OpenMP 并行检测，线程安全的双缓冲世界地图。
- 障碍物 8 邻域膨胀（`expansion = resolution * expansionCoefficient`），同时发布膨胀点云、成本点云与原始数组。

## 处理流程
1) 接收激光雷达点云，执行自定义体素降采样与 PassThrough。  
2) 在局部 `World` 网格中去重/栅格化，得到滤波点云。  
3) 查询 TF 获取机器人位置并裁剪全局地图为局部地图，构建二维网格索引和高度层信息。  
4) 将滤波点云批量变换到地图系，在网格索引（或回退搜索）中对比高度差判定障碍物。  
5) 对障碍物点进行 8 邻域膨胀，生成可视化点云、成本点云和数组数据并发布。  
6) 固定发布频率由 `publish_rate` 控制（默认 2 Hz）。

## 话题接口
| 类型 | 名称 | 消息 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/points_e1r_front` | sensor_msgs/PointCloud2 | 前置雷达点云，launch 可 remap |
| 订阅 | `/map` | sensor_msgs/PointCloud2 | 世界/全局地图点云，双缓冲读取 |
| 发布 | `obs_vis` | sensor_msgs/PointCloud2 | 膨胀后的可视化障碍物点云（地图系） |
| 发布 | `obs_cost` | sensor_msgs/PointCloud2 | 未膨胀的障碍物中心点（地图系） |
| 发布 | `/obs_raw` | std_msgs/Float32MultiArray | 膨胀点的扁平化数组（xyz 顺序排列） |

> 坐标系由参数 `map_frame_id` / `lidar_frame_id` / `base_frame_id` 指定，默认 `"map"`, `"e1r_front"`, `"base_link"`。

## 参数（私有命名空间 `~map/*`）
- 坐标系：`map_frame_id`，`lidar_frame_id`，`base_frame_id`
- 栅格与膨胀：`resolution`(默认 0.3)，`expansionCoefficient`(默认 1.0)
- 预处理：`leaf_size`(默认 0.2)，`local_x_l/u`，`local_y_l/u`，`local_z_l/u`（手动 PassThrough 范围）
- 局部地图：`localMapSize_x/y/z_l/z_u`，`cacheThreshold`（机器人位移阈值，默认 1.0m）
- 高度检测：`height_threshold_grid`（网格索引判定阈值），`height_threshold_fallback`（回退判定阈值），`z_compensation`（雷达到基座高度补偿）
- 多层聚类：`layer_gap_threshold`，`layer_merge_min_points`，`layer_height_tolerance`
- 发布频率：`publish_rate`（默认 2.0 Hz）

膨胀步长 `expansion = resolution * expansionCoefficient`，影响 `obs_vis` 点云和 `/obs_raw` 数组。

## 构建
```bash
cd /home/arts/Galilio/3D_nav_sim_ws
catkin_make
```
依赖：ROS (roscpp/rospy/std_msgs/sensor_msgs/geometry_msgs/tf)、PCL>=1.10、Eigen3、OpenMP。C++14 构建，自动检测 AVX2/SSE2。

## 运行
### 使用 launch（推荐）
```bash
roslaunch obstacle_processor obstacle_processor.launch
```
如需更换雷达或地图话题，可在启动文件中添加 remap，例如：
```xml
<remap from="/points_e1r_front" to="/your_lidar_points"/>
<remap from="/map" to="/your_map_cloud"/>
```
参数可直接在 launch 中调整（已给出默认值）。

### 直接运行节点
```bash
rosrun obstacle_processor obstacle_processor_node \
  _map/resolution:=0.3 _map/leaf_size:=0.2 _map/map_frame_id:=map ...
```
请在运行前确保参数已设置（`rosparam load` 或命令行覆盖），并确保 TF 中存在 `map_frame_id -> base_frame_id -> lidar_frame_id` 的变换链。

## 文件结构
```
obstacle_processor/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/obstacle_processor/
│   ├── execution_classes.h
│   └── backward.hpp
├── src/
│   ├── obstacle_processor_node.cpp
│   └── execution_classes.cpp
└── launch/
    └── obstacle_processor.launch
```
