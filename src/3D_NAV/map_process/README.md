# Map Process - 可通行区域提取包

## 概述
Map Process 是一个基于 ROS 与 PCL 的离线点云处理包，读取磁盘上的地图点云（PCD）与文字轨迹文件，使用轨迹驱动的种子点和区域生长算法提取可通行区域并输出新的 PCD 文件。当前节点不订阅话题，所有输入输出均通过文件完成。

## 已实现功能
- 离线文件驱动：从 `point_cloud_file` 读取 PCD 地图，从 `trajectory_file` 读取轨迹，支持 3 种轨迹文本格式。
- 轨迹采样与种子点生成：按距离间隔 `trajectory_sample_interval` 采样轨迹，在 `seed_search_radius` 平面范围、`seed_height_offset` 高度窗口内搜寻种子点。
- 点云预处理：剔除 NaN/inf 点；当点数超过 `downsampling_threshold` 且使能 `enable_voxel_downsampling` 时按 `voxel_size` 进行体素下采样。
- 地面过滤：对整云 K=50 估计法向；基于法向垂直度 `max_ground_normal_angle` 过滤地面，可选的高度约束由 `enable_height_filter`、`seed_xy_search_radius` 与 `seed_height_diff_threshold` 控制。
- 区域生长：对过滤后的地面点执行 PCL `RegionGrowing`，使用 `normal_angle_threshold`、`curvature_threshold`、`height_threshold`、`search_radius` 等约束；仅保留包含至少一个种子点且点数大于 `min_region_size` 的簇。
- 结果输出：将所有有效簇合并为可通行区域点云并保存到 `output_file`；控制台输出处理时间/点数/簇数统计；可选 `save_individual_regions` 另存每个簇。
- 调试输出：种子点与地面候选点默认保存在 `output_file` 同目录，也可通过 `debug_seed_points_file` 和 `debug_ground_candidates_file` 指定。

## 输入数据与格式
- 点云：PCD 文件，需为 `pcl::PointXYZI` 类型。
- 轨迹：纯文本，按 `trajectory_format` 选择解析方式  
  1. `timestamp x y z qx qy qz qw`  
  2. `x y z param1 param2 param3`（仅使用前三列位置，姿态置为单位四元数）  
  3. 位置在第 5-7 列，其余列忽略。
- 配置：默认由 `config/default_params.yaml` 加载，可使用 `config/test_params.yaml`（简化测试参数）或 `config/test_format3_params.yaml`（轨迹格式 3 示例）替换。

## 处理流程
1. **读取与预处理**：加载 PCD，移除非法点，必要时体素下采样。
2. **轨迹采样与种子生成**：按距离采样轨迹，在水平半径与高度偏移窗口内搜寻地面候选点作为种子。
3. **法向估计与地面过滤**：对整云估计法向；依据法向角度及可选高度差过滤出地面候选点。
4. **区域生长**：在地面点上运行区域生长，筛选包含种子点的簇并按 `min_region_size` 过滤。
5. **输出**：合并有效簇生成可通行区域点云，保存结果并打印统计；如需要可保存各簇。

## 关键参数（默认值以 `config/default_params.yaml` 为准）
- **文件路径**：`point_cloud_file`、`trajectory_file`、`output_file`、`trajectory_format`
- **轨迹与种子**：`trajectory_sample_interval`、`seed_search_radius`、`seed_height_offset`
- **地面过滤**：`max_ground_normal_angle`、`enable_height_filter`、`seed_xy_search_radius`、`seed_height_diff_threshold`
- **区域生长**：`normal_angle_threshold`、`curvature_threshold`、`height_threshold`、`search_radius`、`min_cluster_size`、`max_cluster_size`、`normal_search_radius`、`min_region_size`
- **下采样**：`enable_voxel_downsampling`、`voxel_size`、`downsampling_threshold`
- **输出**：`save_individual_regions`
- **保留但当前未应用的参数**：`merge_distance_threshold`、`remove_outliers`（代码未使用，无合并/离群点去除逻辑）

## 编译与运行
```bash
cd /path/to/your/catkin_ws
catkin_make
source devel/setup.bash
```

- 使用默认配置运行（加载 `config/default_params.yaml`）：
  ```bash
  roslaunch map_process map_process_custom.launch
  ```
- 使用自定义数据或配置：
  ```bash
  roslaunch map_process map_process_custom.launch \
    point_cloud_file:=/abs/path/to/map.pcd \
    trajectory_file:=/abs/path/to/traj.txt \
    output_file:=/abs/path/to/output/traversable_areas.pcd \
    config_file:=/abs/path/to/config.yaml
  ```
- 直接 rosrun 并通过私有参数覆盖路径或开关：
  ```bash
  rosrun map_process map_process_node \
    _point_cloud_file:=/abs/path/to/map.pcd \
    _trajectory_file:=/abs/path/to/traj.txt \
    _output_file:=/abs/path/to/output/traversable_areas.pcd \
    _trajectory_format:=1 \
    _enable_height_filter:=false
  ```

## 输出文件
- 主结果：可通行区域点云保存到 `output_file`。
- 可选：设置 `save_individual_regions:=true` 时，每个簇会按 `region_<id>.pcd` 保存在输出目录。
- 调试点云：默认保存在主输出旁边，可通过 ROS 私有参数覆盖路径。

## 注意事项与限制
- 仓库未提供示例数据，需自备 PCD 地图和轨迹文件，并确保轨迹格式与 `trajectory_format` 一致。
- 当前实现仅做文件读写，不发布/订阅 ROS 话题。
- `merge_distance_threshold`、`remove_outliers` 等参数尚未在代码中生效，如需相应功能需进一步开发。
