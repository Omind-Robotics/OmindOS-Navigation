# PRM-A* + 五阶贝塞尔三维路径规划

## 节点与数据流
```
PCD 静态地图 → planning_3d_PRM_node (/planned_path) → bezier_path_optimizer_node (/path_smooth)
```

## 当前实现的功能
- **点云地图处理**：启动时读取 `pcd_path`，`VoxelGrid(voxel_leaf)` 降采样后构建体素；`/planning_3d_PRM_node/pcd_map` 为 latched 调试输出。
- **3D EDT 距离场**：支持 XY 外扩(`edt_xy_expand`)与 Z 厚度补偿(`edt_z_thickness`)，根据 `safe_margin` 过滤体素；若安全体素不足会自动降半阈值重试。可选头顶空隙检查(`enable_headroom_check`,`robot_height`)。
- **PRM+A***：一次性采样 `max_nodes`，限制坡度(`max_slope_deg`)，0.1 m 离散碰撞检测并记录每条边的最小/平均 clearance；A* 支持基于最小 clearance 的指数惩罚(`use_clearance_penalty`、`clearance_penalty_*`)或旧的 soft penalty。
- **起点 TF、目标话题**：实时监听 `map_frame`→`robot_frame` 作为起点，仅订阅目标 `goal_pose`。状态机 WAITING→GLOBAL_PLANNING→TRACKING→GOAL_ALIGN→COMPLETED/ABORTED，对应 `/navigation_state` (UInt8)/`/navigation_state_debug` (String)。
- **自动重规划与取消**：`auto_replan` 按 `replan_frequency` 频率定时再规划；`/navigation/cancel` 取消当前任务。
- **动态障碍**：`/obs_raw` (Float32MultiArray，序列化 xyz) 会在 `obstacle_block_radius` 范围内封禁邻近 PRM 顶点并触发重规划。
- **路径输出**：`/planned_path` 折线轨迹（z 额外抬升 0.35 m，末段写入目标姿态），`/prm_graph` 与 `/safe_centers` 用于 RViz 调试。
- **贝塞尔平滑**：`bezier_path_optimizer_node` 按 6 点滑窗拟合 5 阶贝塞尔，自动插入 Q1 辅助点，尾段阶数自适应，计算姿态（`end_strategy` 控制末端姿态保持）并发布 `/path_smooth`。

## 快速上手
1) **编译与环境**
```
cd /home/arts/Galilio/3D_nav_sim_ws
catkin_make
source devel/setup.bash
```
依赖（OSQP、PCL、OpenCV、nanoflann 等）由 CMakeLists 自动查找。

2) **启动规划器（终端1）**
```
roslaunch planning_3d 3d_planning.launch
```
默认载入 `/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/traversable_areas.pcd`，可在 launch 覆盖 `pcd_path`。需要 `map_frame`→`robot_frame` 的 TF，可用示例：`rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 100`。

3) **启动贝塞尔平滑（终端2）**
```
roslaunch bezier_path_optimizer bezier_path_optimizer.launch
```
已将 `/planned_path` remap 为 `/planning_3d_PRM_node/planned_path`。

4) **RViz 发送目标**
- 打开 `planning_3d/rviz/default.rviz`（随 launch 启动）。
- 选择工具栏的 “3D Nav Goal”（来自 `rviz-3d-nav-goal-tool` 包），Topic 设为 `/planning_3d_PRM_node/goal_pose`，Fixed Frame 选 `map`，在场景内拖拽发布。
- `/path_smooth` 为平滑轨迹，`/planned_path` 为原始折线。

5) **（可选）动态障碍**
设定 `obstacle_block_radius>0` 后发布障碍点：
```
rostopic pub /obs_raw std_msgs/Float32MultiArray "data: [x1, y1, z1, x2, y2, z2]"
```

## ROS 接口
- 订阅：`goal_pose` (geometry_msgs/PoseStamped)，`/obs_raw` (std_msgs/Float32MultiArray，可选)，TF `map_frame`→`robot_frame`。
- 发布：`/planned_path` (nav_msgs/Path)，`/pcd_map` (sensor_msgs/PointCloud2，latched)，`/prm_graph` (visualization_msgs/Marker)，`/safe_centers` (PointCloud2)，`/current_pose` (PoseStamped)，`/navigation_state` (UInt8)，`/navigation_state_debug` (String)。
- 服务：`/navigation/cancel` (std_srvs/Trigger)。
- 贝塞尔节点：订阅 `/planned_path`，发布 `/path_smooth`。

导航状态编码：0 WAITING，1 GLOBAL_PLANNING，2 TRACKING，3 GOAL_ALIGN，4 COMPLETED，5 ABORTED。

## 关键参数（可在 launch 覆盖）
- 地图/体素：`pcd_path`，`voxel_leaf`，`edt_xy_expand`，`edt_z_thickness`。
- 安全与代价：`safe_margin`，`use_clearance_penalty`，`clearance_penalty_weight`，`clearance_penalty_scale`，`use_soft_penalty`。
- PRM 建图：`step_size`，`max_nodes`，`k_neigh`，`max_slope_deg`。
- 运行与 TF：`map_frame`，`robot_frame`，`auto_replan`，`replan_frequency`，`pos_tolerance`，`pos_tolerance_exit`，`yaw_tolerance`，`obstacle_block_radius`。
- 贝塞尔：`sample_rate`，`q1_scale`，`q1_max_ratio`，`end_strategy`（same_as_prev | keep_original），`enable_orientation`（当前实现默认计算姿态）。

## 使用提示
- 安全体素 <100 时会自动降半安全距离再筛选，仍不足时退回自由体素。
- 规划阶段 z 方向上移 0.35 m，确保离开地面栅格再由贝塞尔平滑。
- 接近目标后进入 GOAL_ALIGN，仅当偏航误差 < `yaw_tolerance` 或调用 `/navigation/cancel` 才结束。
