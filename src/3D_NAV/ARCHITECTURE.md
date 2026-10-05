# 3D_NAV 项目架构与开发流程

本文档用于快速理解当前工作区的模块划分、运行链路、关键数据流，以及日常开发时应该先看哪些文件。

## 1. 项目定位

这个项目不是“在线建图后直接导航”的一体化导航栈，而是明显分成两段：

1. **离线地图准备**
   - 用激光 SLAM 或定位相关模块产出场景点云地图。
   - 用 `map_process` 根据历史轨迹，从原始场景点云中提取“机器人实际走过/可走”的区域。
   - 输出 `traversable_areas*.pcd`，作为后续全局规划器的输入。

2. **在线导航执行**
   - `planning_3d` 在启动时一次性加载 `traversable_areas*.pcd`，构建安全体素和 PRM 图。
   - `bezier_path_optimizer` 对全局折线路径做平滑。
   - `nmpc_planner` 将平滑路径转成速度控制。
   - `obstacle_processor` 在线检测动态障碍，回灌给全局/局部规划。

核心设计前提是：

- **全局规划器输入的是“可通行点云”，不是障碍物点云。**
- 也就是说，`planning_3d` 把 PCD 中存在的体素视为**自由空间/路面**，其余未覆盖区域默认视为不可走。

## 2. 工作区结构

### 2.1 核心包

- `localization/`
  - 负责建图、定位、地图发布。
  - 主要包含 `FAST_LIO`、`galileo_lio`、`open3d_loc`、`map_publisher`、`livox_ros_driver2`。
- `map_process/`
  - 离线可通行区域提取。
  - 输入原始地图 PCD + 轨迹 txt，输出 `traversable_areas*.pcd`。
- `global_planner/planning_3d/`
  - 全局规划核心，采用“可通行体素 + 3D EDT + PRM + A*”。
- `global_planner/bezier_path_optimizer/`
  - 对 `/planned_path` 做贝塞尔平滑，输出 `/path_smooth`。
- `local_planner/nmpc_planner/`
  - 将 `/path_smooth` 转成 `/local_plan`，并由控制器发布 `/cmd_vel`。
- `obstacle_processor/`
  - 在线障碍检测，发布 `/obs_raw` 给全局规划和局部规划。
- `go2_base_controller/`
  - 底盘/运动控制相关脚本。
  - 负责把 ROS `/cmd_vel` 桥接到 Go2 的 Unitree SDK `SportClient.Move()`。
  - 实机上这一层依赖 DDS/SDK 连通性；如果主机有多个网卡，到 Go2 peer 的路由必须走机器狗网口。

### 2.2 建议的阅读顺序

1. 根入口和启动顺序：`README.md`
2. 离线地图处理：`map_process/README.md`
3. 全局规划：`global_planner/ReadMe.md`
4. 局部规划：`local_planner/README.md`
5. 关键 launch
6. 对应节点入口源码

## 3. 总体运行链路

### 3.1 离线链路

```text
激光/SLAM建图
  -> 原始场景点云 map.pcd
  -> 轨迹文件 trajectory.txt
  -> map_process
  -> traversable_areas*.pcd
```

### 3.2 在线链路

```text
定位/TF(map -> base_link)
  + traversable_areas*.pcd
  -> planning_3d
  -> /planned_path
  -> bezier_path_optimizer
  -> /path_smooth
  -> nmpc_local_planner
  -> /local_plan
  -> nmpc_controller
  -> /cmd_vel
  -> go2_base_controller
  -> Go2 SportClient.Move()
```

补充约束：

- `/cmd_vel` 链路跑通，不等于 Go2 一定会动；底盘桥还必须成功连接 Unitree SDK/DDS。
- 实机建议始终通过 `go2_base_control.sh` 启动底盘桥，而不是手工常驻多个 Python 进程。

动态障碍旁路：

```text
激光点云 + /map
  -> obstacle_processor
  -> /obs_raw
  -> planning_3d (封禁PRM节点/触发重规划)
  -> nmpc_local_planner (MPC避障代价)
```

## 4. 建图、可通行区域提取、全局规划的关键数据流

这部分是当前项目最重要的主链路。

### 4.1 第一步：建图

建图能力主要在 `localization/` 下。

- `FAST_LIO` / `galileo_lio`
  - 负责激光里程计、建图或定位。
- `open3d_loc`
  - 负责基于离线地图的全局定位。
- `map_publisher`
  - 在仿真或某些实机流程里，把静态地图发布到 `/map`，主要给障碍物处理用。

这里产出两类数据：

1. **原始场景点云地图**
   - 例如 `map.pcd`、`map.ply`
   - 表示整个场景的几何结构
2. **轨迹数据**
   - 来自 SLAM 或定位记录
   - 作为 `map_process` 提取可通行区域的先验

注意：

- 导航使用的不是原始地图，而是从原始地图里进一步提取后的**可通行面地图**。

### 4.2 第二步：可通行区域提取

关键入口：

- `map_process/launch/map_process_custom.launch`
- `map_process/src/main.cpp`
- `map_process/src/traversable_extractor.cpp`
- `map_process/src/trajectory_processor.cpp`

处理流程如下：

1. **读取文件输入**
   - `point_cloud_file`：原始场景地图 PCD
   - `trajectory_file`：轨迹文本
   - `output_file`：输出可通行区域 PCD

2. **轨迹采样**
   - 按 `trajectory_sample_interval` 从原始轨迹抽样。

3. **生成种子点**
   - 以采样轨迹点为中心，按 `seed_search_radius` 和 `seed_height_offset` 在点云中找“脚下地面候选点”。
   - 这些点不是最终结果，而是后续区域生长的“可信起点”。

4. **点云预处理**
   - 去除 NaN/Inf。
   - 大点云按 `voxel_size` 做体素降采样。

5. **法向估计与地面候选过滤**
   - 对整云做法向估计。
   - 用 `max_ground_normal_angle` 约束表面法向，保留接近地面/坡面的点。
   - 若 `enable_height_filter=true`，还会结合轨迹种子做高度一致性过滤。

6. **区域生长**
   - 在过滤后的地面候选点上做 `RegionGrowing`。
   - 通过法向相似、曲率阈值等条件聚类。
   - 默认只保留包含种子点的区域；若 `require_seed_containment=false`，则可放宽。

7. **合并输出**
   - 将保留的区域合并为 `traversable_cloud_`。
   - 保存为 `traversable_areas*.pcd`。

### 4.3 这里的数据语义

`map_process` 输出的 PCD 语义是：

- **这些点代表机器人可走的面/体素中心采样。**
- 不是完整世界模型。
- 不是障碍物。
- 也不是 OctoMap 或占据栅格。

这决定了 `planning_3d` 的解释方式。

### 4.4 第三步：全局规划

关键入口：

- `global_planner/planning_3d/launch/3d_planning.launch`
- `global_planner/planning_3d/src/planning_3d_PRM.cpp`
- `global_planner/planning_3d/include/edt_3d.h`

处理流程如下：

1. **启动时读取可通行 PCD**
   - `pcd_path` 指向 `traversable_areas*.pcd`
   - 启动后一次性加载，不是在线订阅

2. **体素化**
   - 对输入点云先做 `VoxelGrid`
   - 再将每个点落入离散体素 `VoxelKey(x,y,z)`
   - 存在命中的体素被视为**自由体素**

3. **构建 3D EDT 距离场**
   - 将自由体素写入三维二值体素网格
   - XY 方向做范围扩展，Z 方向做厚度扩展
   - 计算每个自由体素到边界/障碍的欧式距离

4. **筛选安全体素**
   - 保留 EDT 距离大于 `safe_margin` 的体素
   - 可选做头顶净空检查 `hasHeadroom()`
   - 得到 `safe_centers_`

5. **在安全体素上建 PRM**
   - 从 `safe_centers_` 随机采样 `max_nodes` 个 PRM 节点
   - 用半径搜索和 `k_neigh` 连边
   - 连边时检查：
     - 坡度限制 `max_slope_deg`
     - 沿边离散碰撞
     - 沿边最小 clearance 是否满足 `safe_margin`

6. **A* 搜索**
   - 起点来自实时 TF：`map_frame -> robot_frame`
   - 终点来自 `goal_pose`
   - 起终点先吸附到最近 PRM 节点，再做 A*
   - 边代价可叠加基于 clearance 的惩罚

7. **发布路径与状态**
   - 输出 `/planned_path`
   - 发布 `/navigation_state`
   - 进入 `TRACKING` 后可定时重规划

### 4.5 map_process 到 planning_3d 的数据流

这一段可以总结成一句话：

> `map_process` 负责把“原始场景地图”压缩成“可行走空间样本”；`planning_3d` 再把这些样本恢复成离散自由空间、距离场和 PRM 图。

具体是：

```text
原始地图 map.pcd
  + 历史轨迹 trajectory.txt
  -> 提取出 traversable_areas.pcd
  -> 体素化为 free voxels
  -> EDT 计算 clearance
  -> safe_centers
  -> PRM graph
  -> A* path
```

这也是当前项目最值得牢记的“地图语义转换”。

## 5. 平滑、局部规划与状态机

### 5.1 贝塞尔平滑

- `bezier_path_optimizer` 订阅 `/planning_3d_PRM_node/planned_path`
- 输出 `/path_smooth`
- 主要作用：
  - 把折线路径变成更平滑的空间曲线
  - 自动补姿态
  - 保留末端姿态约束

### 5.2 局部规划

`nmpc_planner` 分两部分：

1. `nmpc_local_planner_node`
   - 订阅 `/path_smooth`、`/curr_state`、`/obs_raw`、`/navigation_state`
   - 输出 `/local_plan` 和 `/local_path`
2. `nmpc_controller_node`
   - 读取 `/local_plan`
   - 发布 `/cmd_vel`
   - 同时把 TF 里的当前状态发布为 `/curr_state`

状态机统一使用：

- `WAITING`
- `GLOBAL_PLANNING`
- `TRACKING`
- `GOAL_ALIGN`
- `COMPLETED`
- `ABORTED`

全局规划和局部规划靠 `/navigation_state` 解耦。

## 6. 开发流程建议

建议把日常开发拆成两条流程。

### 6.1 离线地图链路开发

适合修改 `localization/`、`map_process/`、`planning_3d` 地图相关逻辑。

1. 先准备原始地图 PCD 和轨迹文件
2. 运行 `map_process`
3. 检查输出的 `traversable_areas*.pcd`
4. 用该结果启动 `planning_3d`
5. 在 RViz 检查：
   - `pcd_map`
   - `safe_centers`
   - `prm_graph`
   - `planned_path`
6. 再调整参数

优先调的参数：

- `map_process`
  - `seed_search_radius`
  - `seed_height_offset`
  - `max_ground_normal_angle`
  - `enable_height_filter`
  - `trajectory_sample_interval`
- `planning_3d`
  - `voxel_leaf`
  - `safe_margin`
  - `edt_z_thickness`
  - `max_nodes`
  - `k_neigh`
  - `max_slope_deg`

### 6.2 在线导航链路开发

适合修改 `planning_3d`、`bezier_path_optimizer`、`nmpc_planner`、`obstacle_processor`。

推荐启动顺序：

1. 定位或地图发布
2. `planning_3d`
3. `bezier_path_optimizer`
4. `nmpc_planner`
5. `obstacle_processor`

调试时先保证：

- TF 正常：`map -> base_link`
- `planning_3d` 能在 RViz 发目标后稳定出 `/planned_path`
- 平滑器能输出 `/path_smooth`
- 局部规划器能收到 `/curr_state`
- 控制器能收到 `/local_plan`

## 7. 常见修改入口

### 7.1 修改可通行区域提取策略

优先看：

- `map_process/src/traversable_extractor.cpp`
- `map_process/src/trajectory_processor.cpp`
- `map_process/config/default_params.yaml`
- `map_process/config/mid360_traversable_params.yaml`

### 7.2 修改全局规划地图解释方式

优先看：

- `planning_3d/src/planning_3d_PRM.cpp`
- `planning_3d/include/edt_3d.h`

重点函数：

- 体素化与 EDT 构建
- `buildPRM()`
- `isEdgeFree()`
- `computePath()`
- `hasHeadroom()`

### 7.3 修改全局路径平滑

优先看：

- `bezier_path_optimizer/src/bezier_path_optimizer.cpp`

### 7.4 修改局部跟踪与避障

优先看：

- `nmpc_planner/src/local_planner.cpp`
- `nmpc_planner/src/controller_node.cpp`
- `nmpc_planner/src/mpc_solver.cpp`

## 8. 当前实现的几个重要注意点

1. `map_process` 是**离线文件流**，不是在线 ROS 图处理节点。
2. `planning_3d` 读取的是**可通行面地图**，不是全局障碍地图。
3. `planning_3d` 启动时一次性建图和建 PRM，地图变化后通常需要重启节点或重载输入。
4. 动态障碍目前通过 `/obs_raw` 做节点封禁和 MPC 代价，不会在线重建整张 PRM 图。
5. 局部规划器对 `/path_smooth` 从索引 10 开始取点，属于实现细节，改路径采样逻辑时需要一起检查。

## 9. 推荐的日常开发习惯

1. 先确认你改的是“离线地图链路”还是“在线导航链路”。
2. 先看对应 launch，再看节点入口，再看核心类。
3. 改 `map_process` 后，先在 PCD 结果上验证，再联调规划器。
4. 改 `planning_3d` 后，先看 `safe_centers` 和 `prm_graph` 是否符合预期。
5. 改局部规划前，先确认 `/path_smooth` 和 `/navigation_state` 是正确的。

## 10. 关键文件索引

- 根入口：`README.md`
- 架构文档：`ARCHITECTURE.md`
- 可通行区域提取：`map_process/README.md`
- 全局规划：`global_planner/ReadMe.md`
- 局部规划：`local_planner/README.md`
