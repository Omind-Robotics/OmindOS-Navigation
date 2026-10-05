# ROS 2 实机接入缺口分析

## 1. 已验证基线

当前 `nav3d_native` 已在常驻容器 `Nav_3d_ros2` 内通过：

- Flake8；
- 6/6 单元测试；
- `LaserScan -> OccupancyGrid -> 定位 -> A* -> Pure Pursuit -> cmd_vel`；
- `map -> odom -> base_link -> base_scan` TF；
- 自动验收和无残留进程退出。

此基线必须保留，新增实机包不得删除或绕过现有仿真验收。

## 2. 推荐接入路线

优先走“二维实机最小闭环”，再决定是否恢复原 ROS 1 三维规划能力：

```text
MID360 ROS 2 driver
  -> ROS 2 LIO / FAST-LIO
  -> PointCloud2 高度裁剪与 LaserScan 投影
  -> nav3d_native mapper/localizer
  -> nav3d_native A* + Pure Pursuit
  -> ROS 2 Go2 安全底盘桥
```

该路线可以复用当前已经验证的算法层。原工程的 PCD 可通行区域、三维 PRM、贝塞尔、NMPC
和动态障碍链作为第二阶段逐包迁移，不能通过简单修改启动命令获得 ROS 2 能力。

## 3. 组件缺口

| 组件 | 历史 ROS 1 状态 | 当前 ROS 2 状态 | 处理建议 | 优先级 |
|---|---|---|---|---|
| MID360 驱动 | `livox_ros_driver2` | 独立 ROS 2 副本已在派生镜像实机验证，含 SI 单位 IMU 适配 | 保持 `/livox/imu_raw` 与标准 `/livox/imu` 的边界并补长时间稳定性测试 | P0 |
| LIO | FAST-LIO 使用 roscpp/rospy | 无实机 LIO 包 | 采用可验证的 ROS 2 FAST-LIO，统一输出 `PointCloud2 + Odometry + TF` | P0 |
| 点云投影 | 历史链直接使用三维点云 | `/livox/scan` 已应用同安装 accepted 外参并在 `base_link` 实机验证 | 保持 `[-0.18, 0.30] m` 高度切片；机械安装变化时重新标定并复核过滤参数 | P0 |
| 机器人外参 | 历史 launch 发布部分静态 TF | 已导入同安装 `/home/nuc/Nav_test` 的 accepted 外参并实机验证静态链 | 后续只有机械安装变化时才重新标定；保持唯一 TF owner | P0 |
| 地图尺寸 | 历史使用 PCD | 当前二维地图固定约 12 m x 10 m | 将宽、高、分辨率和原点改为 ROS 2 参数 | P0 |
| 地图加载 | 历史 `map_publisher` 发布 PCD | 只保存 PGM/YAML，没有加载节点 | 增加 `nav2_map_server` 或原生地图加载节点 | P0 |
| 初始定位 | 历史链使用已知 TF/三维定位 | 当前首次扫描匹配窗口约 0.6 m | 增加 `/initialpose`、定位质量和丢失恢复机制 | P0 |
| 动态障碍 | `obstacle_processor` 使用 roscpp/PCL | 当前只对静态 OccupancyGrid 膨胀 | 移植成 rclcpp/PCL 或设计 ROS 2 局部代价图 | P1 |
| 三维全局规划 | PCD + PRM/A* + 坡度/净空 | 当前为二维栅格 A* | 二维实机通过后再移植，保持独立包 | P2 |
| 路径平滑 | 独立贝塞尔节点 | 当前 planner 内置简化/平滑 | 第一阶段保留当前实现；需要时再移植贝塞尔 | P2 |
| 局部控制 | CasADi NMPC | 当前 Pure Pursuit | 第一阶段低速使用当前控制器；之后做实机对比 | P1 |
| Go2 底盘桥 | `rospy + unitree_sdk2py` | 前后/旋转符号、H4 门禁和只读安全监督均通过；一次零 Move 路径静止，但单次 `StopMove` 返回 `-1`，SDK/TF/非零运动继续禁用 | 不重试或替换运动 API；先查清固件/服务端 `-1` 语义并重新审批，LIO 作为导航位姿 | P0 |
| RViz | ROS 1 RViz + 自定义 3D Goal | 当前镜像无 RViz2 | 调试阶段可在独立容器/工作站使用 RViz2 | P2 |
| 总 bringup | ROS 1 XML launch | 只有算法分段 launch | 新增 ROS 2 Python bringup，默认禁止底盘输出 | P1 |

## 4. 当前接口与实机提供方

| 当前接口 | 类型 | 实机提供方 |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | MID360 点云投影节点 |
| mapper `pose_topic` | `nav_msgs/msg/Odometry` | ROS 2 LIO 建图位姿 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | Go2 状态桥或 LIO 局部里程计 |
| `odom -> base_link` | TF2 | 唯一的局部里程计提供方 |
| `base_link -> base_scan` | TF2 | URDF 或静态外参发布器 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | mapper 或离线地图加载器 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz2、CLI 或任务层 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Go2 ROS 2 安全底盘桥消费 |

必须保证每条父子 TF 只有一个发布者。LIO、底盘桥和定位器不能重复发布同一变换。

## 5. 容器与依赖策略

- 不在宿主机安装 ROS、PCL、Livox 或 Unitree 依赖。
- 以现有镜像为基础构建新的硬件派生镜像，现有镜像保持不变。
- 所有新增源码只进入当前工程的独立 ROS 2 包，不修改其他工程。
- 不直接运行 `livox_ros_driver2/build.sh` 改写历史 ROS 1 参考目录；先提取独立副本。
- 仿真继续使用专用 bridge 网络和 `ROS_DOMAIN_ID=142`。
- 实机网络方案单独设计和审批，不在传感器、TF、安全门禁通过前启动 `/cmd_vel`。

## 6. 实机分阶段验收门槛

### H0：离线和只读环境

- 派生镜像冷构建成功；
- 原仿真验收继续 PASS；
- SDK probe 只读连通成功；
- 不修改宿主机路由和系统依赖。

### H1：传感器只读

- MID360 点云和 IMU 持续稳定；
- 频率、时间戳、frame_id、QoS 和 ROS SI 单位符合约定；
- 无丢包突增、时间反转或多余 ROS 1 节点。

### H2：LIO 与 TF

- 静止漂移、运动连续性满足现场阈值；
- `map/odom/base_link/lidar` 形成单一连通 TF 树；
- 保存 PCD、轨迹和 rosbag2 后可重复回放。

### H3：二维地图与定位

- 投影扫描稳定且不包含明显地面/机身点；
- 地图尺寸覆盖现场；
- 保存后能重新加载；
- 初始定位、持续定位和丢失检测均可验证。

### H4：规划控制但底盘禁用

- 目标能生成无碰路径；
- `/cmd_vel` 数值有限且停止条件正确；
- enable 默认为 false；
- watchdog 和急停测试通过。

### H5：低速实机

- 先零速度，再支架/悬空测试；
- 空旷区域限速短距离运动；
- 最后才允许完整导航接管 `/cmd_vel`。

任何阶段失败都回到前一门槛，不通过临时放宽急停、watchdog 或速度限制继续测试。
