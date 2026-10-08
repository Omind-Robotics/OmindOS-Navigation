# OmindOS Navigation：轮式与四足统一平台预览

本次以 `b5881e32d69698e7135aa5e40b3b697e9a1e8603` 为开发基线。
目标是一个导航发行版、一个任务接口、多种底盘适配。本次交付为研发预览；
未创建新 Release、未修改 v0.1 资产、未给任何实机发送运动命令。

## 1. 平台与能力边界

| 配置 | 命令能力 | 预览限速（m/s、rad/s） | 实机所需下游 |
| --- | --- | --- | --- |
| `wheeled_diff` | 前后、转向；拒绝横移 | x 0.60 / y 0 / yaw 1.20 | 差速底盘驱动 |
| `wheeled_omni` | 前后、横移、转向 | x 0.60 / y 0.40 / yaw 1.20 | 全向轮底盘驱动 |
| `quadruped` | 机身前后、横移、转向 | x 0.45 / y 0.20 / yaw 1.00 | 自研四足步态控制器及电机/CAN 驱动 |

表中数值是通用仿真示例，不代表任何实体机器人的额定能力。
配置文件为 `src/nav3d_native/nav3d_native/platform_profiles.json`；支持通过
`profile_file:=/absolute/path/profiles.json` 加载经校验的外部配置。
同一导航器目前生成前后与转向命令；横移在接口和运动学模拟器支持，
不表示全向路径规划或四足落足点规划已经实现。四足预览没有关节、接触、平衡或步态动力学。
Go2 历史桥不会由此入口启动，自研四足仍是后续独立适配对象。

## 2. 一套启动入口

在 ROS 2 Humble 环境中构建开发预览（仅选择原生演示包）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select nav3d_native
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=171
ros2 launch nav3d_native platform_preview.launch.py platform:=wheeled_diff
```

将最后一个参数改为 `wheeled_omni` 或 `quadruped` 即使用同一核心运行不同配置。
这个入口只运行确定性的平面建图、定位、A* 与跟踪演示，不加载 Livox、FAST-LIO、
三维 PRM、C++ NMPC 或厂商 SDK。它不能代替完整导航发行包构建和实机验收。
仿真建图阶段使用预设采样位姿，属于已有演示机制，不是自主探索或真实采集。

全部预览话题及 TF 已移至 `/omindos_preview/`。轮式与四足共用：

| 话题（省略 `/omindos_preview` 前缀） | 类型 | 用途 |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/PoseStamped` | map 坐标系导航目标 |
| `/navigation/cmd_vel` | `geometry_msgs/Twist` | 导航器输出 |
| `/robot/cmd_vel` | `geometry_msgs/Twist` | 经平台检查后给运动学模拟器的机身速度 |
| `/odom` | `nav_msgs/Odometry` | odom → base_link，名称不再绑定轮式 |
| `/localization/pose` | `geometry_msgs/PoseStamped` | map 坐标系定位 |
| `/plan` | `nav_msgs/Path` | 路径与更新时刻 |
| `/emergency_stop` | `std_msgs/Bool` | true 锁定急停，false 不解锁；需重启预览 |
| `/platform_status` | `std_msgs/String` | 配置、检查状态、实际输出、preview_only |

## 3. 停止条件

适配节点单独启动时默认关闭运动；预览 launch 只为同一入口创建的模拟器显式开启。
适配器按 50 Hz 检查：命令 0.25 秒、定位 0.75 秒、里程计 0.5 秒、路径 2 秒。
任一来源缺失、无效或超时，立即输出零速度。带 Header 的消息还检查源时间戳、
坐标系和有限位姿，消息传输延迟计入有效期。空路径、非有限数值、非平面命令
和差速底盘横移命令会被拒绝。超限命令整体等比例缩放，保持前进/转向比例。
模拟器另有 0.3 秒命令超时，防止适配器退出后一直使用旧速度。

这些检查仅属于预览软件边界；没有接入硬件急停、关节状态或实体碰撞包络。
不得把预览输出直接接到实机。实体驱动必须有自己的命令有效期、失联保护、
姿态/电量/运动模式反馈与停止确认，并完成对应机器人的现场测试。
原有 C++ NMPC 与 Go2 历史链未在本次修改中获得这些检查。

## 4. 验证方法

```bash
python3 -m pytest src/nav3d_native/test
bash scripts/validate_platform_preview.sh
```

纯逻辑测试可在无 ROS 环境中运行：

```bash
PYTHONPATH=src/nav3d_native python3 -m unittest discover \
  -s src/nav3d_native/test -p 'test_platform_*.py' -v
```

没有 ROS 时消息测试明确跳过，不将它算作通过。真实 ROS 消息测试会中断命令、
定位、里程计、路径更新，注入空路径、过期位姿、错误坐标系和 NaN，并检查恢复与急停锁定。
三种配置的端到端演示必须到达目标且连续至少一秒输出零速度，并保持新鲜定位及真值反馈。
验收仅涵盖该仿真场景，不包含地形、动态障碍、末端朝向、多点巡检或实体停止精度。

## 5. 正式发行前剩余工作

1. 接入目标轮式驱动及自研四足步态控制器，确定型号、机载架构、TF、传感器外参和实体限值。
2. 整合并验证实际 C++ 导航链、重定位、任务结果/取消、多点巡检与返航、底盘故障反馈。
3. 按型号验证静态/动态包络和地形通行能力；四足步态由底层控制器单独验收。
4. 完成现有分发审计的第三方来源/许可待办，再构建干净、可复现的新镜像、清单和校验值。
5. 每个型号单独记录实机支持状态；研发预览通过不自动提升为实机支持。

当前仓库还保留首次导入源码的 bootstrap 工作流，其中 `rsync --delete` 会覆盖后续开发。
本次取消其 push 自动触发，手动重新导入也需要显式勾选确认，避免后续合并时丢失平台适配。
开发/公开发行仓库的可见性拆分不由此预览改动执行。

## 6. 本次验证记录（2026-10-08）

在现有腾讯服务器的独立容器中完成。使用 ROS 2 Humble；基础镜像本地 ID 为
`sha256:7074a696598c73e8e922534efb02cc17e6539b27bb91677de9911d77a8d6de2b`。
容器使用 `--network none`、`ROS_LOCALHOST_ONLY=1` 和独立域 171，移除 capabilities，
未挂载机器人设备、未连接真实传感器或电机。测试容器退出后已删除。

- 指定包重新构建成功。
- pytest：20 项通过（6 项原有算法、10 项平台逻辑、2 项 ROS 消息失效/急停、
  2 项模拟器横移/发送方消失后停止）。每项包含的子场景不另计数。
- 原 `demo.launch.py` 基线仍为 PASS；其原验收语义不要求一秒连续零指令。
- 三种统一平台配置均通过目标到达与一秒连续零指令门槛：

| 配置 | 结果 | 本次耗时（s） | 目标位置误差（m） | 连续零指令（s） |
| --- | --- | ---: | ---: | ---: |
| `wheeled_diff` | PASS | 26.01 | 0.255 | 1.81 |
| `wheeled_omni` | PASS | 25.01 | 0.257 | 1.08 |
| `quadruped` | PASS | 28.51 | 0.237 | 1.30 |

以上为单次平面运动学仿真记录，误差是该演示的结果，不是产品精度指标或实机统计。
日志、地图、JSON 结果及补丁保存在服务器本次独立研发目录。
本地无 ROS 的检查单独显示消息/模拟器测试跳过；上列 20 项来自实际 ROS 容器，未计跳过项。
