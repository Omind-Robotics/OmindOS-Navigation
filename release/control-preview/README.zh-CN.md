# OmindOS 控制预览 0.2.0-preview.5

此包面向 Linux x86_64 的参数配置、ROS 2 关节预览与原版 MEVIUS2 闭环仿真。
需安装 Docker Engine；Python、ROS 2 Humble 和仿真依赖已打包，无需自行安装。
独立 Windows/Linux 桌面工作台另行发行，默认使用不要求 ROS。
本包不是 Jetson ARM64 安装包，也不是已验收的实机控制固件。

## 启动

解压后进入目录：

```sh
./omindos workbench
```

浏览器访问 http://127.0.0.1:8085 。按“URDF → 电机 → 其他参数”配置并保存，旧版本保留。
工作台默认不启动 ROS 控制端，不会向电机发指令。首次启动自动加载随包镜像。
数据保存在包内 data/，更新时保留此目录。

可选启动真实 ROS 2 节点驱动的关节轨迹预览：

```sh
./omindos ros-preview
```

先在工作台保存参数，再执行“加载到 ROS 2”并运行预览。软件急停锁存；复位不自动续跑。
此面板是关节运动学轨迹预览，未与下述四足策略进程连接。

## 四足闭环仿真

```sh
./omindos validate-dynamics
```

运行前行、横移、转向与站立受扰四项原版 MEVIUS2 仿真，结果写入 data/dynamics.json。

分别在两个终端启动仿真控制进程和 ROS 桥：

```sh
./omindos quadruped
./omindos quadruped-ros
```

启动时为 DISARMED，必须通过 ROS 服务显式 arm、stand、walk。
服务前缀为 /omindos_quadruped/，类型 std_srvs/srv/Trigger；
速度话题为 /omindos_quadruped/cmd_vel，类型 geometry_msgs/msg/TwistStamped，
frame_id 为 base_link，时间戳需为当前 ROS 时间。速度断流会转入站立保持，恢复需再次显式 walk。
stop 进入零速保持；estop 锁存故障并停止目标流，不能用 arm 清除。
默认 ROS_DOMAIN_ID=42，ROS_LOCALHOST_ONLY=1；所有 HTTP 服务仅监听本机。

## 范围与验收边界

- 参数提交、校验、保存、按编号读取及历史版本保留。
- 工作台三维运动学预览、可选 ROS 2 参数加载与关节轨迹。
- 通用关节角度接口与原版 MEVIUS2 MuJoCo 闭环仿真。策略 50 Hz，仿真角度下发 200 Hz；不代表实机频率。
- 客户方案记录：Jetson 经 USB 连接 DM-MC02，三路 CAN 暂按每路四关节规划。
- DM-MC02 通信固件、RS04 角度协议适配、Jetson ARM64 发行、实机驱动、客户整机步态和平衡仍待验收。
- 未连接、使能或验证真实电机。软件急停验证不等于物理急停验收。

后端业务程序以编译模块交付，不附带业务 Python/C 源码。网页资源、模型、参数与第三方运行依赖保留。
原版 MEVIUS2 模型与策略按 MIT 许可提供，固定提交 4f09680bb575574377b903bbc971bdb2695d507b。
配置和模型文件保留在镜像 /opt/omindos/config 与 /opt/omindos/reference；许可证在 /opt/omindos/LICENSES。
镜像内第三方软件按各自许可分发，相关说明见 THIRD_PARTY.md。
