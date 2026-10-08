# 四足运动控制与 RS04 驱动开发预览

2026-10-08 更新：默认运行链路改为 [通用关节目标角度接口](JOINT_POSITION_INTERFACE.zh-CN.md)，上位机输出角度，仿真关节执行闭环。板卡型号未绑定，实际角度通信适配器待接入。本文的主机 PD / SocketCAN 扭矩路径保留为显式 `--control-mode host-effort-test` 对照测试；下表的 57 项测试、4 个动力学场景、15 项 ROS 2 检查属于此前基线，新增证据见 [通用接口验收记录](validation/joint-position-20261008.md)。

本增量接续 ROS 2 关节预览，加入使用真实姿态/关节反馈的 MEVIUS2 策略控制、浮动基座接触动力学、ROS 2 指令接口和 ROBSTRIDE04 SocketCAN 驱动代码。代码与仿真可供开发验收；尚未执行真实电机或整机验收，尚未包含在公开的 `v0.2.0-preview.2` 下载包中。

## 已实现与适用范围

| 部分 | 当前实现与验证 |
| --- | --- |
| 运动控制 | 200 Hz 控制循环、50 Hz 策略推理；显式使能、3 秒起身、零速策略站立、前行/横移/转向 |
| 姿态反馈 | 机身坐标角速度、四元数投影重力、12 关节位置/速度；不使用预设关节动画代替闭环 |
| 动力学 | 固定版本 MEVIUS2 原版 MuJoCo 模型，含浮动机身、重力和接触；四个数值场景通过 |
| ROS 2 | Humble 真实 DDS → 控制进程 → 动力学 → JointState/Imu/status；15 项端到端检查通过 |
| RS04 | 私有 29 位 CAN 报文、总线/电机/主机身份校验、位置/速度/扭矩/温度反馈、故障锁存、固件超时参数写入与回读；模拟总线测试通过 |
| 客户配置 | RS04 标定 JSON 模板，无需改源码；加载内容的 SHA-256 随运行状态返回。模板中的未知值阻止开启硬件接口 |

用户截图显示 12 个 RS04 单机及 1 套 RS04 + CAN 转 USB 模块，另有两项 EDULITE 05。记录为选型目标，不作为已购买/已安装/已标定证据。四足控制目标为 12 个 RS04；剩余电机和 EDULITE 未分配用途。

**上游原版仍使用 RS03。** 当前动力学保留原版质量、惯量、关节范围和 60 N·m 执行器限幅，不能把该结果写成客户 RS04 整机仿真通过。更换电机、减速机构、载荷或腿部结构后，须更新实际模型并验证策略适配。该策略不承诺直接控制任意导入的 URDF。

## 文件与配置

- `scripts/quadruped_controller.py`：无 I/O 的传感器校验、策略输入/输出及控制状态机。
- `scripts/robstride04_driver.py`：RS04 编解码、Linux SocketCAN、标定和电机会话。
- `scripts/joint_interface.py`：独立于厂家与关节数量的角度指令、反馈和接收校验。
- `scripts/quadruped_backends.py`：默认关节角度闭环仿真，以及原 MuJoCo / RS04 扭矩后端。
- `scripts/quadruped_worker.py`：控制进程，默认仅仿真；监听 `127.0.0.1`，独立随机令牌鉴权。
- `scripts/quadruped_ros2.py`：ROS 2 接口；启动保持 DISARMED。
- `config/robot_profiles/robstride04.calibration-template.json`：本机标定模板。

标定公式为 `q_joint = direction * (p_actuator - offset_rad) / external_ratio`，`tau_actuator = direction * tau_joint / external_ratio`。RS04 协议提供集成执行器输出轴单位；`external_ratio` 只填写输出轴之后的外部传动比，不可再次计入电机内部减速比。零点、方向、关节限位、速度、扭矩及温度门槛必须来自本机记录；不复制 MEVIUS2 作者机器的零点。

模板保留建议 CAN 分组和名称对应，实际地址须核对。`robot_serial`、`calibration_record`、每关节标定、IMU 安装标定，以及针对该整机的策略验证记录均为实机入口条件。`policy_validation.approved` 是使用者的验收声明，不是程序自动证明；状态中的 `hardware_validated` 保持 false。

## ROS 2 接口与状态

所有名称均在 `/omindos_quadruped` 下：

| 接口 | 类型与作用 |
| --- | --- |
| `cmd_vel` | `geometry_msgs/TwistStamped`，`frame_id=base_link`；使用 linear.x、linear.y、angular.z |
| `joint_states` | `sensor_msgs/JointState`，测量关节位置和速度，按名称映射 |
| `imu` | `sensor_msgs/Imu`；仿真时发布，实机时订阅已经转换到 base_link 的机身姿态/角速度 |
| `status` | `std_msgs/String` JSON，含状态、原因、后端、真实硬件激活标记、拒绝输入计数和策略推理次数 |
| `arm`, `stand`, `walk`, `stop`, `disarm`, `estop` | `std_srvs/Trigger` 服务，显式控制生命周期 |

流程是 `DISARMED → arm → STANDBY → stand → STANDUP → HOLD → walk → LOCOMOTION`。

- `stop`：进入 HOLD，速度指令归零，继续使用姿态反馈和策略保持站立。
- 速度输入超过 0.3 秒未更新：进入 HOLD；新消息不会自动恢复行走，须再次 `walk`。
- `estop` 或传感器/控制故障：FAULT 锁存，停止输出扭矩；不能调用 arm 自动恢复，排查后重启控制进程。
- `disarm`：停机卸力。卸力不等于站稳或受控下蹲，机器人可能倒下。
- 接受的开发预览命令范围：`|vx|≤0.3 m/s`、`|vy|≤0.2 m/s`、`|wz|≤0.5 rad/s`；这是软件输入门槛，非实机性能承诺。

ROS 输入校验来源时间戳、递增顺序、base_link 坐标及有限数值；进入控制进程时保留原始数据年龄。IMU 或关节反馈过期、异常四元数、过大倾角、策略输出异常和控制循环超过期限均触发锁存。导航接入方应发布上述带时间戳接口，不能只把原 `/cmd_vel` 的 Twist 类型重映射成 TwistStamped。

## 驱动行为

驱动仅实现 RS04 的私有 29 位扩展帧协议；不是 CANopen 或 11 位 MIT 协议。Linux 接口须已配置为 1 Mbit/s。套装 USB 模块是否暴露 SocketCAN 尚未确认，不能将任意串口设备直接当作 can0。

协议量程：速度 ±15 rad/s、扭矩 ±120 N·m、Kp 0–5000、Kd 0–100；位置按厂家示例 C 常量 ±12.57 rad 编码，手册表格表示为 ±4π。这些是编码范围，不是推荐工作负荷。

显式 arm 时先停止所有已配置电机并收齐反馈；设置并回读 `0x7028 canTimeout=2000`（手册规定 20000 对应 1 秒，即本配置 100 ms），再设置并回读运行模式 0。使能前重新核验位置、IMU 和截止时间，逐个使能后立即发送零增益/零目标扭矩报文。该协议量化的零扭矩存在小于 0.004 N·m 的误差。

在显式 `host-effort-test` 对照路径中，主机按实际关节反馈计算 PD 扭矩、逐关节限幅，再通过类型 1 的前馈扭矩字段发送，电机 Kp/Kd 固定为 0，避免只限制前馈而放过电机内部 PD 输出。每周期发送前完整校验整批命令；反馈/发送超过 50 ms、编码位置/速度触及饱和端点、故障位、温度、机械限位或总线错误会停止所有电机并锁存。操作系统接收时间戳用于识别积压旧帧。进程崩溃时依赖已回读的电机端通信超时，而非 Python 退出回调。此路径不作为通用角度适配器使用。

本实现是 Python 开发预览，尚无真实 CAN 延迟、固件兼容性、长期温升或硬实时保证。没有修改电机 ID、机械零点、持久化参数或自动清除故障的接口。对未连接或已经失效的总线，软件发出停止报文不能证明机械装置已经停下。

## 复现仿真

固定上游仓库 `haraduka/mevius2@4f09680bb575574377b903bbc971bdb2695d507b`，启动前检查受版本管理文件清洁及策略 SHA-256。不要执行上游默认的真实电机主程序。

已验证依赖：Python 3.12、PyTorch `2.14.1+cpu`、MuJoCo `3.14.0`、NumPy `2.5.3`；ROS 2 Humble 使用其 Python 3.10 环境。两者通过带鉴权的本机接口通信，避免将 Humble 的 rclpy 加载到不兼容的 Python 版本。

在安装好对应依赖的策略环境中执行：

```bash
python scripts/validate_quadruped_dynamics.py \
  --source /path/to/pinned-mevius2 \
  --output /tmp/quadruped-dynamics.json
python scripts/quadruped_worker.py \
  --source /path/to/pinned-mevius2 \
  --token-file /tmp/omindos-quadruped.token
```

在同一主机的 ROS 2 Humble 环境中执行（隔离测试建议设置独立 ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY=1）：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/quadruped_ros2.py --token-file /tmp/omindos-quadruped.token
```

另一个同 ROS 域终端运行自动验收脚本。该脚本会执行仿真使能、起身、行走、停止和急停，并先检查后端必须为 mujoco：

```bash
python3 scripts/check_quadruped_ros2.py --output /tmp/quadruped-ros2.json
python3 -m unittest discover -s tests -v
```

旧扭矩台架入口另须指定 `--control-mode host-effort-test --backend robstride04 --allow-hardware --calibration <本机标定文件>`。随附空模板会在打开 CAN 之前失败。默认角度模式尚不提供真实板卡适配器。本次没有运行实机入口，没有连接任何用户电机，也没有远程发出真实使能命令。

## 验证与来源

结果见 [验收记录](validation/quadruped-rs04-20261008.md)、[动力学数据](validation/quadruped-dynamics-20261008.json)、[ROS 2 数据](validation/quadruped-ros2-20261008.json)。前行仍存在约 0.10 m 侧偏及约 0.20 rad 航向漂移；没有宣称通过速度跟踪、精确导航、复杂地形或广泛扰动恢复验收。

MEVIUS2 观测布局与部署常量保留 [MIT 许可](../LICENSES/MEVIUS2-MIT.txt)。模型和权重来自上述固定外部检出，本增量不复制分发它们，也不复制厂家手册全文。

RS04 报文依据 [厂家手册 260713](https://github.com/RobStride/Product_Information/blob/3f0cae4986e175337c7b03d891b899247b1f2933/Product%20Literature/RS04/RS04User%20Manual260713.pdf) 的 §3.3.6、§4.1、§4.4。PDF SHA-256 `6782bf105cf4cc945142e65356bd7a21a9fc007b008ec3616307956030d72b31`，Git blob `bc92a2ef968b7940fd924326106e997c3f00e12a`；已核对协议表格与示例常量，驱动为独立实现。
