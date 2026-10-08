# 通用关节目标角度接口（开发预览）

默认链路为：上位机策略生成关节目标角度，经通信适配层送到关节驱动器，关节驱动器完成闭环。关节位置、速度、故障和机身 IMU 返回上位机。达妙卡的具体角色由其型号和协议决定；本接口不假定达妙卡计算力矩，也不假定它已兼容 RS04。

本次提供可运行的通用接口、接收校验和 MuJoCo 关节闭环仿真。**实际板卡通信适配器尚未接入，真实 RS04 驱动、客户整机步态和平衡仍待验收。** 原版 MEVIUS2 模型和策略的适配范围不因接口通用化而扩大。

## 控制数据

`scripts/joint_interface.py` 不依赖电机厂家、CAN、ROS 或固定关节数量。通过名称对应关节，收到乱序列表时重新排列；拒绝缺失、重复或未知关节。数据进入执行层前整批检查，通过后才输出。

| 字段 | 定义 |
| --- | --- |
| `schema` | `omindos.joint_position.v1` |
| `unit` | 固定 `rad`，关节侧角度；界面显示度数时由界面转换 |
| `joint_names` | 本机配置的完整关节名称列表 |
| `position` | 与名称一一对应的目标角度；有限数值且在配置范围内 |
| `session_id` | 每次显式使能产生的新会话，禁止复用旧会话 |
| `sequence` | 会话内递增整数；拒绝重复或倒序 |
| `issued_monotonic_s` | 接收执行端时钟域内的命令生成时间 |
| `valid_for_s` | 有效期，当前默认接口最大 0.05 秒 |

这是 OmindOS 内部契约，**不是任何厂家的串口或 CAN 报文**。示例对象可通过 `PositionCommand.to_dict()` 序列化，`from_dict()` 恢复。跨计算机传输时，适配器须处理时钟差和网络年龄，不能直接比较不同机器的 monotonic 时间，也不能给旧指令重新盖一个新时间戳。

`JointFeedback` 包含关节名称、实测位置（rad）、速度（rad/s）、逐关节故障码和采样时间。反馈同样按名称校验，拒绝非有限值、过期数据和故障状态。IMU 继续使用 `base_link` 坐标。通用契约支持不同数量的旋转关节；MEVIUS2 策略本身仍固定为 12 个关节，不能直接作为双臂或任意机器人策略。

## 生命周期与执行层

启动无运动输出。`arm` 明确使能后从当前关节位置开始；`stand` 起身；`walk` 接受底盘速度指令。`stop` 继续零速平衡策略；`disarm` 和 `estop` 停止输出。没有命令不等于命令所有关节去零点。

过期指令、错误会话、重复序号、非法角度和接收端看门狗到期均停止执行并锁存故障。新角度不能自动恢复失效会话。真正的硬件接收端还必须独立实现通信超时、速度/力矩限值、使能、停止和反馈处理；主机上的 Python 检查不能替代这些硬件行为。

默认 `quadruped_worker.py --control-mode joint-position` 使用 `SimulatedPositionBackend`。策略只调用 `Controller.tick_targets()` 生成角度。仿真适配器以 200 Hz 模拟关节内部 PD 和执行器限幅，策略以 50 Hz 更新目标；起身插值和目标下发为 200 Hz。原版参考增益为 Kp=50、Kd=2，限幅 60 N·m。这是数学模型，不表示 RS04 的 CSP、PP 或运控固件已验证等效。

仿真参考关节使用与既有传感器检查一致的 0.08 rad 软限位容差，供折叠初始姿态捕获；真实适配器须采用本机实际机械限位。增益、传动比、零点、方向、地址及限制参数应来自客户配置，不要求客户修改策略源代码。新增未支持的硬件协议仍需开发者实现适配器。

原 `Controller.tick()` 和 RS04 SocketCAN 扭矩路径保留为 `--control-mode host-effort-test` 对照测试。它们不属于默认角度链路。默认角度模式请求真实 RS04 或 RS04 扭矩报文仿真时，会在加载策略或打开设备前报告适配器未实现，不会静默退回主机扭矩控制。

## ROS 2 与运行

原 `cmd_vel`、`joint_states`、`imu`、状态和动作服务保持原接口。新增 `/omindos_quadruped/joint_targets`（`std_msgs/String`）发布上述完整 JSON 的**只读诊断副本**，约 50 Hz，不是硬件控制传输端口。保留生成时间、序号、会话，不重复发布同一包，不重置有效期；停机后停止发布。状态包含 `control_mode`、`joint_interface`、`joint_adapter_active`、`joint_adapter_fault` 和当前 `joint_command`。

```bash
python scripts/quadruped_worker.py \
  --source /path/to/pinned-mevius2 \
  --token-file /tmp/omindos-quadruped.token
python3 scripts/quadruped_ros2.py --token-file /tmp/omindos-quadruped.token
```

策略与 ROS 2 分别使用已配置的 Python 环境，见 [四足运行说明](QUADRUPED_RS04_RUNTIME.zh-CN.md)。重现默认路径：

```bash
python -m unittest discover -s tests -v
python scripts/validate_quadruped_dynamics.py \
  --source /path/to/pinned-mevius2 \
  --output /tmp/joint-position-dynamics.json
python3 scripts/check_quadruped_ros2.py --output /tmp/joint-position-ros2.json
```

`check_rs04_driver_dynamics.py` 则专门验证旧的主机扭矩/RS04 报文路径及质量敏感性；其结果与通用角度链路分别记录。质量敏感性采用假定的新增质量分布和惯量缩放，不等于完成客户 CAD 建模。`rs04-simulation-control.json` 的 Kd=3 候选只用于仿真，不作为实机整定参数。

`check_rs04_hardware.py` 默认只列出主机 CAN/USB 串口设备，不打开接口、不发送报文。显式只读参数探测属于台架诊断，也不是通用板卡适配器。
