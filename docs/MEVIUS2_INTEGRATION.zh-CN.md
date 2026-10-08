# MEVIUS2 四足运动控制适配记录

日期：2026-10-08。客户确认机器人参考 MEVIUS2 设计，尚无已运行的站立/遥控行走程序。本次客户版范围包含四足站立、平衡、步态和电机接入；继续与差速、全向轮式共用导航核心和安装入口。现按用户决定沿用上游默认模型/控制参数推进，不等待额外硬件数据；客户有差异时按照 [配置指南](CUSTOMER_ROBOT_CONFIGURATION.zh-CN.md) 更新本机配置。

## 固定参考与已有能力

- 上游：[haraduka/mevius2](https://github.com/haraduka/mevius2)，固定提交 `4f09680bb575574377b903bbc971bdb2695d507b`。
- 项目说明：[MEVIUS2 hardware](https://haraduka.github.io/mevius2-hardware/)。本文已核对仓库和项目页；未将论文的全部实验结论作为客户实测结果。
- `scripts/mevius2_main.py` 已包含待机、起身和策略行走流程，使用 ROS 1 `rospy`；不能直接当作 ROS 2 Humble 节点。
- `models/policy.pt` 是已有 TorchScript 策略；部署输入 34 维，输出 12 个关节动作。提供 URDF、MuJoCo 模型和训练仓库入口。
- `scripts/parameters.py` 使用 12 个 RobStride03 电机、两路 CAN，仿真/电机环 200 Hz、策略 50 Hz。这些是**原设计参数**，不是客户硬件已确认信息。
- 已读取电机通信实现 `scripts/xiaomimotor_lib.py`。原版主程序不加 `--sim` 会初始化真实电机；本文复现只调用独立无硬件验证脚本，不执行该默认路径。

部署关节顺序为 BL、BR、FL、FR，每腿 collar、hip、knee。迁移须按关节名映射；训练顺序、URDF 顺序和电机 ID 不能按数组位置混用。原机 `MOTOR_OFFSET_ANGLE` 是个体校准结果，不可复制成客户电机零点。

## 已执行的原版仿真基线

验证脚本：[validate_mevius2_reference.py](../scripts/validate_mevius2_reference.py)。完整数值、依赖版本与轨迹：[JSON 记录](validation/mevius2-reference-20261008.json)。

脚本校验上游提交、受版本管理文件的清洁状态及策略 Git blob；以 AST 读取常量，按名称映射关节，复现上游观测缩放、策略动作和 PD 控制。加载原版模型、权重，不加载 ROS、CAN 或电机 SDK。这是同步无界面验证程序，不等同于原版多线程 ROS 节点或 OmindOS ROS 2 集成验证。

策略 SHA-256：`392b4d3f30e856caedcb0ec25f125201e97b28795bc9342ee2fc1d8125656510`。

| 阶段 | 仿真时间 | 本次观测 |
| --- | --- | --- |
| 待机 | 1 秒 | 初始折叠姿态稳定 |
| 起身 | 3 秒 | 机身高度增加约 0.335 m |
| 零速策略站立 | 3 秒 | 保持直立，存在约 0.131 m 前向位移 |
| 前向命令 0.2 m/s | 8 秒 | x 位移 1.176 m，y 偏移 −0.127 m |
| 命令归零 | 5 秒 | 最后 1 秒平均平面速度约 0.00181 m/s |

合计 20 秒仿真、800 次策略推理；六项基础检查通过：状态有限值、策略输入输出尺寸、策略阶段直立、机身离地、前向移动、归零后低速。检查阈值在脚本中显式定义。

这只证明原版模型/策略在该平地场景可以执行基本运动。站立阶段仍有漂移，行走实际速度与命令存在差异；未建立或通过客户速度跟踪和定位精度门槛。转向、横移、扰动、地形、长时间热负荷、驱动断连、通信实时性均未验证。记录中的 `hardware_tested`、`ros2_integration_tested`、`customer_robot_model_tested` 全为 `false`。

复现时在隔离环境中安装与 JSON 记录相符的 NumPy、PyTorch 和 MuJoCo，使用固定上游检出：

```bash
git clone https://github.com/haraduka/mevius2.git /tmp/mevius2-reference
git -C /tmp/mevius2-reference checkout 4f09680bb575574377b903bbc971bdb2695d507b
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 scripts/validate_mevius2_reference.py \
  --source /tmp/mevius2-reference --output /tmp/mevius2-reference-validation.json
```

上述命令只运行仿真。此记录不证明当前客户 Docker 包已包含这些依赖、模型或策略。

## 接入任务与待确认信息

1. 以客户 CAD/URDF、电机型号、传动比、腿长、整机质量及载荷更新模型；核对质量中心/惯量、关节正方向/限位、CAN 分组和 IMU 坐标。先评估已有策略，再决定是否重训，不能仅凭外形相似承诺复用权重。
2. 将观测、策略、关节目标与硬件通信拆分，建立 ROS 2 速度请求及状态接口。四足接收 `vx/vy/wz`；差速禁止 `vy`，全向按现有平台配置处理。
3. 增加显式使能、状态机、速度指令时效、IMU/电机反馈时效、故障锁存和复位。原版急停/异常路径不能直接当作客户验收通过；四足“零速保持站立”和“电机卸力”必须分别定义和测试。
4. 将实际机器人模型及 ROS 2 适配器加入仿真，完成三平台回归、速度跟踪、取消任务、断流/异常输入和重启状态测试，再构建客户候选包。
5. 硬件就绪后完成独立零点校准和受控实机验收，记录机器序列、标定、固件和测试环境，再冻结正式客户发行范围。

当前客户配置仍保留未知值，避免将原设计的 CAN ID、零点、主机架构或接口误标为客户实测参数。现有公开发行仍为导航预览版；本分支尚未提供正式实机运动控制发行。

## 来源与许可记录

本验证脚本的观测布局和控制参数源于上述固定提交，保留上游作者归属及 [MIT 全文](../LICENSES/MEVIUS2-MIT.txt)。当前增量未复制上游模型、权重、CAN 驱动或 IsaacGym 工具文件。

上游根目录 `LICENSE` 为 MIT，`package.xml` 标注 BSD；README 另列 IsaacGym / legged_gym 来源。将这些文件或依赖纳入可分发客户包前，需逐项记录实际来源与通知文件，不能以根目录标签代替全部依赖的许可清单。
