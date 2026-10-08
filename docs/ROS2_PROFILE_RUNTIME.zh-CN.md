# 参数工作台 → ROS 2 关节预览

本增量把已保存的配置编号加载到实际 ROS 2 节点，三维工作台可启动、停止关节轨迹，并显示节点回传的姿态。客户无需修改源代码。它使用 PR #2 的三维工作台及实体渲染界面。

这里的“已加载”明确指 `runtime_target=ros2_joint_preview`。当前是固定基座运动学关节预览，尚不是四足平衡/步态控制器，也没有接入电机驱动或完成实机验收。`runtime_applied=true` 不等于 `hardware_active=true`；后者始终为 false。

## 启动

需要 Python 3.10+ 和 ROS 2 Humble 的 `rclpy`、`std_msgs`、`sensor_msgs`。在同一台机器的两个终端中，使用同一个绝对配置目录。

终端一：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=218
export ROS_LOCALHOST_ONLY=1
python3 scripts/robot_profile_runtime.py \
  --store "$HOME/omindos-robot-configurations" --port 8086
```

终端二：

```bash
python3 scripts/robot_workbench.py \
  --store "$HOME/omindos-robot-configurations" --port 8085 \
  --runtime-url http://127.0.0.1:8086
```

打开 `http://127.0.0.1:8085/`，导入完整配置，调整参数，点击“保存并加载到 ROS 2”，随后点击“ROS 2 试运行”。关节目标与运行时长来自工作台；试运行从 ROS 节点当前姿态开始。“停止 ROS 2 预览”使关节预览保持当前位置。单独的“试一下 · 运动学”仍可使用，不依赖 ROS 2。

输入参数不同于已加载版本时，先保存并重新加载。旧版本一直保留，读取旧编号后也可显式加载。尚未填写完整电机参数的 URDF 可以查看，但不能提交为有效配置。关闭页面后的会话心跳超时会停止预览；进程重启始终回到未加载状态。

## 接口和实际生效范围

工作台同源代理与节点本机服务共用以下路径。节点仅监听 `127.0.0.1`，Host/Origin 校验限制本机同源请求，不是公网服务。

| 路径 | 方法 | 请求/作用 |
| --- | --- | --- |
| `/v1/runtime/status` | GET | 当前配置编号、状态、模型、关节姿态、预览作用域；只读，不续租 |
| `/v1/runtime/load` | POST | `profile_id`；从共享存储读取并校验摘要，返回本次会话 `lease_token` |
| `/v1/runtime/run` | POST | `profile_id`、`lease_token`、完整关节名到角度映射 `target`、`duration_s` |
| `/v1/runtime/stop` | POST | `lease_token`；停止当前预览 |
| `/v1/runtime/heartbeat` | POST | `lease_token`；续租并返回当前状态，页面每 0.5 秒调用 |

节点串行处理配置和控制请求。运行中拒绝加载配置；非法版本、文件被篡改、未知关节、超出关节限位或不满足峰值速度要求的轨迹均被拒绝。过期且尚未执行的请求不会在恢复后补执行。发生请求超时时，应查询状态再重试。会话切换后，旧令牌不能控制新配置。

心跳超过 2 秒未收到时保持最后一个已计算姿态，不自动继续；这是一项关节预览行为，不等于物理急停或实体制动。更换减速比和零点会改变节点回传的电机侧角度估算，没有向真实电机下发。

| ROS 2 话题 | 类型 | 语义 |
| --- | --- | --- |
| `/omindos_workbench/robot_description` | `std_msgs/String` | 已加载版本的 URDF，可靠且 transient-local |
| `/omindos_workbench/joint_states` | `sensor_msgs/JointState` | 30 Hz 计算关节位置/速度；effort 是静态保持力矩估算，不是传感器回读 |
| `/omindos_workbench/runtime_status` | `std_msgs/String` | 10 Hz JSON 状态，含作用域和硬件未启用标志 |

状态中的 `motor_positions_rad` 按已保存的方向、减速比、零点计算，只用于检查坐标转换。四足机身移动导航和关节预览目前没有闭环连接，不能据此声称四足会站稳或行走。没有发布 `/cmd_vel`、硬件关节轨迹或 CAN 命令。

## 下一步实机接入所需信息

接入选定机器后，驱动层必须明确型号/协议、SDK 版本、反馈是在电机侧还是关节侧、零点标定、控制模式和设备失联后的停机语义，再建立真实状态回读与使能流程。MEVIUS2 参考值不能直接替代客户机器的标定。完成驱动适配与实体停止验收前，界面“真机执行”保持不可用。

## 验证

```bash
python3 -m unittest discover -s tests -v
source /opt/ros/humble/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 python3 scripts/check_robot_profile_ros.py
```

真实 ROS 验证使用教学模型和隔离 ROS 域，不连接硬件。浏览器联调脚本为 `scripts/check_robot_runtime_browser.cjs`，需测试机安装 Puppeteer/Chromium，指向测试用工作台。运行环境不依赖这些开发工具。

完整记录见 [本次验收记录](validation/ros-profile-runtime-20261008.md)。现有已发布 `v0.2.0-preview.2` 包保持原样；本增量在开发 PR 中，尚未重新打包发行。
