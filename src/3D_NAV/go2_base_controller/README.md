## 项目概述

这是一个用于Go2四足机器人的ROS 1基础控制器模块，位于ROS工作空间中。项目提供了机器人的速度控制、急停监控和遥控器接口功能。

## 核心架构

### 主要组件

1. **Go2BaseController** (`loco/go2_base_controller.py`)
   - ROS 1节点，订阅`/cmd_vel`话题接收运动指令
   - 使用`unitree_sdk2py`与Go2机器人通信
   - 50Hz控制循环，处理运动指令并发送给机器人
   - 支持急停功能，订阅`/emergency_stop`话题

2. **EmergencyStopMonitor** (`loco/emergency_stop_monitor.py`)
   - 监控Go2遥控器的急停按键（B键）
   - 订阅机器人低级状态话题`rt/lf/lowstate`
   - 当检测到急停按键时发布`/emergency_stop`信号

3. **RemoteController** (`loco/remote_controller.py`)
   - 解析Go2遥控器数据的工具类
   - 提供按键映射和摇杆数据解析
   - 支持KeyMap定义的16个按键

4. **SDK Probe** (`loco/sdk_probe.py`)
   - 只读检测Unitree SDK/DDS连通性
   - 调用`robot_state.ServiceList()`并订阅`rt/lf/lowstate`
   - 不发送运动指令，适合实机动测前排障

### 依赖关系

- **ROS 1**: 使用rospy，geometry_msgs，std_msgs
- **Unitree SDK 2**: `unitree_sdk2py`用于与Go2机器人通信
- **Python 3**: 所有脚本都使用Python 3

## 常用命令

### 启动和停止系统

使用控制脚本：
```bash
# 启动完整的控制系统（急停监控器 + 控制器）
# 启动前会自动检查 GO2_PEER 的主机路由是否走 GO2_IFACE，不对就修正
# 如果发现旧实例还在，也会先按 PID 文件清理，避免重复启动多个底盘桥
./loco/go2_base_control.sh start

# 停止控制系统
./loco/go2_base_control.sh stop

# 重启控制系统
./loco/go2_base_control.sh restart
```

启动脚本可通过环境变量指定DDS网络参数：
```bash
GO2_IFACE=enp86s0 GO2_PEER=192.168.123.161 ./loco/go2_base_control.sh start
```

如果主机有多个网卡，脚本会为 `GO2_PEER` 自动补一条主机路由，确保与 Go2 的 DDS/SDK 通信走 `GO2_IFACE`，避免误走 Wi-Fi 导致 `Move()` 调用超时。

推荐的实机联调顺序：
```bash
# 1. 先做只读 DDS/SDK 连通性探测
python3 loco/sdk_probe.py --iface enp86s0 --peer 192.168.123.161

# 2. 再启动底盘桥
GO2_IFACE=enp86s0 GO2_PEER=192.168.123.161 ./loco/go2_base_control.sh start

# 3. 最后做一次性 /cmd_vel 小步测试
python3 loco/test_cmd_vel_forward.py --distance 0.3 --speed 0.1
```

### 单独运行组件

```bash
# 仅运行基础控制器（自动使用默认网络接口）
python3 loco/go2_base_controller.py

# 或指定特定网络接口
python3 loco/go2_base_controller.py --iface eth0

# 指定静态DDS peer
python3 loco/go2_base_controller.py --iface enp86s0 --peer 192.168.123.161

# 仅运行急停监控器（自动使用默认网络接口）
python3 loco/emergency_stop_monitor.py

# 或指定特定网络接口
python3 loco/emergency_stop_monitor.py --iface eth0

# 测试遥控器连接
python3 loco/test_remote.py --iface enp86s0 --peer 192.168.123.161

# 只读SDK/DDS连通性探测
python3 loco/sdk_probe.py --iface enp86s0 --peer 192.168.123.161

# 一次性前进1米，测试/cmd_vel链路
python3 loco/test_cmd_vel_forward.py --distance 1.0 --speed 0.2
```

### ROS话题

- **订阅话题**:
  - `/cmd_vel` (geometry_msgs/Twist): 机器人运动指令
  - `/emergency_stop` (std_msgs/Bool): 急停信号
  - `rt/lf/lowstate` (unitree_go.msg.dds.LowState_): 机器人状态

- **发布话题**:
  - `/emergency_stop` (std_msgs/Bool): 由急停监控器发布

## 开发注意事项

### 系统要求

- 确保已安装`unitree_sdk2py`包
- 需要ROS 1环境（rospy可用）
- 机器人必须在同一网络中且SDK通道可访问

### 控制流程

1. 急停监控器首先启动，建立与机器人的状态连接
2. 等待2秒后启动基础控制器
3. 控制器以50Hz频率处理运动指令
4. 任何急停信号都会立即停止所有运动并关闭节点

### 安全机制

- 指令超时保护：1秒内未收到新指令则停止运动
- 急停按键监控：遥控器B键触发急停
- 双重急停：监控器和控制器都能响应急停信号
- 性能监控：跟踪Move()调用延迟，超过100ms时发出警告

### 调试

- 控制器每50个控制循环（1秒）打印一次状态信息
- 急停监控器每10次状态更新打印一次遥控器数据
- 所有组件都有详细的错误日志记录

### 常见问题排查

1. `test_cmd_vel_forward.py` 有日志，但机器人不动
   - 先跑 `sdk_probe.py`，确认 `ServiceList code: 0`，并且能收到 `rt/lf/lowstate`
   - 再看 `go2_base_controller` 日志里是否出现 `SLOW Move()`；如果 `Move()` 接近 10s 超时，通常是 DDS/SDK 没有真正连上机器人

2. 主机同时连了 Wi-Fi 和机器狗网口
   - 优先用 `./loco/go2_base_control.sh start` 启动，不要手工常驻多个 `python3 go2_base_controller.py`
   - 启动脚本会自动修正 `GO2_PEER` 的主机路由，避免到机器狗的请求误走其他网卡

3. `/cmd_vel` 有订阅者，但底盘仍不响应
   - 检查是否重复启动了多个 `go2_base_controller` 实例
   - 检查机器人是否已站立、处于可运动状态，并确认没有被遥控器急停或安全状态锁住
