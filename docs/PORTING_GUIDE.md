# 实机与 3D 前端迁移指南

当前分支先完成无实机的原生 ROS 2 算法闭环。接入新传感器或底盘时，应保留标准接口边界，
不要把厂商 SDK、ROS 1 消息或 bridge 调用写回 `nav3d_native` 算法层。

## 接入激光/LIO

二维扫描有两种接入方式：

1. 激光直接发布 `sensor_msgs/msg/LaserScan`；
2. 3D 雷达发布 `PointCloud2`，由独立前端按高度、距离和地面规则投影为 `LaserScan`。

建图需要一个与扫描时间对齐的全局位姿话题：

```bash
ros2 run nav3d_native mapper --ros-args   -p pose_topic:=/lio/odom   -r /scan:=/projected_scan   -p map_output:=/data/maps/site_a
```

定位阶段则需要局部连续 `/wheel/odom`、`/scan` 和已生成的 `/map`。若采用外部 LIO/SLAM
直接定位，可替换 `nav3d_localizer`，但必须继续发布 `map -> odom` 或等价的
`/localization/pose`，使规划控制层无需感知传感器品牌。

检查项：

- 点云/扫描时间戳使用同一时钟；
- `base_link -> base_scan` 外参正确；
- 地图、里程计和基座形成一棵 TF 树；
- 静止时定位不持续漂移；
- 移动时没有明显跳变或时间反转。

## 接入机器人底盘

创建独立 ROS 2 包实现：

```text
/cmd_vel
  -> 限速 + watchdog + enable/estop
  -> 厂商 SDK / CAN / 串口 / ros2_control

底盘状态
  -> /wheel/odom
  -> odom -> base_link
```

建议验证顺序：

1. 只读检查 SDK、网络、关节/底盘状态；
2. 在导航栈关闭时发送零速度；
3. 发送受限的短时小速度并验证急停；
4. 验证 watchdog 在命令中断后停车；
5. 接入 `/cmd_vel`，先悬空或支架测试；
6. 在空旷区域以低速运行完整导航。

## 从二维回归扩展到 3D 地形导航

保留当前 Docker 回归作为基础门禁，再以独立包增加：

- `PointCloud2` 去畸变、地面分割和高度/坡度估计；
- 2.5D elevation/cost map 或体素地图；
- 机器人足迹、净空、坡度和台阶约束；
- 3D/SE(2.5) 全局搜索与局部避障；
- rosbag2 数据集和固定指标回放。

不要删除现有二维确定性测试。新增 3D 算法应同时提供无硬件数据集测试，并保持最终输出仍为标准
`nav_msgs/Path` 与 `/cmd_vel`，或通过明确的新接口版本升级。

## 迁移完成标准

```text
[ ] Docker 中 colcon 构建成功
[ ] 算法单元测试全部通过
[ ] 数据集或仿真可重复生成有效地图
[ ] 定位误差和丢失恢复达到项目阈值
[ ] 目标可生成无碰路径
[ ] 控制输出有界且能到达目标
[ ] TF 树单一、连通、无重复发布
[ ] 底盘桥具备限幅、watchdog 和急停
[ ] 网络、设备和外参只存在于平台配置中
[ ] ROS 1 组件不进入默认原生 ROS 2 运行链
```
