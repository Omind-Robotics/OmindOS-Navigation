# 项目入口

- 通用部署与跨平台入口：仓库根目录 `README.md`
- 新传感器/底盘适配：`docs/PORTING_GUIDE.md`
- 不绑定硬件的启动：`roslaunch nav_bringup bringup_navigation.launch`
- 简洁命令文档：`COMMANDS_BRIEF.md`
- 架构与数据流：`ARCHITECTURE.md`
- 可通行区域提取：`map_process/README.md`
- 全局规划：`global_planner/ReadMe.md`
- 局部规划：`local_planner/README.md`

# 启动顺序
## 1. 定位（地图维护）
```bash
# 用于仿真（仿真定位使用真值，但需要发布一个静态地图用于障碍物检测）
roslaunch map_publisher map_publisher.launch
# 用于实机 MID360（FAST-LIO 提供 map->base_link）
roslaunch fast_lio localization_mid360_nav.launch
```

## 2. 全局规划器
```bash
roslaunch planning_3d 3d_planning.launch # PRM-A*
roslaunch bezier_path_optimizer bezier_path_optimizer.launch # 贝塞尔插值
```

## 3. 局部规划器
```bash
roslaunch nmpc_planner nmpc_controller.launch # 按下i进入自动模式，机器人跟踪/local_plan，按下q进入手动控制模式，机器人接收其他速度输入
```

## 4. 障碍物检测
```bash
roslaunch obstacle_processor obstacle_processor.launch
```
