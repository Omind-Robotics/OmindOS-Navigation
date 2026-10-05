# M20Pro 前雷达楼梯导航：启动与录包命令

本流程固定使用前雷达 `/m20/lidar/front` 和标准楼梯步态 `4099`，不启动在线避障。脚本会复用已有的传感器通信，自动补齐 ROS 环境、固定地图路径和底盘 bridge 参数。

运行前确保 M20Pro 与 NUC 网络正常，遥控器可随时接管。楼梯导航期间必须保持测试区域清空。

## 命令 1：建图并提取可通行区域

已有同一轮生成的 `scans.pcd` 和 `traversable_areas.pcd` 时，可以跳过本命令。

```bash
/home/arts-inpector/ART_3dnav/src/3D_NAV/nav_bringup/scripts/m20_stair_workflow.sh map
```

脚本会自动启动传感器通信和前雷达 FAST-LIO。移动机器人完成建图后，在该终端按一次 `Ctrl+C`；脚本随后自动检查本轮点云与轨迹，并生成：

```text
/home/arts-inpector/ART_3dnav/src/data/traversable_cloud/map_process_test_front/traversable_areas.pcd
```

出现 `Traversable map ready` 后，本步骤完成。

## 命令 2：启动定位、规划和控制

在导航终端运行并保持：

```bash
/home/arts-inpector/ART_3dnav/src/3D_NAV/nav_bringup/scripts/m20_stair_workflow.sh nav
```

该命令一次启动：

```text
前雷达定位 -> 全局规划 -> Bezier 平滑 -> NMPC 控制 -> RViz
```

它不会启动 `obstacle_processor`，也不会立即进入自动控制。

## 命令 3：启动楼梯底盘链路

先使用遥控器切换到标准楼梯步态和导航模式，再在另一个终端运行并保持：

```bash
/home/arts-inpector/ART_3dnav/src/3D_NAV/nav_bringup/scripts/m20_stair_workflow.sh base
```

脚本只在 `/MOTION_INFO` 同时满足 `state=17`、`gait=4099` 时启动，并自动合并以下两个进程：

```text
/cmd_vel -> ROS1/ROS2 parameter_bridge -> stair motion adapter -> /NAV_CMD
```

看到 `M20 stair base is ready` 后，底盘链路已就绪。状态或步态不正确时脚本会拒绝启动，不要绕过检查。

## 开始导航

1. 在 RViz 使用 `3D Nav Goal` 下发目标。
2. 确认已经生成有效路径。
3. 回到命令 2 的终端，按 `i` 进入自动模式。

退出时按以下顺序操作：

1. 在命令 2 的终端按 `q` 退出自动模式，再按 `s` 清零速度。
2. 在命令 3 的终端按 `Ctrl+C`，停止楼梯 adapter 和 bridge。
3. 在命令 2 的终端按 `Ctrl+C`，停止导航；脚本会在底盘进程已退出时一并清理由本流程启动的传感器通信。
4. 使用遥控器退出导航模式并恢复接管。

手动模式下 `q` 是左前移动，不是停止；必须再按 `s` 清零速度。

## 命令 4：一键录取数据包

需要保存楼梯导航测试数据时，在独立终端运行并保持：

```bash
/home/arts-inpector/ART_3dnav/src/3D_NAV/nav_bringup/scripts/m20_stair_workflow.sh record
```

该命令会复用已经启动的前雷达通信；通信尚未启动时会自动启动。数据包包含前雷达、IMU、TF、定位里程计、规划路径、控制状态和 `/cmd_vel`，使用 LZ4 压缩并在单个文件达到 4096 MB 时自动分包。

完成测试后，在录包终端按一次 `Ctrl+C`，等待 rosbag 索引写入完成后再关闭终端。文件默认保存在：

```text
/home/arts-inpector/ART_3dnav/src/data/bags/m20_stair_<时间>*.bag
```

录包会在磁盘剩余空间低于 10 GB 时自动停止。需要保存到外接盘时，在命令前指定目录：

```bash
M20_BAG_DIR=/media/arts-inpector/<外接盘目录> /home/arts-inpector/ART_3dnav/src/3D_NAV/nav_bringup/scripts/m20_stair_workflow.sh record
```

建议在命令 2 启动成功后开始录包，再执行命令 3 和下发导航目标，这样可以完整记录定位、规划和底盘控制过程。
