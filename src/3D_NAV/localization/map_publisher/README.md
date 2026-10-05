# Map Publisher

一个轻量级的ROS节点,用于加载PCD点云地图并发布到`/map`话题。

## 功能特点

- **单一功能**: 仅加载和发布地图,不涉及任何定位相关话题
- **避免冲突**: 专为仿真环境设计,不会与仿真器的真值定位冲突
- **体素降采样**: 支持可配置的体素降采样,减少数据量
- **灵活发布**: 支持单次发布或周期性发布

## 依赖

- ROS (Noetic或更高版本)
- Open3D
- open3d_loc (用于点云转换工具)

## 编译

```bash
cd /home/arts/Galilio/3D_nav_sim_ws
catkin_make --pkg map_publisher
source devel/setup.bash
```

## 使用方法

### 1. 基本用法

```bash
roslaunch map_publisher map_publisher.launch map_path:=/path/to/your/map.pcd
```

### 2. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map_path` | string | `$(find map_publisher)/../data/map.pcd` | PCD地图文件路径 |
| `voxel_size` | double | 0.1 | 体素降采样尺寸(米),设为0则不降采样 |
| `map_frame_id` | string | "map" | 地图坐标系ID |
| `publish_rate` | double | 0 | 发布频率(Hz),≤0时仅发布一次 |
| `latch` | bool | true | 锁存模式,新订阅者可收到最后发布的消息 |

### 3. 示例

#### 使用现有地图
```bash
roslaunch map_publisher map_publisher.launch \
    map_path:=/home/arts-inpector/ART_3dnav/src/data/point_cloud/scans.pcd \
    voxel_size:=0.15
```

#### 周期性发布(1Hz)
```bash
roslaunch map_publisher map_publisher.launch \
    map_path:=/path/to/map.pcd \
    publish_rate:=1.0
```

#### 不降采样
```bash
roslaunch map_publisher map_publisher.launch \
    map_path:=/path/to/map.pcd \
    voxel_size:=0
```

## 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map` | sensor_msgs/PointCloud2 | 点云地图 |

**注意**: 此节点不会发布任何定位相关话题(如`/Odometry`, `/localization`等),以避免与仿真器真值冲突。

## 与open3d_loc的区别

- **open3d_loc**: 完整的定位系统,加载地图+实时定位+发布多个话题
- **map_publisher**: 仅加载和发布地图,专为仿真环境设计

## 典型应用场景

在仿真环境中测试规划算法:
1. 仿真器提供真值定位(如Gazebo的ground truth)
2. map_publisher提供静态地图
3. 障碍物检测模块订阅`/map`话题
4. 规划算法使用仿真器真值定位,避免定位误差干扰测试

## 故障排除

### 1. 无法找到PCD文件
```
ERROR: Failed to read point cloud from: /path/to/map.pcd
```
**解决**: 检查`map_path`参数是否正确,确认文件存在

### 2. 加载的点云为空
```
ERROR: Loaded point cloud is empty!
```
**解决**: 检查PCD文件是否损坏,尝试用CloudCompare打开验证

### 3. 编译错误:找不到open3d_conversions
**解决**: 确保已编译`open3d_loc`包:
```bash
catkin_make --pkg open3d_loc
catkin_make --pkg map_publisher
```

## 目录结构

```
map_publisher/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   └── map_publisher_node.cpp
├── launch/
│   └── map_publisher.launch
└── config/
    (可选的配置文件)
```

## License

BSD
