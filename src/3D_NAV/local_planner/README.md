# nmpc_planner (ROS1)

基于CasADi实现的非线性模型预测控制（NMPC）局部规划器与控制器，提供键盘手动控制、自动跟踪全局路径、障碍物避障及终点朝向对齐（GOAL_ALIGN）等功能。工作空间位于`/home/arts/Galilio/3D_nav_sim_ws`。

## 功能概述
- NMPC 局部规划：将全局路径转换为可跟踪的速度序列，并发布`/local_plan`（线速度v、角速度omega成对排列）。
- 障碍物避障：在MPC目标函数中加入TEB风格的平滑惩罚，可配置安全距离、影响距离和障碍物权重，并对原始障碍物进行网格化去重。
- 朝向对齐：`NavState::GOAL_ALIGN`状态时仅输出纯旋转控制，将机器人朝向对齐到目标姿态。
- 控制器节点：支持键盘手动模式（默认，按`i`切到自动，`q`返回手动），自动模式下读取`/local_plan`首个控制量并发布`/cmd_vel`，同时发布机器人当前位姿`/curr_state`。
- 轨迹可视化：发布`/local_path`（`nav_msgs/Path`）用于RViz查看MPC生成的轨迹。

## 目录结构
- `src/`、`include/`：核心代码（控制器、局部规划器、MPC求解器、键盘输入）。
- `config/controller_params.yaml`：默认参数示例，可按需加载或在launch中覆盖。
- `launch/nmpc_controller.launch`：同时启动控制器与局部规划器的示例启动文件。

## 节点与话题
### nmpc_controller_node
- 发布：`/cmd_vel`(`geometry_msgs/Twist`)，`/curr_state`(`std_msgs/Float32MultiArray`，顺序为[x,y,z,yaw,roll,pitch])。
- 订阅：`/local_plan`(`std_msgs/Float32MultiArray`，按[v0,omega0,v1,omega1,...]存储)，`/navigation_state`(`std_msgs/UInt8`，0~5枚举)。
- 依赖 TF：`map`→`base`（默认`map`→`motion_link`，launch示例使用`base_link`）。
- 模式切换：启动后默认手动；`i`切自动，`q`或`Ctrl+C`回手动；`w/x/a/d/q/e/z/c/s`为速度控制。

### nmpc_local_planner_node
- 发布：`/local_plan`（控制指令序列），`/local_path`（MPC预测轨迹）。
- 订阅：
  - `/path_smooth`(`nav_msgs/Path`)：全局规划路径，从第10个点起按步长1下采样并缓存末端目标姿态。
  - `/curr_state`(`std_msgs/Float32MultiArray`)：当前位姿。
  - `/obs_raw`(`std_msgs/Float32MultiArray`)：每个障碍物为[x,y,z]三元组，按网格大小`obstacle_grid_size`去重后参与避障。
  - `/navigation_state`(`std_msgs/UInt8`)：导航状态机，`TRACKING`执行NMPC，`GOAL_ALIGN`执行纯旋转，`COMPLETED/ABORTED`清空路径并发布零控制。

### 导航状态枚举
`WAITING(0), GLOBAL_PLANNING(1), TRACKING(2), GOAL_ALIGN(3), COMPLETED(4), ABORTED(5)`。

## 主要参数
可在`launch/nmpc_controller.launch`调整：
- 通用：`base_frame`（默认`motion_link`）、`map_frame`（默认`map`）。
- 控制器：`control_frequency`, `plan_size`。
- 规划器：`planning_frequency`, `desired_path_size`, `path_buffer_limit`, `robot_height`, `pos_tolerance`, `min_angular_vel`, `obstacle_grid_size`。
- MPC：`mpc/control_horizon`, `mpc/prediction_steps`, `mpc/max_linear_vel`, `mpc/max_angular_vel`, 平滑权重`mpc/smooth_v_weight`, `mpc/smooth_omega_weight`。
- 避障：`mpc/safe_distance`, `mpc/influence_distance`, `mpc/obstacle_weight`, `mpc/use_time_weight`, `mpc/max_obstacles_consider`, `mpc/obstacle_influence_range`, `mpc/smooth_epsilon`。

## 构建
```bash
cd /home/arts/Galilio/3D_nav_sim_ws
catkin_make  # 或 catkin_make --pkg nmpc_planner
```
需要依赖：`roscpp`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `Eigen3`，以及已安装的 CasADi（头文件/库默认搜索`/usr/local`或`/opt/casadi`）。

## 运行与使用
```bash
source /home/arts/Galilio/3D_nav_sim_ws/devel/setup.bash
roslaunch nmpc_planner nmpc_controller.launch
```
该 launch 同时启动控制器与局部规划器并加载示例参数。若需单独启动：
```bash
rosrun nmpc_planner nmpc_controller_node
rosrun nmpc_planner nmpc_local_planner_node
```
运行时需提供：
- TF: `map`→`base_frame`变换。
- 全局路径：向`/path_smooth`发布`nav_msgs/Path`。
- 障碍物：向`/obs_raw`发布`Float32MultiArray`（每障碍物[x,y,z]）。
- 导航状态：按需要发布`/navigation_state`（例如2=TRACKING，3=GOAL_ALIGN）。

键盘手动模式可直接通过终端输入按键控制速度；切入自动模式后将跟随`/local_plan`输出。
