# OmindOS 四足轻量工作台（未发布构建候选）

完整解压后运行 `OmindOS-Workbench`（Windows 为 `.exe`）。浏览器自动打开
`http://127.0.0.1:8085/`。用户无需安装 Python、ROS 2、PyTorch、MuJoCo、Node.js 或 Docker。
使用 `--verify` 校验内部文件；`--no-browser` 只启动本机服务。

复用机械臂工作台同一套 Nuitka 编译后端、三维渲染和参数 API，包含：

- URDF/完整配置导入导出，关节、连杆、质量、惯量和电机参数编辑。
- 配置版本保存、重启恢复、0.1° 点动、全身正弦及运动学轨迹预览。
- 软件停止联锁；可选 `--runtime-url http://127.0.0.1:8086` 接独立 ROS 2 关节预览服务。

默认12关节模型是**合成教学示意模型**，不是客户整机 CAD、MEVIUS2 动力学模型或标定结果。
请导入实际 URDF 和参数。三维运动学不验证步态、接触、碰撞或平衡，不连接真实电机。
机械臂设备通信和四轮底盘专用界面在此变体中隐藏。

ROS 2 服务与工作台需要相同配置目录；有服务才可使用 ROS 联动。动力学仿真需另装完整
Linux amd64 Docker 包，其保留 ROS 2 Humble、PyTorch CPU、NumPy 1.x、MuJoCo 和原版 MEVIUS2。
本次完整包优化保持旧 preview.5 编译业务模块；新桌面的新增接口不能据此视为已完成跨版本联调。

目标平台为 Windows x64、Ubuntu 22.04+ x64；实际构建、检查结果以候选构建报告为准。
不代表 Jetson ARM64、GPU 或真机驱动验收。Windows 候选未签名。

私有构建命令：
`python release/workbench/package_binary.py --output build/quadruped --variant quadruped --version 0.2.0-size-candidate.1`

仅交付通过检查的 archive 和校验值，不交付 source、private-build、编译报告或 C/Python 业务源码。
