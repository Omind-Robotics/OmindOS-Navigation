# OmindOS 0.2.0-preview.5 — Linux 控制预览

参数配置、ROS 2 关节预览与原版 MEVIUS2 四足闭环仿真的编译发行包。

- 参数 API：提交、校验、保存、按编号读取，保留旧版本。
- 三维工作台：URDF → 电机 → 其他参数，支持运动学预览；ROS 2 关节轨迹联动可选启用。
- 四足默认采用通用关节角度接口，在原版 MEVIUS2 模型中执行 MuJoCo 闭环仿真。
- DM-MC02 客户方案记录为 Jetson USB → 控制板 → 三路 CAN，暂按每路四关节规划。
- 后端业务逻辑以编译模块交付，公开包不附带业务 Python/C 源码；保留模型、参数、使用说明和第三方许可。

此包为 Linux x86_64 + Docker 发行，已包含 Python、ROS 2 和仿真依赖。Windows/Linux 独立桌面工作台仍单独发行，默认使用不要求 ROS。

DM-MC02 通信固件、RS04 实机角度驱动、Jetson ARM64 安装包、客户整机步态和平衡仍待验收。仿真频率与测试结果不代表实机性能。没有连接或使能真实电机。

## 下载与验证

[下载 Linux x86_64 完整包（约 783 MiB）](https://omindos.cn/downloads/navigation/v0.2.0-preview.5/omindos-control-preview-0.2.0-preview.5-linux-amd64.tar)

SHA-256：`276b666a8fee186ba839c30bda8676baba4125a7c9e51408f07ed596cc9db5cc`

编译包验证：110 项回归、9 项 ROS 参数加载、10 项 ROS 工作台浏览器检查、4 项原版 MEVIUS2 动力学场景、18 项真实 ROS 四足联调全部通过。后端编译模块 19 个，无业务 Python/C 源文件。安装后的默认工作台不启动 ROS 控制端。

需 Docker Engine；Python 和 ROS 运行依赖随包提供。下载后解压运行 `./omindos workbench`；可选 `./omindos ros-preview` 或 `./omindos validate-dynamics`。详细说明见附件 README.zh-CN.md。