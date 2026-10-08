# ROS 2 联动（preview.6）

PR #2、#3 与通信配置功能已经合并。桌面包提供完整的 ROS 2 预览面板及本机 HTTP 桥接，支持保存并加载、试运行、停止、状态回读和软件急停转发。它不包含 ROS 2 运行库，也不会连接真实电机。

## 已有本机 ROS 2 预览服务

工作台与服务必须使用**同一配置目录**。先启动服务，再运行：

```sh
./OmindOS-Workbench --store /absolute/path/to/profiles --runtime-url http://127.0.0.1:8086
```

Windows 的入口名为 `OmindOS-Workbench.exe`。仅接受 `http://127.0.0.1:端口` 或 `http://localhost:端口`，不接受远程地址、路径或嵌入凭据。端口必须与实际服务一致。

## Linux x64 配套运行方式

现成 ROS 2 Humble 编译运行环境已包含在 [OmindOS control preview.5 Linux/Docker 包](https://github.com/maganrobotics-boop/OmindOS-Navigation-Release/releases/tag/v0.2.0-preview.5)。完整包需要 Docker Engine；其 Source code 附件不是安装包。

先按该发行包的安装说明加载镜像。确认 `omindos/control-preview:0.2.0-preview.5` 已存在后，可单独启动关节预览服务，复用桌面工作台默认配置目录：

```sh
mkdir -p "$HOME/OmindOS/workbench-configurations"
docker run --rm --init --network host --user "$(id -u):$(id -g)" \
  --cap-drop ALL --security-opt no-new-privileges \
  -e HOME=/tmp -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=42 \
  -v "$HOME/OmindOS/workbench-configurations:/data/profiles" \
  omindos/control-preview:0.2.0-preview.5 \
  profile-runtime --store /data/profiles --port 8086
```

保留该终端，在另一个终端运行：

```sh
./OmindOS-Workbench --runtime-url http://127.0.0.1:8086
```

如果使用整个 Linux/Docker 工作台而非桌面程序，可在该完整包目录执行 `./omindos ros-preview`，浏览器访问其显示的地址。两种工作台不要占用同一个端口。

## 操作与验收范围

1. 加载或导入模型，填写参数和目标姿态。
2. 点击 ROS 2 区域的“保存并加载”。
3. 确认状态为已就绪，再点击“试运行”。
4. 点击“停止”或软件急停后保持当前位置；复位不会自动继续运动。

参数修改、会话失配、限位/限速失败和心跳丢失均会阻止继续执行或停止预览。未启动服务、配置目录不一致或端口填写错误时，不得把未连接状态当作功能成功。

Windows/Linux 原生桌面包均测试编译程序的启动、静态资源和本机桥接协议；该协议测试使用显式标记的本机测试服务。真实 ROS 2 DDS/HTTP 运行验证使用 Linux/Docker。Windows 原生 ROS 2 节点及 WSL 部署不属于本次验收范围。

当前仅进行运动学与静态重力预览。`runtime_applied=true` 仅表示 ROS 2 预览节点已加载；`hardware_active` / `hardware_validated` 仍为 false。软件急停不能代替硬件急停。
