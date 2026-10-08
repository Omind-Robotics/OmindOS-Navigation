# OmindOS 预览版发行

## 0.2.0-preview.5 · Linux 控制预览

[发布说明与校验文件](https://github.com/Omind-Robotics/OmindOS-Navigation-Release/releases/tag/v0.2.0-preview.5) · [下载完整包](https://omindos.cn/downloads/navigation/v0.2.0-preview.5/omindos-control-preview-0.2.0-preview.5-linux-amd64.tar)

Linux x86_64 + Docker，已打包 Python、ROS 2 Humble 和原版 MEVIUS2 仿真依赖。提供参数 API、三维工作台、可选 ROS 2 关节预览及通用角度接口仿真；不附带后端业务源码。解压后运行 `./omindos workbench`。

独立 Windows/Linux 桌面工作台另行发行，默认使用不要求 ROS。RS04 / DM-MC02 实机适配、Jetson ARM64 包及客户整机步态、平衡仍待验收。

下方为历史导航包说明；其功能范围与 preview.5 控制包不同。

---

## 历史版本：OmindOS Navigation 0.2.0-preview.2

[下载完整安装包与校验文件](https://github.com/maganrobotics-boop/OmindOS-Navigation-Release/releases/tag/v0.2.0-preview.2)

统一导航与参数 API 预览版。客户按“URDF → 电机 → 其他参数”提交配置，无需修改系统源码。

## 安装

完整包适用于 Linux x86_64，需已安装 Docker、Python 3.10 或更新版本及 Bash。参数 API 仅用 Python 标准库。解压后：

```bash
./omindos verify
./omindos install
./omindos test
./omindos preview quadruped
```

轮式使用 `wheeled_diff` 或 `wheeled_omni`。导航环境与 preview.1 完全相同，包内已包含镜像，安装无需联网。`test` 包含 13 项参数/配置测试、20 项原导航测试和三平台仿真。

## 提交本机参数

```bash
./omindos api --store "$HOME/omindos-robot-configurations" --port 8085
```

保留该终端，在另一终端提交自己的请求文件：

```bash
curl --fail-with-body -X POST http://127.0.0.1:8085/v1/robot-profiles \
  -H 'Content-Type: application/json' --data-binary @robot-parameters.json
curl --fail-with-body http://127.0.0.1:8085/v1/robot-profiles/PROFILE_ID
```

第一次可用 `examples/robot-parameters.synthetic.json` 熟悉接口；这是合成机构样例，不是客户机器人标定值。修改参数后重新提交得到新编号；相同参数得到同一编号。按旧编号仍可读取旧版，服务重启后继续使用同一存储目录。Ctrl-C 停止服务。升级只需解压新版并沿用包目录以外的配置存储目录；卸载可删除解压目录，客户配置独立保留。

[参数 API 字段与调用说明](docs/ROBOT_PARAMETER_API.zh-CN.md) · [客户参数填写指南](docs/CUSTOMER_ROBOT_CONFIGURATION.zh-CN.md) · [MEVIUS2 原版仿真记录](docs/MEVIUS2_INTEGRATION.zh-CN.md)

## 本版范围

| 功能 | 状态 |
| --- | --- |
| URDF 与电机参数校验、提交、版本保存、按编号读取 | 已提供，本机 HTTP 服务 |
| 差速、全向、四足机身速度导航预览 | 包含原 preview.1 平面仿真环境 |
| MEVIUS2 原版起身、站立、行走、归零验证 | 提供脚本与已有记录；模型、权重及 MuJoCo/PyTorch 依赖需另备 |
| 参数启用到 ROS 2 控制端 | 尚未接通，返回 runtime_applied=false |
| 三维可视化调参界面 | 此包未包含 |
| 客户模型、真实机器人步态/平衡/电机验收 | 尚未完成 |

API 只监听 127.0.0.1，不自动启动控制器，不发送电机命令。当前接收独立旋转关节与固定连接；任意 URDF 自动训练、网格下载、闭环机构转换不在本版范围。原版 MEVIUS2 仿真记录不等于本包 ROS 2 联调或客户实机结果。

`runtime/` 原样保留 preview.1 的镜像、源码、许可证、系统组件清单与校验文件。根目录 `MANIFEST.json`、`VALIDATION.json`、`SOURCE_FILES.sha256` 和 `SHA256SUMS` 记录此次新增部分；第三方通知见 `LICENSES/`。旧版本的标签与资产未改动。
