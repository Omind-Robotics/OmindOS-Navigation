# OmindOS Navigation 0.2.0-preview.1

**统一导航预览版 · Linux amd64。** 项目负责人已于 2026-10-08 确认继承代码可对外发行。
来源和确认记录见 `PUBLICATION_STATUS.md`。

一个安装包提供差速轮式、全向轮式、四足机身速度三种配置。三者共用 ROS 2 Humble
导航入口、建图/定位/路径规划示例、速度限制与超时停机逻辑。

## 安装及运行

解压完整发行包，进入解压后的目录。在有 Docker 的 Linux x86_64 主机执行：

```bash
chmod +x omindos
./omindos verify
./omindos install
./omindos test
./omindos preview quadruped
```

轮式分别使用 `./omindos preview wheeled_diff` 和 `./omindos preview wheeled_omni`。
`test` 自动运行单元/ROS 消息测试与三种平台的到达目标、连续零指令检查，完成后退出。
`preview` 保持运行并输出日志；按 Ctrl-C 停止。此次不提供三维图形界面。
Docker 需要当前账户已有使用权限；安装器不会修改用户组、宿主网络或宿主 ROS。

镜像随完整发行包提供，安装和测试无需访问 Docker Hub 或下载旧 v0.1 运行包。
校验失败会在导入镜像或启动容器之前退出；安装后还核对精确镜像 ID。
运行限制为隔离网络、1 GiB 内存、最多 2 CPU，不挂载硬件或宿主文件系统。

## 本版支持范围

| 配置 | 已实现 | 尚未包含 |
| --- | --- | --- |
| 差速轮式 | 前后/转向机身速度，横移拒绝，平面仿真 | 真实底盘、电机、制动验收 |
| 全向轮式 | 前后/横移/转向机身速度，平面仿真 | 全向规划优化、真实轮系验收 |
| 四足 | 同一导航核心与机身速度配置，平面仿真 | 步态、平衡、关节、足端接触、CAN、实机验收 |

本版使用原生 Python 二维示例节点；没有声明 ROS2/C++ 三维链已运行。
仿真建图使用预设采样位姿。到达目标的误差和耗时仅用于该演示验收，不能作为实体产品指标。
任何配置都不能直接作为真实机器人运动驱动。

## 包内组成

- `omindos`：统一安装、校验、测试和预览入口。
- `omindos-navigation-0.2.0-preview.1-linux-amd64.tar.gz`：完整 Docker save 压缩镜像。
- `IMAGE_ID`、`SHA256SUMS`：精确镜像标识和文件完整性校验。
- `MANIFEST.json`：版本、基础镜像、来源提交、平台及发布状态。
- `VALIDATION.json`：独立镜像的测试结果。
- `SOURCE_FILES.sha256`、`OS_PACKAGES.tsv`：应用源码校验与系统包清单。
- `PUBLICATION_STATUS.md`：来源与发行确认记录。

- `LICENSE`、`NOTICE`：应用许可证、原维护者和本次修改说明。
- `application-source.tar.gz`：本版实际构建的应用源码、Dockerfile 和验证脚本。

旧 v0.1 标签、镜像及资产保持不变。本版的“支持”限于统一导航接口与平面仿真，
不表示任何实体底盘或四足步态已经验收。
