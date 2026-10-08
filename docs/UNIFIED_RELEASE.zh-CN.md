# OmindOS Navigation 0.2.0-preview.1 发行记录

2026-10-08：已公开发布 GitHub Pre-release。

[发行页、安装包入口和使用说明](https://github.com/maganrobotics-boop/OmindOS-Navigation-Release/releases/tag/v0.2.0-preview.1)

同一 Linux amd64 包提供差速轮式、全向轮式、四足机身速度三种导航配置。
范围为统一导航接口与平面运动学仿真；没有四足步态、平衡或真实硬件验收。

- 完整包：`omindos-navigation-0.2.0-preview.1-linux-amd64.tar`，265,287,680 字节。
- SHA256：`cb5977638d80635bc0bc8002bfdfae9e431f723fdf9e151c8c8a9e51c84e1a3f`。
- 镜像 ID：`sha256:9d7e8482a295689eac77bdd19253d95c098842af031cc792ae2cc3b6079724e1`。
- 应用构建快照：`60abc5b7448e27fcbef723c731a0d8ab3e1f4ec6`；包内附实际应用源码和逐文件校验。
- 20 项测试通过，三种平台仿真均 PASS，连续零指令分别为 1.02、1.17、1.23 秒。
- 完整包重新解压、Docker 导入、精确镜像 ID 核对及篡改拒绝检查通过。
- GitHub 发布任务 `37710017572` 成功；5 个 GitHub 附件的摘要与原文件一致。

完整运行包通过 OmindOS 的独立版本目录提供，GitHub 附件提供源码、校验文件及报告。
独立设备已核对公开地址 HTTP 200、文件长度和校验文件；源站 HTTPS 完整响应的 SHA256 一致。
跨网完整传输尝试遇到连接重置或超时，未将其记为完整下载通过。
下载目录使用独立 Nginx 路由，不依赖网站当前页面版本。

继承代码发行权已由项目负责人确认，应用中保留 Apache-2.0、原维护者和修改 NOTICE。
开发仓库已转私有，本次代码已同步至其 main。
旧 Community/v0.1 的可见性、标签和下载资产保持原状，以保留旧下载入口。

历史候选包具有不同校验值，不能与本发行包混用。
