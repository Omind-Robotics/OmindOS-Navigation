# 统一导航发行候选：0.2.0-preview.1

日期：2026-10-08。**历史候选记录**：负责人现已确认发行权，最终版采用新的镜像与校验值。
请以 `release/unified-preview/PUBLICATION_STATUS.md` 和公开发行页为准；以下保留原候选阶段记录。
不是实机正式版，不包含四足步态或电机驱动。

## 已完成的具体结果

完整包：`omindos-navigation-0.2.0-preview.1-candidate-linux-amd64.tar`。

- 大小：265,287,680 字节（约 265 MB）。
- SHA256：`9e41cf4f4dc44423d590129aadc681540d62c641c57cdfcdf12ba9b1df175461`。
- Docker 镜像：`omindos/navigation:0.2.0-preview.1`。
- 镜像 ID：`sha256:0dd175ec916d710f4c57e3ea1cdc878bf5c89e8f75a7d4c00035d310c17818c5`。
- 应用/构建定义快照：`e33cdf30adbb421f90f3698a63c6359fd255e013`。

安装入口已经实现 `verify`、`install`、`test`、`preview`、`profiles`。
同一个包支持 `wheeled_diff`、`wheeled_omni`、`quadruped` 三种仿真配置。
无需下载旧 v0.1 运行包，安装与测试使用隔离 Docker 环境。

## 独立镜像的验收结果

20 项测试全部通过。三种配置均到达目标，连续至少一秒输出零指令：

| 配置 | 结果 | 目标误差（m） | 连续零指令（s） |
| --- | --- | ---: | ---: |
| 差速轮式 | PASS | 0.226 | 1.63 |
| 全向轮式 | PASS | 0.209 | 1.03 |
| 四足机身速度 | PASS | 0.236 | 1.25 |

以上是本次平面运动学仿真的观测值，不是实机精度或性能指标。
镜像全部 16 层、40,921 条路径检查未发现本次明确排除的历史应用目录。
这项检查不等于完整许可证或秘密扫描。

已从完整包重新解压，完成文件校验、Docker 导入、精确镜像 ID 核对和三种配置查询。
另在独立临时副本中篡改说明文件，安装器按预期在调用 Docker 之前拒绝执行。
正常包未被这项负向测试修改。

## 公开前需要确认的事实

本次继承的 `src/nav3d_native` 在已有分发审计中被标为
`OPEN: declaration only; not cleared`。其 manifest 声明 Apache-2.0，
但未找到对应权属或上游授权记录，不能据维护者名字自动认定归属。

需要确认：**这部分代码是否属于团队可对外发行的自有代码，或已经获得覆盖本次分发的授权？**

用户对继续发行的操作授权已经具备；这里缺的是上述来源事实。
确认后记录依据、保留适用声明并完成公共仓库/发行页面上传。
原 v0.1 tag 和公开资产均未修改。

候选包、安装检查、日志和源码补丁已保存在服务器独立研发目录，未放入公共下载目录。
来源详见 [公开状态说明](../release/unified-preview/PUBLICATION_STATUS.md)，
使用方法详见 [候选包说明](../release/unified-preview/README.zh-CN.md)。
