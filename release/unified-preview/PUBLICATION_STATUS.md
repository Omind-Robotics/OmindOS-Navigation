# 本次预览版发行确认

状态：`APPROVED_PREVIEW`。这表示发行已获授权；实际公开时间和下载地址以 GitHub Release 为准。

项目负责人先前已授权“好的，请继续发行”。针对继承 `nav3d_native` 是否属于团队
可对外发行的自有代码或已取得上游授权这一明确问题，负责人于 2026-10-08 回复
“可以的，没问题”。据此记录本次分发权利已确认；该回复未区分上述两种来源情形，
本记录不另行认定版权所有者。

继承证据与保留信息：

- 导入基线：`maganrobotics-boop/OmindOS-Navigation@b5881e32d69698e7135aa5e40b3b697e9a1e8603`。
- 导入工作流指向 `fushenger/nav_3d2`。
- 原 `package.xml` 与 `setup.py` 声明 `Apache-2.0`，维护者为 `Chen Lvping`。
- 本次补齐 Apache-2.0 许可证全文和修改 NOTICE，保留原维护者信息。
- 历史 v0.1 审计中的该包待确认项，由本次负责人确认补充；不覆盖其他历史组件。

本次发行只包含固定 ROS 基础环境、`src/nav3d_native`、平台配置和配套验证脚本。
不含历史 `3D_NAV`、FAST-LIO、Livox/Unitree SDK、C++ 三维规划/NMPC、硬件 bringup。
构建使用路径白名单与固定 ROS 官方基础镜像，没有继承旧 v0.1 混合源码镜像。

本记录不构成完整系统法律审计。系统依赖保留各自许可证，系统包清单用于追溯。
