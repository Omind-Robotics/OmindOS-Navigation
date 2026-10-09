# OmindOS 四足工作台 · Ubuntu 开发版本

默认展示固定版本 MEVIUS2 的完整外观模型，包含 13 个 STL 网格与 12 个活动关节。
来源为 haraduka/mevius2@4f09680bb575574377b903bbc971bdb2695d507b，附上游 MIT 许可。
使用前可导入自己的 URDF 或完整 JSON 配置，并修改连杆、质量、惯量、关节限位和电机参数。

当前截图来自源码工作台的开发预览。公开下载的 Ubuntu 完整安装包尚未由本次界面修改重新构建。
普通用户继续使用下载首页的一个 Ubuntu 完整安装包；以下构建命令只供内部开发。
本次仅开发 Ubuntu 22.04+ x64，不继续开发 Windows。

界面复用现有工作台后端、三维渲染与本地 API，支持配置版本保存、重启恢复、0.1° 点动、
全身正弦与运动学预览。默认模型使用上游几何、质量、惯量和限位。每条腿的初始展示姿态为
0 / 0.7 / -1.2 rad。预览中的电机到关节坐标使用单位映射；客户电机方向、传动比和零位
必须按实际硬件标定。客户记录中的未知项继续保留为 null，默认模型不是客户整机的标定结果。

三维预览不验证步态、接触、碰撞或平衡，不连接真实电机。机械臂、四轮底盘、协同规划和
机械臂参考照片在四足界面中隐藏。软件停止联锁保留；真实板卡和机器人尚未验收。

内部构建命令：

```bash
npm ci --prefix web/robot-workbench
node web/robot-workbench/build-renderer.cjs
python scripts/fetch_mevius2_workbench.py
python release/workbench/package_binary.py --output build/quadruped --variant quadruped --version 0.3.0-reference-preview.1
```

构建会复核固定上游资源的 Git blob SHA-1 与长度，下载来源不接受用户输入。资源只以
明确白名单本地路径加载。完整构建后还需运行可执行文件、独立解压与浏览器检查，再发布
新的 Ubuntu 完整安装包及 SHA256。此说明不表示这些二进制验收已经完成。

只交付通过检查的安装包和校验值；不交付私有源码、private-build 或编译报告。
