# 四足交付拆分与体积实测

测量日期：2026-10-09（北京时间）。本次仅创建私有构建候选和 Draft PR；没有更新 main、发布 tag 或旧资产。

## 已实际构建的结果

MB 使用十进制（1 MB = 1,000,000 字节）；下表是用户下载的安装包大小，不是展开目录或 Docker 磁盘占用。

| 交付物 | 实测字节 | 下载体积 | 范围及依赖 |
|---|---:|---:|---|
| 原四足 preview.5 Linux amd64 | 820,449,280 | 820.45 MB | ROS 2、Python、PyTorch CPU、MuJoCo、工作台与 MEVIUS2 闭环仿真；宿主需 Docker |
| 完整运行包优化候选 | 540,416,000 | 540.42 MB | 保持原业务程序、模型、ROS 和仿真；同样需要 Docker |
| 四足轻量工作台 Linux x64 | 21,990,166 | 21.99 MB | 自带编译程序与 Python 运行库；本地调参、三维、运动学无需 Docker/ROS/Python 另装 |
| 四足轻量工作台 Windows x64 | 8,232,521 | 8.23 MB | 同上；Windows 原生编译，未签名 |
| 原机械臂 preview.6 Linux x64（参考） | 21,994,602 | 21.99 MB | 桌面工作台，ROS 服务独立启动 |

**相同完整功能的安装包减少 280,033,280 字节，即 34.13%。** Linux 轻量包比原完整包小 97.32%，这个差异来自交付范围拆分，不能称为同功能压缩率。

轻量包解压后文件总量：Linux 56,246,042 字节；Windows 22,042,034 字节。两平台由各自 Python/Nuitka 原生工具链构建，运行库不同，不以 Windows 的大小推算 Linux。

## 大文件实际来自哪里

下表是原镜像内普通文件的逻辑字节总量，排除目录和符号链接本身。它与最初 `du -B1` 的分配块数不同，不能直接加总为下载包大小；“全部虚拟环境”包含下面的各 Python 依赖。

| 实际内容 | 展开后的逻辑文件量 | 轻量工作台是否需要 | 完整包处理 |
|---|---:|---|---|
| Python 虚拟环境（总计） | 814.96 MB | 只需自身编译运行库 | 清理后 804.19 MB |
| PyTorch `torch/` | 656.71 MB | 不需要 | 保留；固定策略执行依赖它，已经是 CPU 版 |
| NumPy + `numpy.libs` | 64.46 MB | 不需要 | 保留 1.26.4，保持 ROS 扩展 ABI |
| MuJoCo | 21.03 MB | 不需要 | 保留 3.3.7，用于接触与动力学积分 |
| SymPy | 26.08 MB | 不需要 | 保留 PyTorch 依赖，不盲删 |
| `/opt/ros` | 76.09 MB | 本地预览不需要 | 保留；不包含系统侧 DDS/Python 依赖的全部占用 |
| 系统 Python 标准库 `/usr/lib/python3.10` | 41.86 MB | 使用独立打包运行库 | 保留 |
| 系统 Python 第三方目录 | 73.86 MB | 不需要整套目录 | 保留 ROS 所需依赖 |
| 模型、策略与参考资源 | 65.64 MB | 不需要完整动力学模型 | 保留固定校验值 |
| 19 个编译业务模块 | 3.53 MB | 仅编译工作台所需模块 | 完整包逐文件不变 |
| pip 下载缓存 | 217.74 MB | 不需要 | 已删除 |
| apt 软件包索引 | 87.20 MB | 不需要 | 已删除；以后装系统包需先 apt-get update |
| Cython 包主体 | 10.48 MB | 仅编译时需要 | 连同其元数据及 wheel 构建工具移除 |
| 系统 GCC 目录 | 101.59 MB | 不需要 | 本次保留，防止破坏 ROS 开发依赖 |

ROS 实际安装 191 个 `ros-humble-*` 包，dpkg 的 Installed-Size 合计 86,168 KiB。该口径与上表文件遍历也不同。

发行镜像中，依赖准备层压缩后约 334.35 MB，复制 Python 虚拟环境的层约 214.23 MB，复制业务和模型资源的层约 11.18 MB。这解释了为什么业务模块只有几 MB，完整交付仍达 820 MB。没有把这些层再武断拆成“某依赖压缩后占多少”。原外层 `.tar` 已包含 `.image.tar.gz`，对外层再压一次不能解决依赖体积。

## 实施方式

完整包使用 `release/control-preview/Dockerfile.optimized`：以已核对的 preview.5 为输入，在中间阶段移除 pip 缓存、apt 索引、Cython、wheel 和虚拟环境字节码缓存；最终阶段复制清理后的文件系统。仅在原镜像上新增删除层会把旧字节留在交付层里，多阶段复制使其不进入最终镜像（参见 [Docker 多阶段构建文档](https://docs.docker.com/build/building/multi-stage/)）。

没有升级或替换 PyTorch、NumPy、MuJoCo、ROS。保留 pip/setuptools、第三方源码和许可文件。`/opt/omindos` 的 71 个文件 SHA256 前后完全一致，含编译模块与模型。优化包内业务版本仍显示 preview.5，包名另带 `size-candidate.1`，避免与原版混淆。

卸载 GCC/Python 开发包的 apt 模拟计算会连带移除 ROS 包，因此没有执行这项删除。长期可从更小 ROS 运行基线重新建包，但本次没有构建该方案，不给体积预测。把 PyTorch 换成 ONNX Runtime、去掉 Torch 头文件/测试目录、使用其他压缩算法、Jetson ARM64 优化也未实施或估算。

轻量包复用现有机械臂 `release/workbench/package_binary.py`、Nuitka 4.2.2、同一套参数 API/运动学/三维渲染器，只增加四足变体和构建参数。没有另写一套工作台后端。默认12关节模型为明确标注的合成教学示意，支持导入客户 URDF；它不是客户 CAD 或 MEVIUS2 仿真验收模型。机械臂设备通信和四轮底盘专用面板在四足变体中隐藏。

## 功能与兼容性

| 功能 | 轻量工作台 | 完整优化包 |
|---|---|---|
| 参数 API、三维模型、配置版本、0.1° 点动、正弦运动学 | 独立运行 | 保留 preview.5 自带工作台 |
| ROS 2 关节预览 | 需独立 ROS 服务、相同配置目录 | 内置 Humble 服务 |
| PyTorch 策略、MuJoCo 浮动基座与接触动力学 | 不包含 | 保留四组 MEVIUS2 场景 |
| Linux | Ubuntu 22.04 CI 编译和执行；Ubuntu 24.04 上补做浏览器验收 | Linux amd64 + Docker；容器内 Ubuntu 22.04 |
| Windows | Windows Server 2022 x64 CI 编译、执行及重新解压测试 | 没有构建 Windows 原生完整包 |
| Jetson ARM64、真实 RS04/DM-MC02 电机闭环 | 未验收 | 本次未构建和验收 |

## 实际验证

- 两个桌面平台各通过 45 项共享源码单元测试；实际可执行程序在打包前、独立重新解压后各通过 15 项检查，包括12关节、点动、正弦、持久化、停止和缺失 ROS 服务处理。
- 编译后的 Linux 桌面通过真实浏览器加载、12关节检查、点动和停止检查；1440px 与390px截图无页面脚本异常，并完成目视检查。
- 完整优化镜像通过 110 项编译模块回归、9 项真实 ROS 参数加载/DDS 检查、18 项四足 ROS 检查，以及前进、侧移、转向、站立受推四组动力学测试。
- **另外将新编译的 Linux 轻量桌面与优化后完整镜像实际连接，通过9项跨包 ROS/DDS 检查**：共享配置、模型广播、关节运动、心跳丢失停止、软件停止、复位不自动继续、服务退出后本地配置仍可用。这是独立于桌面模拟服务检查的真实 ROS 验证。Windows ROS/WSL 联调未测试。
- 新完整包单独解压，核对21个清单文件，重新 `docker load`，运行随包启动器成功。原820 MB安装包 SHA256仍为原发行值。
- 桌面 archive 未发现业务 `.py/.pyc/.pyo/.c/.cpp/.h/.map`；完整应用目录未发现业务源码或客户私有记录。第三方依赖及许可按原版保留。编译不等于防逆向保证。

## 复现、证据与交付位置

私有构建目录：`/home/ubuntu/omindos-build/size-study-20261009`。完整候选位于 `distribution/`，未复制到官网公开下载目录。

桌面构建：[GitHub Actions run 37847826019](https://github.com/Omind-Robotics/OmindOS-Navigation/actions/runs/37847826019)，构建代码 `80cd7d907a97e6b3bdc35691fe5dcc3be5849a52`。Actions 候选仅对仓库有权用户开放，保留到2026-10-22；后续报告提交只补文档、测量工具和检查脚本，不改变这批二进制。

原镜像 ID：`sha256:5fb43c0a3be2960f41cb06f22d8efa3db4a86b06b014f6eb60ec0f7f68b13fbb`；优化镜像 ID：`sha256:094e974a408dbc17c6e3ef1440c6308dcd6c0fb4c3a29bc53862dabf67dd1dc1`。

脚本入口：`measure_image.py` 测量；`validate_size_candidate.py` 运行冻结测试；`package_size_candidate.py` 在精确镜像验收通过后生成独立候选；`scripts/check_quadruped_desktop_browser.cjs` 检查桌面浏览器；`scripts/check_split_workbench_ros.py` 检查拆分后的真实ROS联动。

在包含原构建镜像的机器上，先核对原tag对应上述镜像 ID，再运行：

```bash
docker build --pull=false --network none -f release/control-preview/Dockerfile.optimized \
  --build-arg SOURCE_IMAGE=omindos/control-preview:0.2.0-preview.5 \
  -t omindos/control-size-candidate:20261009 release/control-preview
python release/workbench/package_binary.py --output build/quadruped \
  --variant quadruped --version 0.2.0-size-candidate.1
```

构建环境需要编译器、Nuitka、Node/esbuild；这些属于构建机依赖，不要求桌面用户安装。完整包优化路径复用原编译镜像，没有把私有源代码COPY进任何镜像层，也不是从零重建所有第三方依赖。

结构化精确字节、校验值和验收结果见 [quadruped-size-study-20261009.json](validation/quadruped-size-study-20261009.json)。Docker inspect 本机报告3,137,851,788→2,520,904,935字节，这个引擎使用OCI压缩内容与解包快照存储，不能把它直接当下载量或普通文件总和；交付比较以实际 archive 字节为准。
