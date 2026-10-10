# RM75 PR #23 接续检查点 · 2026-10-10 19:00 CST

用户已授权“必须做完后合并提交”；随后要求“下页继续”。本检查点暂停工作，PR 仍为草稿，尚未合并。下一页沿此记录继续，验收通过后可直接合并，无须再次索要合并授权。

## 当前源码与已通过验证

- 仓库：Omind-Robotics/OmindOS-Navigation
- PR：https://github.com/Omind-Robotics/OmindOS-Navigation/pull/23
- 分支：codex/rm75-cad-workbench-20261010；基线 main：612890679c1c584051c08eb15faa66ebcb9a8248
- 最后产品提交：fdb6813a3d1877be7965a182022cbb1e5c7acb96；本检查点只增加测试/记录。
- 128 项源码回归通过：[工作流 38046342736](https://github.com/Omind-Robotics/OmindOS-Navigation/actions/runs/38046342736)。
- 10 项合成传输网格浏览器回归通过：[工作流 38046342739](https://github.com/Omind-Robotics/OmindOS-Navigation/actions/runs/38046342739)。含新增协同按钮可用性检查。
- 实际模型用 28 个保守碰撞盒；14 个唯一真实 STL 共 68,804,176 字节，20 个外观实例。原始几何、轴线/限位、未知动力学参数均保留。
- 修复客户模型协同演示误用旧参考轨迹、无四轮关节导致按钮禁用，以及末端聚焦使用旧名字的问题；当前协同演示走真实 TCP 的碰撞规划。
- 碰撞算法/覆盖证明与 774 姿态对照记录见 rm75-collision-runtime-20261010.md/json。

## Ubuntu 单包已完成的验收

Ubuntu 22.04 Docker 编译，Python 3.10.12 / Nuitka 4.2.2 / numpy 1.26.4 / scipy 1.14.1。独立解压后以下全部通过：
- 基础原生可执行验收 17 项；
- 规划验收 9 项；
- 导航与停车/停止联锁 13 项；
- 实际 RM75 捆绑资源接口 7 项：三型号自动注册、所有实际 STL 的 SHA256、三型号双臂/腰部关节路径及双 TCP 位移、停止 423、配置重启恢复。

单一压缩包：omindos-workbench-0.3.1-rc.1-linux-x64.tar.gz
- 大小：87,982,793 字节
- SHA256：f51612e2717e7a9c1fe7cc316ac0b924127fb438a92364beac4a2eacfd666b9f
- 实际模型 manifest SHA256：cb38efe909587f762e7ad66e408f6de933dfdc87addfdbc0653c36fbf1294cf1
- 产品源提交：fdb6813a3d1877be7965a182022cbb1e5c7acb96
- 原生编译在 3e3323b97421040cf4b8978359fca0949d033327 开始，随后唯一产品变化是未编译的前端 JS。139 个源文件逐项对 Git blob 摘要复核一致。已校正 manifest 的 source_commit、重新压缩、独立解压，并重新通过上述 46 项，未重新编译未变化的后端。

原始 JSON 汇总保存在同目录 rm75-ubuntu-candidate-20261010.json。安装包尚未公开发布，公开 v0.3.0 附件未替换。

## 必须继续完成的最后界面验收

完整 CAD 源码界面已通过：
1. 20/20 实际网格实例；
2. 实际关节与 left_tool0/right_tool0 映射；
3. 工具末端聚焦；
4. 写入程序且未知参数保持 null；
5. 双臂加腰部目标关节路径播放。

后续点击协同按钮的 Playwright 稳定性等待在 30 秒内超时。第一次发现了实际按钮禁用问题并已修复；第二次按钮已启用，但完整浏览器流程仍未通过。当时原生编译并行且主机内存紧张；尚不能断言仅环境问题，不能跳过此门禁直接合并。

已提交的 scripts/check_rm75_installed.cjs 使用真实用户界面和真实可执行文件，不注入产品状态；增加 120 秒动作等待、页面崩溃和 WebGL 存活断言。下一页在编译已结束后单独运行它，使用最终独立解压的可执行文件。必须完成双 TCP 协同位移、播放中停止/复位不续跑、390px 布局、重启后配置与 20 个网格恢复，总共预期 9 项，页面错误与资源失败为 0。并查看桌面、手机截图。若再失败，修复实际原因，不强制点击绕过不可用状态或削弱断言。

## 已授权构建机与接续命令

Desktop Commander 设备 VM-0-3-ubuntu：
f7d97a0e-0317-4b45-8c43-db15f58ada8c
宿主机 Ubuntu 24.04.4，Google Chrome / SwiftShader。不声称 GPU 性能验收。

工作根目录：
/home/ubuntu/omindos-build/rm75-final-20261010-1826

最终独立解压目录：
compiled-final/fresh-unpack/omindos-workbench-0.3.1-rc.1-linux-x64

在该主机运行（所有路径绝对，下面命令可直接执行）：

```bash
NODE_PATH=/home/ubuntu/omindos-build/rm75-final-20261010-1826/browser/node_modules CHROMIUM_EXECUTABLE=/usr/bin/google-chrome node /home/ubuntu/omindos-build/rm75-final-20261010-1826/check-rm75-installed.cjs /home/ubuntu/omindos-build/rm75-final-20261010-1826/compiled-final/fresh-unpack/omindos-workbench-0.3.1-rc.1-linux-x64/OmindOS-Workbench /home/ubuntu/omindos-build/rm75-final-20261010-1826/installed-browser > /home/ubuntu/omindos-build/rm75-final-20261010-1826/installed-browser.log 2>&1
```

- 检查脚本 SHA256：8b2fb94139b823c8c2d92266edb603a3e2b75514ec691ecfee00c289434677f8
- 输出：installed-browser/ACTUAL_CAD_BROWSER.json 和 actual-rm75-desktop.png、actual-rm75-mobile.png
- 模型：rm75-runtime；源码快照：source；Python 编译环境 compiler 只能在 Ubuntu22 Docker 中调用；前端测试可直接用宿主 node/Chrome。
- 最终报告：compiled-final/FINAL_ARTIFACT.json、四个 *fresh-unpack-validation.json；构建日志 compile-final.log、finalize-artifact.log。
- Docker 镜像 omindos/control-builddeps:preview5；编译和最终打包进程均已结束。已清理本任务旧源码测试容器与已不使用的原生中间 C/对象文件；不可清理无关项目。
- 原生软件不需要外部 Python/Node/Docker；这些只供开发与浏览器测试。
- 构建目录持有完整实际 CAD 与候选包，不要重新传输 69 MB 网格。
- 本地临时工作区 /workspace/scratch/d9470231a6ca/repo 是部分快照，不要盲推整个目录。用 GitHub create_tree 基于最新远端树，仅提交明确文件；update_ref 必须带 expected_sha。

## 通过后的收尾顺序

1. 把完整安装版浏览器证据写入最终验收报告，更新之前两份历史报告的接续说明。当前旧 PR 正文仍有“64盒、122项测试、完整CAD/安装包待验收”等过时描述，需要整体更新为实测结果。
2. 对最后提交确认 GitHub 源码与浏览器检查通过；若后续仅测试/文档改动，可复用已验证产品二进制并明确产品源提交，不要无理由重新编译。
3. mark_pull_request_ready_for_review，再以最新 expected_head_sha 合并 PR #23；最后读回 merged=true 与 main 提交身份。
4. 向用户简述完成项和 PR 链接。实际 CAD 未作为公开附件发布；公开发布安装包是后续独立动作。

范围仍是几何运动学软件验收。真机零位/传感器型号、连续碰撞、真实驱动、动力学与力控未验收，未知参数不可补造。
