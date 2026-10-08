# DM-MC02 STM32 USB 接入增量

本次补齐 STM32H723 USB CDC 与既有协议核心的接线层，仍是禁用运动的台架程序。
它没有 CAN 发送或电机使能功能，也没有链接启动文件形成可烧录镜像。

## 已实现

- 从初始化后的 HAL RNG 读取四个随机字，生成本次启动的 16 字节 nonce。
  RNG 失败、全零 nonce 或配置不合法时拒绝启动；同一启动内不能重新初始化会话。
- 使用 PRIMASK 保存/恢复保护 USB 回调，保留调用前的中断状态。
- 提交前检查 USB 已配置、CDC class 实例及 TxState，避免未枚举或拔线后解引用空指针。
- 在临界区内交给 STM32 USB 库，只有回调的地址、长度匹配且 TxState 已清零才释放发送缓冲。
  固定厂商 middleware 的 DataIn 会先完成数据包及必要的 ZLP，再清 TxState 并调用 TransmitCplt。
- CDC DeInit 与已枚举后的掉线均锁存故障；重连不恢复旧会话。

固定外设来源为 dmBots/DM-MC02 的提交
`d65a3688981405dc205f628f1a026bde8cf8303e`。
`prepare_dm_mc02_usb.py` 校验 CDC 原文件 Git blob
`1d0c9b82ce8a9e206125ecfcf12abb030f89e1fd`，
只生成新的接线文件；源文件不符、重复应用或输出已存在均拒绝。
不修改或运行厂商的电机控制任务。

## 集成顺序

1. 使用上述固定工程。启用 STM32 HAL RNG 模块、提供正确 RNG 时钟和初始化句柄；
   原 ALL 示例未提供 RNG 初始化，这一步必须单独补齐。
2. 把 `om_usb.c/h`、`om_cdc.c/h`、`om_stm32_usb.c/h` 加入自己的禁用运动工程，
   增加 HAL RNG 实现文件。关节参数与 digest 必须来自实际配置，不能拷贝测试夹具。
3. USB 启动前调用 `om_stm32_usb_init(rng, digest, joints)`，失败则停止启动。
   初次初始化期间不得存在 USB 中断或旧 DMA 缓冲所有权。
4. 用以下命令生成 CDC 文件，审核后替换自己工程中的同名文件：

   `python3 scripts/prepare_dm_mc02_usb.py /path/to/DM-MC02 /tmp/usbd_cdc_if.c`

5. 禁用原电机发送任务；由一个独立主循环或 RTOS task 持续执行
   `om_stm32_usb_poll()`，同时保留 HAL 毫秒 tick。原 ALL 工程进入 RTOS 调度，
   因而不能把 poll 仅放到 scheduler 后不会执行的 while 循环里。
6. USB reset/deinit、端点错误与物理断线须转入 disconnect；DeInit 已由生成文件接线。
   保持固定工程的 USB DMA 关闭。若启用 DMA/cache，则必须另做内存位置、对齐和缓存一致性验收。

## 验证与边界

`python3 -m unittest discover -s tests -p 'test_dm_mc02*.py' -v`：
20 项既有协议测试、8 个便携 CDC 场景、10 个新增 HAL 模拟场景通过。
HAL 模拟覆盖成功收发、空 class、RNG 错误/全零、中断屏蔽恢复、
错误完成回调、断线重连、轮询发现掉线、USB 提交失败和禁止重复初始化。
模拟测试不证明电气连接或真实 USB 时延。

`python3 scripts/check_dm_mc02_stm32_usb.py` 在 CI 拉取固定厂商工程，
使用真实头文件交叉编译接线层和生成的 CDC 回调文件。
这是对象编译检查，不是完整链接、刷写或板卡验证。

下一步仍需 RNG 时钟与板级启动工程、三路 CAN 只读接收及反馈回传，
硬件看门狗/停止输出、RS04 参数回读和逐关节校准；之后才可接入独立使能路径。
Jetson、DM-MC02 与 RS04 尚未连接实测。此次不更新公开 ARM64 发行资产。
