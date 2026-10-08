# DM-MC02 三路 CAN 只读接收与 USB 状态回传

2026-10-08，基线 `5244ece498fb628226a66d4214116288dc0f4e04`（PR #9）。
本增量是可编译、可模拟验证的禁用运动通信模块，**不是可烧录固件，也没有实机通信通过记录**。
不提供 CAN 发送、使能、参数写入、故障清除、零点设置或自动重新使能入口。

## 代码与边界

- `om_stm32_can.c/h`：FDCAN1/2/3 的独立 HAL 接收层，保留 ID、帧类型、DLC、总线来源和 ISR 接收时间；32 项共享静态队列，单次中断最多收 8 帧。保留进入前 PRIMASK，ISR 不解析浮点反馈、不发送 USB、不调用 RTOS API。
- 三路都使用 Classic CAN、BUS_MONITORING、零 TX FIFO/Buffer/Event，扩展 ID 过滤主机地址 253，拒绝不匹配及远程帧。运行前检查三个句柄、READY 状态和厂家 Message RAM offset，HAL 初始化/过滤/启动任一步失败都停下已触及的控制器并锁存。
- `om_feedback.c/h`：按总线和电机 ID 路由，12 关节、每路 4 个；沿用现有 RS04 私有 29 位协议类型 2（状态）、17（参数回复）、21（故障/告警）。类型 24 等其他帧只计 ignored，不推测新固件协议。
- 状态单位是**电机输出轴** rad、rad/s、N·m、℃；没有转换为标定后机器人关节坐标。参数回复保留原始 uint32，不猜参数类型。
- 初始未收到、收到且新鲜、收到但过期分别可辨。观察阈值为 100 ms，不代表控制周期或安全认证指标。任何关节启动后长期无状态反馈、过期/未来/乱序状态、CAN 错误、FIFO/队列异常、编码饱和、电机告警及意外运行/标定模式均锁存；后来新帧不能清故障。
- `om_usb.c` 增加 STATUS（5）/STATUS_REPLY（0x85），帧版本、CRC、最大载荷 64 字节和既有 PROBE/TICKET/VALIDATE/STOP 格式不变。查询回显主机的 sequence，不续租、不消费运动校验票据。
- CAN 观察故障对应 core fault 9，阻止票据/目标验证；仍允许 PROBE 获得启动 nonce 后读 STATUS。原 STOP/坏包故障仍锁存。USB 断开或传输故障仍终止传输，重连不恢复旧会话。
- `BenchClient.read_status(0..11 / 255)` 提供 Python 解码，检查 nonce、sequence 和载荷；始终返回 `motion_locked=True`、`hardware_adapter_ready=False`。参数捕获明确标 `verified_readback=False`。

BUS_MONITORING **不发 CAN 数据，也不 ACK**。只接板卡和一个被动/静默电机不一定有可观察反馈；台架必须有另一个正常 CAN 节点提供测试帧和 ACK。禁止为得到反馈偷偷启用电机、发送目标或打开主动上报配置。

队列时间戳为 HAL tick 下的 **ISR/任务收取时间**，不是硬件线缆时刻。FIFO 积压超过单次预算会锁存，不会被当作健康数据。100 ms 检查仅在调度器和 HAL tick 正常运行时成立；CPU 挂死/长时间关中断仍需独立硬件看门狗及台架验收。

## 厂家工程核对结果

厂家固定源：[`dmBots/DM-MC02@d65a368`](https://github.com/dmBots/DM-MC02/tree/d65a3688981405dc205f628f1a026bde8cf8303e/examples/CtrBoard-H7_ALL)。检查的是 `examples/CtrBoard-H7_ALL`，未改写厂家工作树。

| 检查项 | 当前源码事实 | 本次处理 / 剩余工作 |
|---|---|---|
| MCU/时钟 | STM32H723；HSE_VALUE=24 MHz；PLL M=2/N=40/P=1/Q=4；HSI48 已打开 | 据源码计算 SYSCLK 480 MHz、FDCAN PLL1Q 120 MHz，需在真实板卡读 RCC/测量确认，不能沿用 Keil UI 的 CLOCK(12000000) 标签作为实测值 |
| 三路 CAN 波特率 | nominal prescaler 24、TSEG1=2、TSEG2=2、SJW=1 | 据上述时钟推算 120 MHz/(24×5)=1 Mbit/s；本次保留位时序和引脚，控制频率尚未定 |
| CAN 引脚 | CAN1 PD0/PD1 AF9；CAN2 PB5/PB6 AF9；CAN3 PD12/PD13 AF5 | 不改厂家 MSP；实物收发器、接口编号、终端和共地待验 |
| Message RAM | offset 0、0x406、0x812（word）；原为每路 32 RX+32 TX、1 标准滤波器 | 保留 offset；新布局 1 个扩展过滤器+32×8字节 RX，占 130 word/路，最后终点 2196 word，小于 HAL 的 2560 word；TX 分配为零 |
| 原接收回调 | `Bsp/can_bsp.c` 仅保存 8 字节 payload；过滤仅对 hfdcan1 配置标准 ID | 独立只读目标排除 `can_bsp.c`，由本模块独占三个 HAL 接收/错误回调；不改厂家发送函数的 ABI |
| 原测试任务 | `App/fun_test_task.c` 会向三路发标准 ID 0x520 | 禁用整组测试/电机发送任务，不得连同本模块一起运行 |
| RNG | HAL v1.11.1；RNG 模块关闭，缺少四个 RNG 源/头文件，没有 MX_RNG_Init；ioc 显示 RNGFreq_Value 48 MHz | 编译脚本取固定官方 HAL `b716379...` 的四文件到独立 overlay；完整工程仍须启用 HAL_RNG_MODULE_ENABLED、配置 RNG kernel mux/外设时钟、初始化句柄并检查错误 |
| RNG 时钟选择 | 现有 HSI48 可作为候选，HAL 有 RCC_RNGCLKSOURCE_HSI48 | 完整目标需明确 RCC_PERIPHCLK_RNG + RngClockSelection、启用 RNG 外设时钟、HAL_RNG_Init/CED，再调用 USB init；不能用固定 nonce 或跳过错误检查；此接线尚未落为板级启动代码 |
| RTOS | `MX_FREERTOS_Init` 创建 default、Key、LCD、IMU、FunTest；default task 启动 USB；调度后 main while 不执行 | 需要单独 bench task 替代这些任务，先完成 RNG/协议/CAN 初始化，再启动 USB，循环 CAN poll → USB poll → 延时；本次没有把 poll 错放到 main while |
| 时基/中断 | RTOS 1 kHz；HAL tick=TIM23；FDCAN IRQ priority 5；max syscall priority 5 | 保留 TIM23/HAL_IncTick，回调不用 RTOS API；任务栈、高水位、抢占/阻塞上限需实测 |
| USB/DMA | USB HSI48；DMA disabled；CDC 静态缓冲 | 保留关闭 DMA；若以后开 DMA/cache，另验 RAM 可达性、对齐、cache 一致性 |
| 启动/链接 | 厂家有 Keil `.uvprojx`、ARMASM `startup_stm32h723xx.s`、CMSIS system、HAL/USB/FreeRTOS 源；没有本模块独立 GCC bench target/linker script | 不是“厂家没有工程”；缺的是明确排除运动逻辑并包含新模块的完整目标、对应 GCC 启动/链接/RTOS port/运行库配置与链接验收 |

## 完整可链接固件还缺什么

1. 独立 bench 目标（Keil 或 GCC/CMake），精确 source list；加入 5 个 OmindOS `.c`、生成 CDC 文件、官方 2 个 RNG `.c`，排除原 `can_bsp.c`、FunTest 和所有电机发送任务。所有对象使用一致 CPU/FPU/float ABI，不能把本次 soft-float 检查对象直接混入厂家的 hard-float 工程。
2. 与 STM32H723 Flash/RAM 区域及 bootloader 偏移一致的启动向量、链接脚本/工程存储配置；`.data/.bss`、栈堆、USB 缓冲区和 Message RAM 的 map 检查。厂家 Keil 启动不能直接作为 GNU assembler 输入。
3. 实际板级 RNG mux/clock/init 和句柄；RNG clock/seed error 的失败路径；冷启动/复位后 nonce 唯一性记录。
4. 板级 task/startup 接线，接入真实关节表和配置 digest，禁止使用测试夹具；HAL tick、RNG、USB、FDCAN IRQ 路由、单一 poll 所有者。
5. 真正链接所需 HAL FDCAN/RCC/GPIO/TIM/PCD、USB middleware/device handle、CMSIS/system、RTOS port/heap 和 C/数学/compiler runtime；唯一 HAL 回调定义，不能与原 BSP 冲突。
6. ELF 全符号解析、map/size/栈检查、hex/bin 生成、可恢复刷写流程，以及硬件 watchdog/停止输出验收。软件模拟不能替代这些项。

`check_dm_mc02_stm32_usb.py` 输出 JSON 列出当前对象组全部未解析外部符号，供完整 target 收敛。`-r` 仅完成对象间符号合并，不解析硬件 HAL/启动依赖。

## 集成顺序（工程接线要求，尚非可烧录示例）

1. 从固定厂家工程建立独立 bench target，保留原工程不变，断开电机动力或采用经检查的台架隔离方式。
2. 按厂家初始化 HAL、SystemClock、GPIO、FDCAN1/2/3（只 Init，不调用 `can_bsp_init`/Start），保持原引脚和位时序。
3. 配置并初始化 RNG；调用 `om_stm32_usb_init(&rng, actual_digest, actual_joints)`。失败时不启动 USB/CAN。初始化时 USB 端点/中断须未启动。
4. 用 `{&hfdcan1,&hfdcan2,&hfdcan3}` 调用 `om_stm32_can_init`。它将 READY 句柄重新 Init 为监听配置，并安装 STATUS provider；同次启动不能再次 init。失败时停止 CAN，若 USB core 已成功初始化，可保留 USB 诊断显示 init fault；绝不退回厂家发送任务。
5. 单一 RTOS 任务启动 USB，然后持续执行 `om_stm32_can_poll(); om_stm32_usb_poll(); osDelay(1);`。这是接线顺序，不保证实际 1 ms 调度。保留 USB 断连回调。任务要有经测量的栈空间，不能照搬厂家 defaultTask 的 128 word 预算。
6. USB 客户端先 PROBE 校验 digest，再读 255 号总线诊断和 0..11 关节；不能以任何 fresh 字段推断电机可使能。

## STATUS v1 载荷

所有多字节整数及 float32 为 little-endian；USB 外层沿用协议 v1。请求载荷为一个 selector 字节，nonce 来自本次 PROBE，sequence 为主机查询序号，deadline=0。回复原样回显 sequence/deadline，不改变板卡运动票据状态。未来客户端不能将新命令发送给未声明支持此扩展的旧固件（旧固件会锁存未知命令）；本次 Python 方法应只配套本增量台架模块使用。

| 字节位置 | 关节 selector 0..11 | 总线 selector 255 |
|---|---|---|
| 0..3 | schema=1、motion_locked=1、receiver_initialized、selector | 相同 |
| 4..7 | bus、motor ID、flags、该路锁存故障 | now_ms uint32 |
| 8..11 | any_latched_fault、mode、状态 fault、parameter error | timeout_ms uint32 |
| 12..15 | now_ms | 三路锁存故障 + 1 保留字节 |
| 16..23 | 状态 received_ms、sample_count | bus0 accepted、ignored |
| 24..39 | position、velocity、torque、temperature float32 | bus0 malformed/overflow、bus1 accepted/ignored |
| 40..47 | 类型21 fault、warning uint32 | bus1 malformed/overflow |
| 48..51 | parameter index uint16 + 2 保留字节 | bus2 accepted |
| 52..59 | parameter raw value、parameter received_ms | bus2 ignored、malformed |
| 60..63 | 本总线 malformed reports | bus2 overflow |

flags：bit0 状态收到，bit1 参数回复收到，bit2 故障帧收到，bit3 状态年龄<100 ms，bit4 参数年龄<100 ms。未收到时数值字段无效，Python 返回 None。fault bits：1 格式异常、2 溢出/积压、4 总线/HAL错误、8 超时、16 电机异常、32 初始化失败。计数饱和于 uint32 最大值；IRQ 合并的异常计为故障报告次数，不保证等于全部丢帧数量。

## 可复现软件验证

```sh
python3 -m unittest discover -s tests -p 'test_dm_mc02*.py' -v
python3 scripts/check_dm_mc02_stm32_usb.py --output-dir /tmp/dm-mc02-objects
```

依赖：Python 3、host gcc（含 ASan/UBSan）、Git、`gcc-arm-none-eabi` 与 newlib 头文件。可传 `--vendor-dir /path/to/DM-MC02 --hal-dir /path/to/stm32h7xx-hal-driver` 复用**固定提交且无 tracked 改动**的目录，无需重复下载。输出目录必须不存在；不修改厂家源码。

本次本地验证：42 项 unittest 全部通过。其中包含原 20 项协议测试、新 19 项反馈/USB 往返测试，以及 8 个 CDC、12 个 STM32 USB、39 个 STM32 CAN 模拟场景（后三组各作为一个 unittest）。CAN 模拟以 AddressSanitizer/UndefinedBehaviorSanitizer 运行；只关闭本环境无法使用的 LeakSanitizer，固件不分配堆内存。覆盖三路路由、FD/DLC/ID/remote 错帧、64字节错误 DLC 的缓冲安全、软件队列/FIFO溢出、bus-off/协议错误/HAL失败、每个初始化调用失败、未收/断流/过期/时间回绕、错误锁存和 PRIMASK 恢复。USB 原有断连、TX busy/超时、错误完成回调继续通过，新增 CAN fault 后诊断可读、USB 断连后诊断不再发送。

GCC ARM 13.2.1：8 个 Cortex-M7 对象通过 `-Wall -Wextra -Werror`，并完成可重定位链接；无未解析 OmindOS 内部符号，也无 CAN 发送函数引用。官方 RNG 源码仅作为匹配依赖参与编译，不代表 RNG 时钟已上板验证。CI 保存 `dm-mc02-software-validation` 的 validation.json；软件证据见 `docs/validation/dm-mc02-can-rx-20261008.json`。

## 首次硬件验收（全部尚待完成）

| 顺序 | 接线与操作 | 需要保留的证据 |
|---|---|---|
| 1 | 核对板卡型号/revision、RS04 固件/协议模式/ID，保留厂家备份及恢复手段；电机不使能 | 照片、版本与配置清单、可回退固件标识 |
| 2 | 先仅给 DM-MC02 上电，验 HSE/PLL/RNG 时钟与启动状态，验证 HAL tick 和独立任务运行 | RCC/错误码、tick、栈高水位、不同启动 nonce；RNG 故障时不启动会话 |
| 3 | Jetson USB 枚举、PROBE digest、逐项 STATUS；拔插、端点错误、忙/超时、反复断开 | USB 抓包及时间戳；重连不恢复旧会话；未收反馈正确显示 |
| 4 | CAN 分路、先无动力电机/测试节点；核对 GND/CAN_H/CAN_L、两端终端、收发器供电与使能电平 | 三路物理接口对照、示波器/分析仪波形、实际 nominal bitrate；确认板卡监听无 TX/ACK |
| 5 | 用带 ACK 的台架节点注入每路4个 ID 的已知帧、错误帧、突发/溢出、断流 | USB 与 CAN 抓包逐字节比对、bus/motor 路由、单位、时间戳及各故障锁存 |
| 6 | 再接单个保持不使能的 RS04；通过经过检查的外部工具进行只读查询；逐路扩到12个 | 原始 RS04 状态/参数回复与 USB 对照；型号、模式、参数含义实证。被动捕获不等于完成请求—应答参数回读验收 |
| 7 | 故障拔线、掉电、CAN bus-off、Jetson退出、USB/CAN压力和任务延迟测试 | 独立 watchdog/停止输出行为与响应时间；没有任何运动/使能帧 |
| 8 | 最后按实际最坏帧长、反馈/参数流量、总线占用、ISR/RTOS/USB抖动确定周期 | 三路带宽核算、实测 p95/p99/最坏延时与余量；暂不承诺控制频率 |

以上通过后也仅能确认非使能通信链路。电机使能、单关节标定/低功率运动、12关节同步和整机步态须另行验收，本增量没有实现这些入口。
