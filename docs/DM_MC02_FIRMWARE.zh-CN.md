# DM-MC02 禁使能台架固件：完整 ARM GCC 构建

2026-10-09。本目标复用已合并的通信/反馈模块和固定厂家工程，完成完整可执行文件链接。
它是 **只读观察固件候选**；没有刷写、USB 实板枚举、CAN 实板接收或整机验收记录。

| 阶段 | 本次状态 | 证据与限制 |
|---|---|---|
| 软件编译与完整链接 | 已通过 | ARM GCC 13.2.1；44 个对象；无未解析符号；47 项主机单元测试通过；179 个向量逐项检查 |
| 烧录格式生成 | 已通过 | ELF/HEX/BIN、SHA256、map/readelf/size/stack-usage；BIN 31,392 字节。烧录基址预设为 0x08000000，尚未核实实物 bootloader/option bytes |
| 真实硬件验收 | 未进行 | 所有启动、时钟、RNG、USB、CAN、栈余量和故障时序结果均待实板测量；不能称实机可用 |

## 独立目标与只读边界

- `scripts/build_dm_mc02_bench.py` 精确列出源码；厂家目录只读，不修改 Keil、HAL、USB 或原应用文件。变换仅写入新输出目录的 overlay。
- 厂家固定版本 `dmBots/DM-MC02@d65a3688981405dc205f628f1a026bde8cf8303e`，目标目录 `examples/CtrBoard-H7_ALL`。
- 仅从官方 HAL `b716379524ba3549e4db11804889776ffeb239d1` 取现有匹配的四个 RNG 源/头文件；不混入整个新 HAL。
- 使用厂家 FreeRTOS 10.3.1 kernel，补入官方 FreeRTOS GCC CM7/r0p1 port，固定提交 `88e32327e975ddde97c390bc5b6c1f8e7d9d239e`。不使用厂家 Keil/RVDS 汇编 port。
- 排除厂家 `main.c`、`freertos.c`、`stm32h7xx_it.c`、`gpio.c`、`can_bsp.c`、所有电机驱动/发送/测试任务，以及按键、LCD、IMU、PWM、串口和外部 Flash 测试。仅保留 HAL、FDCAN MSP、TIM23、USB middleware/device 和 CMSIS system 等必需源码。
- 三路 FDCAN **从第一次 HAL 初始化起** 即 BUS_MONITORING，TX FIFO/Buffer/Event 为零。无数据发送，也不 ACK；不能把它当成唯一确认节点。
- `OM_BENCH_OBSERVER_ONLY` 使固件拒绝 TICKET/VALIDATE，最终 ELF 排除 `om_encode_targets` 和传统标定初始化 `om_init`。PROBE/STATUS 与 STOP 保留，STOP 仅锁存本机软件状态，不发送停止电机帧。
- `observer-routing.json` 只有三路、每路四个预期 ID，明确 `routing_verified_on_hardware=false`。不填写假限位、方向、减速比、零位或增益。这份表不是已验收整机配置；接电机前须确认并按实际 ID 重建。
- PROBE digest 为配置 JSON **原始字节** 的 SHA256；默认 `c8414196769758edad1a4c71ee2bb88c593754ef5677d194cf176a1887cd4cad`。接收数值仍为电机输出轴单位，未做机器人关节坐标标定。

## 启动与任务接线

1. GNU startup 按固定厂家 ARMASM 的原始 179 项向量顺序生成，设置 MSP，调用 CMSIS SystemInit，复制 `.data`、清零 `.bss` 后进入 main。只有 C 源，无需 C++ 构造器。
2. main 设置 VTOR，HAL_Init，沿用厂家 HSE=24 MHz 与 PLL 配置，然后三路只读 FDCAN Init。**不启动未知 GPIO 电源开关，不假定收发器供电/使能极性**。
3. 创建唯一静态 bench task，再启动 FreeRTOS；原厂家应用任务不运行。SYSCLK 按源码推算 480 MHz、FDCAN kernel 120 MHz、nominal CAN 1 Mbit/s，全部尚待实测。
4. task 显式配置 RNG kernel=HSI48，开启 RNG 外设时钟，设置 CED，初始化并读取四个 RNG word 构造 boot nonce。任何 RNG/nonce 初始化失败均不启动 USB 会话；无固定 nonce 或降级随机源。
5. 初始化观察协议和 STATUS provider，设置三路接收回调，再启动 USB。CAN init 失败时锁存，可保留 USB 诊断；不回到任何厂家发送任务。
6. 单一任务循环 `om_stm32_can_poll()` → `om_stm32_usb_poll()` → `vTaskDelay(1 tick)`。1 kHz tick/请求延时不等于已保证的 1 ms 执行周期。
7. FDCAN 6 个中断、USB OTG_HS、TIM23 均绑定唯一强定义；SVC/PendSV/SysTick 由 GCC FreeRTOS port 独占。CAN/USB ISR 不调用 RTOS API，优先级沿用 5。
8. HAL 时基保持 TIM23，并在独立副本中修正启动时 APB2 DIV1 的计数频率计算。USB DMA、LPM 均关闭：厂家设备配置 LPM=1 与底层 PCD LPM=DISABLE/空 BOS 回调不一致，本目标的配置副本统一为 0。

`om_bench_debug` 可通过 SWD 读取 phase、failure、rng_error、loops、hal_ms、stack_free_words。
错误/assert/栈溢出进入禁中断停机；本目标 **没有独立硬件 watchdog 实现或验收**，因此首次台架必须依靠已确认的物理动力隔离。软件 STOP 不能替代硬件急停。

## 链接和内存核查

所有对象统一 `cortex-m7/thumb/fpv5-d16/hard-float`，不混入此前 soft-float 的可重定位检查对象。

| 区域 | 布局 | 实际占用/预留 |
|---|---|---|
| Flash | 0x08000000，1 MiB，直接 SWD boot 假设 | BIN 31,392 字节；含向量、代码和 `.data` 的 Flash 装载映像 |
| AXI RAM | 0x24000000，320 KiB | `.data` + `.bss` 共 29,128 字节，约 8.89% |
| MSP | DTCM 顶部 0x2001E000–0x20020000 | 明确预留 8 KiB；其余 DTCM 未分配 |
| bench / idle 任务 | 静态 AXI RAM 分配 | 16 KiB / 2 KiB，已含于上述 `.bss` |
| FDCAN Message RAM | 外设 RAM，offset 0 / 0x406 / 0x812 word | 每路 130 word；最后结束 2196 word，小于 2560 word；不占上述 CPU RAM |

- 链接器拒绝重复强定义、RAM/Flash 溢出与 ABI 冲突；`nm -u` 必须为空。
- 自动核对 Reset/main/RNG 与 11 个实际 IRQ 的强定义，逐字验证 BIN 中 179 个向量及初始 MSP；检查 Flash LMA 和 BIN 大小对应。
- 最终 ELF 不含 CAN enqueue/transmit、厂家任务、目标编码器、传统标定初始化、动态 RTOS/newlib heap 入口。USB 的 `USBD_static_malloc` 是厂家静态 CDC 缓冲，不是动态堆分配。
- 保存 `.su` 静态函数栈报告，保留函数中最大单函数帧 248 字节；**这不是完整调用链/中断嵌套最坏栈证明**。必须在实板压力场景测 bench/idle 栈高水位和 MSP 使用。
- 厂家未使用的 HAL 睡眠、Flash 编程与 EXTI 函数产生 5 个 unused-parameter warning，保留于编译日志；这些函数已由 `--gc-sections` 丢弃。OmindOS 自有源码使用 `-Werror`。没有以压掉全部警告冒充无警告构建。

## 可复现构建

Ubuntu 24.04 的依赖：Python 3、Git、host GCC、ARM GCC/newlib。
本次工具链：gcc-arm-none-eabi `15:13.2.rel1-2`，binutils-arm-none-eabi `2.42-1ubuntu1+23`，newlib `4.4.0.20231231-2`。

```sh
sudo apt-get update
sudo apt-get install -y --no-install-recommends gcc-arm-none-eabi libnewlib-arm-none-eabi
python3 -m unittest discover -s tests -p 'test_dm_mc02*.py' -v
python3 scripts/build_dm_mc02_bench.py --output-dir /tmp/omindos-bench-a
python3 scripts/build_dm_mc02_bench.py --output-dir /tmp/omindos-bench-b
cmp /tmp/omindos-bench-a/dm-mc02-observer.bin /tmp/omindos-bench-b/dm-mc02-observer.bin
cmp /tmp/omindos-bench-a/dm-mc02-observer.hex /tmp/omindos-bench-b/dm-mc02-observer.hex
cd /tmp/omindos-bench-a
sha256sum -c SHA256SUMS
```

脚本首次浅取固定公共依赖到 `/tmp`，跳过无关 CAD/media LFS 文件；也可使用
`--vendor-dir`、`--hal-dir`、`--freertos-dir` 指定已有固定提交的干净 tracked 工作树。
所有源码路径和输出路径应无空白；输出目录必须不存在，失败输出不要当成通过产物。
修改预期 ID 时使用 `--config /path/to/observer-routing.json`，digest 随原始 JSON 字节改变。
工具链不同可能改变 SHA256；应按新输出证据验收，不套用本次 checksum。

产物：ELF/HEX/BIN、SHA256SUMS、validation.json、map、readelf、size、完整命令、编译依赖/source SHA256、静态栈报告和 overlay。
CI `.github/workflows/dm-mc02-bench.yml` 运行相同测试、完整构建及 BIN/HEX 重建比较，上传私有候选 artifact；不自动发布或刷板。

## 首次安全上板的具体前提和顺序

1. **板卡与恢复**：核对 DM-MC02 revision、实际 STM32 型号/Flash/RAM容量、24 MHz HSE，确认 SWD 接口/电平/NRST及 ST-Link 能连接。备份厂家 Flash、option bytes、bootloader 和当前设置；备份读回 hash、恢复流程经过核对。不要自动解除读保护或执行全片擦除。
2. **启动布局**：核实本板应用入口是否就是 0x08000000。若有 bootloader/保留区，不可直接刷本包；先调整 linker、VTOR/boot 流程并重新构建。确认 ELF/HEX/BIN hash 与选定版本一致。
3. **物理隔离**：首次仅给控制板受控限流供电，断开全部电机动力/输出连接；硬件急停和独立断电手段可用。核对板内电源开关与 CAN 收发器供电/STB 极性，本目标没有代为打开未知电源脚。
4. **先验证启动**：SWD 核对 phase=8、RCC/HSE/PLL/RNG mux、TIM23 时间间隔、FreeRTOS tick/循环和两个任务栈余量；记录多次冷启动/复位的 nonce。验证 RNG 错误/全零失败路径没有有效会话。
5. **再验证 USB**：与 Jetson/PC 枚举、PROBE digest 和 0..11/255 STATUS 抓包；验证坏 CRC、STOP、TICKET/VALIDATE 拒绝及拔插/忙/超时锁存。确认重连不恢复旧会话，需复位开启新会话。
6. **CAN 测试节点先行**：逐路确认 CAN_H/L/GND、两端终端、收发器电源/使能、1 Mbit/s 实测；使用至少一个正常 ACK 节点与 CAN 分析仪注入已知状态/参数/故障帧，观察本板无数据 TX/ACK。覆盖路由、断流、bus-off、突发溢出与错误锁存。
7. **最后接单个禁使能 RS04**：确认型号、固件/私有29位协议模式、实际ID，再通过经过检查的外部只读工具获得反馈；不能为看到帧从本固件加入使能/目标/主动上报写指令。记录原始帧与 USB 数值、单位及参数含义，逐路扩到12个。

若后续需要允许动力连接、无人值守或运动控制，独立 watchdog、硬件急停、发送/使能协议、参数/标定、单关节低功率与整机稳定性必须另外实现并验收；本包不提供这些功能。
