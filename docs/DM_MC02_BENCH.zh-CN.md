# Jetson USB—DM-MC02 三路 CAN：首轮开发与联调

连接方案固定为 Jetson → USB CDC → DM-MC02 → 三路经典 CAN → RS04。
12 个关节先按每路 4 个规划；CAN 地址、方向、零点、外部传动比与实机控制频率均需现场确认。

## 本次实现

`scripts/dm_mc02_protocol.py` 与 `firmware/dm_mc02/om_usb.c` 实现相同的 USB 帧格式、
CRC32、整批目标校验、重放拒绝、启动会话和 MCU 时钟上的绝对有效期。
板卡签发单次 ticket 后，主机才生成新目标；旧包到达时不会重新起算有效期。
ticket 的 50 ms 是软件测试期限，不是已验收控制频率。每次 ticket 请求多一次往返，
需要实测 USB 调度与 CAN 负载后再决定是否采用流水线或时钟同步的实时方案。

帧为小端：`OM(2) | version(1) | kind(1) | length(2) | boot_nonce(16) |
sequence(4) | deadline_ms(4) | payload(0..64) | CRC32(4)`。
CRC 使用 IEEE CRC32，与 Python `zlib.crc32` 一致，不提供身份认证。
目标 payload 为固定配置顺序的十二个 IEEE float32 关节弧度；配置 digest 为
板卡校准配置规范化文件的 SHA256，主机必须逐字节匹配。

| 请求 | 返回 | 用途 |
| --- | --- | --- |
| PROBE=1 | 0x81，锁定标志与配置 digest | 启动会话、配置检查 |
| TICKET=2 | 0x82，启动 nonce、递增序号及 MCU 到期时刻 | 单次有效校验授权 |
| VALIDATE=3 | 0x83，锁定标志 | 校验十二关节并试编码，不发送 CAN |
| STOP=4 | 0xff，原因 7 | 锁存测试会话停止 |
| 错误 | 0xff，原因 1..8 | 配置、报文、会话、重放、超时、目标、停止、断线 |

MCU tick 回绕由模 2^32 差处理；有效期限定为 50 ms，独立调度器必须持续调用 `om_tick`。
USB 重连、PROBE、新 ticket 都不能清除锁存错误；重新初始化须是一次受控板卡复位，
并从板卡 RNG 取得全新非零 boot nonce。接收溢出、断线、CAN 错误需调用 `om_fault`。

RS04 编码器使用 32 位 ID，生成 29 位扩展帧、8 字节数据；三路 bus 索引为 0/1/2。
浮点方向、零点及外部传动比映射后，编码位置、零目标速度、Kp、Kd 与零前馈力矩。
每批十二关节全部合法才返回报文数组。本次测试的 Kp/Kd、零点、限位与地址是
**合成测试夹具**，不是用户机器人配置，也不作为厂家推荐参数。

## 当前边界

这是可编译的便携 C 通信核心和 Python 主机客户端，**尚非可刷写的完整 DM-MC02 固件**。
没有 ARM 指令、CAN 发送接口或运动后端；`hardware_adapter_ready=false`。
未接入 STM32H723 启动文件、USB CDC 中断/队列、HAL FDCAN、RNG、定时器、
RS04 参数回读与使能、反馈回传、板卡 IMU 和独立停止输出。
不会自动启动原版 MEVIUS2 策略驱动客户 RS04。

外设集成时，采用厂商固定提交 `d65a3688981405dc205f628f1a026bde8cf8303e`。
CDC 接收回调只将字节写入有界队列；主循环解析，发送回应前处理 USB_BUSY 与超时。
三路 FDCAN 配置为经典 CAN、扩展 ID、DLC=8、BRS 关闭，实际波特率先读取核对。
初始化必须保持电机禁用；硬件停止输出、参数回读及反馈门禁通过后才增加独立使能路径。
CAN 编码不等于已证明报文能在实际总线可靠收发。

## ARM64 构建

`.github/workflows/arm64-dm-mc02.yml` 在原生 ARM64 runner 中构建 ROS 2 Humble、
Python 3.10、CPU PyTorch、MuJoCo 和二十个编译业务模块，检查 ELF AArch64、模块导入、
ROS 导入、CPU 运算、MuJoCo 一步，以及业务源码/客户私有记录未进入最终镜像。
候选镜像和检查报告保存在私有 Actions artifact，不自动发布公开 Release。
构建成功仍不代表 Jetson 实测完成；CUDA、TensorRT、JetPack 与载板 USB 配置未验收。
原 x86_64 preview.5 发行包保持不变，ARM64 的客户启动器与完整安装验收尚待补齐。

## 现场验收顺序

1. 核对 JetPack/Ubuntu 版本、载板、供电、急停、三路线束/终端电阻、RS04 固件与 CAN 参数。
2. 烧录已完成外设集成的禁用固件，仅做 USB 握手、配置 digest 和 CAN 接收验证。
3. 逐个核对 motor ID、关节名称、方向、零点、传动比和限位，记录真实校准参数。
4. 单关节台架核对参数回读、通信看门狗、断线/超时/急停停止时间、温度与故障反馈。
5. 测三路四关节的带宽、延迟和抖动，再确定控制频率；最后进入悬吊整机与步态/平衡验收。

当前没有连接 Jetson、DM-MC02 或 RS04，未发送硬件报文，实机联调未完成。
