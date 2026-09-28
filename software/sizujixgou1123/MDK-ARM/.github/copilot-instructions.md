# Copilot 使用说明（仓库专用）

简短说明：此仓库为同轴 5 连杆四足机器人工程（STM32F4 + HAL + FreeRTOS，Keil MDK 构建），电机为小米 CyberGear。该文档给 AI 代理可直接使用的要点与查找位置。

- **总体架构（大局）**
  - MCU 平台：STM32F4（启动文件 `startup_stm32f427xx.s`）；RTOS/组件由 `sizujixgou1123/` 下 RTE 配置管理。
  - 硬件接口层：`CyberGear/`（`cybergear.h` / `cybergear.c`）封装了 CAN 报文与电机控制、运动学实现。
  - 高层策略/序列：`robot_config/`（如 `leg0_config.c`）实现单腿动作、校准与轨迹生成，调用 `CyberGear` API。
  - 输入层：`SBUS/` 负责遥控数据解析并上抛为高层命令。

- **关键文件（优先查看）**
  - `CyberGear/cybergear.c`, `CyberGear/cybergear.h` —— CAN 报文、命令编码、正/逆运动学。
  - `robot_config/leg0_config.c` —— 单腿初始化、零点校准与轨迹示例。
  - `SBUS/sbus.c` —— RC 输入解析。
  - `sizujixgou1123/*.uvprojx`, `DebugConfig/` —— Keil 项目与调试配置。

- **CAN 报文与协议（务必核对）**
  - ExtId 格式在代码中通过位移硬编码构造，例如使能：`(0x03 << 24) | (0x00FD << 8) | motor_id`。
  - 位置命令实现中把 float 通过指针转为 32-bit 再写入 `tx_data[4..7]`（代码将 MSB 写入 `tx_data[4]`）；注释声称“小端”，但实现表现为 MSB-first。不要更改字节序，除非同时验证接收端或有回归测试。
  - 避免重复实现 ExtId 位域逻辑：复用 `CyberGear_*` 封装函数。

- **运动学与状态管理（容易出错）**
  - `Kinematic_Inverse_Update` 处理角度跳变（2π 环绕），受 `param->initialized`、`last_a1`、`true_a1` 控制。修改运动学或角度处理时，保证初始化路径和连续性测试（±2π 场景）。

- **项目特有约定 / 风格提示**
  - API 前缀样式：硬件层用 `CyberGear_*`，高层动作用 `Leg0_*` / `Leg_*`。新增函数请参考对应层级前缀。
  - 硬件假设：代码中大量使用 `HAL_Delay`、`hcan1` 推定单 CAN 总线；若改为多线程或高频控制，应替换阻塞延时与增加互斥保护。
  - ID 映射注意：代码注释中对 α/β 电机 ID 的描述与 `CyberGear_Init` 的实际调用存在不一致（请在硬件接入前确认 `CAN_ID` 映射）。

- **AI 代理操做建议（可直接执行的规则）**
  - 修改 CAN/电机逻辑：先在 `CyberGear` 内实现并注释 ExtId 与字节序示例，保持上层不变。
  - 修改运动学/轨迹：检查并保留 `param.initialized` 相关逻辑；在变更后添加单元级示例（例如用已知姿态验证正/逆解一致性）。
  - 增加日志/自检：在 `robot_config/leg0_config.c` 的初始化路径插入状态检查函数（读取 motor.error_code 并返回详细错误），以便在硬件上快速排查。

- **构建 / 调试简要流程**
  - 在开发机上用 Keil uVision 打开 `sizujixgou1123.uvprojx`，Build -> Flash。输出文件在 `sizujixgou1123/`。
  - 调试使用 `DebugConfig/` 中的 dbgconf，硬件以 ST-Link 或 J-Link 连接 MCU。

如需把 CAN 报文逐字节详列或生成 Keil CLI 构建脚本/CI 草案，我可以继续补充。
