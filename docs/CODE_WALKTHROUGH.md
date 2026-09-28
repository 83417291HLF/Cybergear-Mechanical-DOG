# 从代码拆解 Cybergear Mechanical DOG

分析对象为 `software/sizujixgou1123`。相对路径以下均以该目录为起点。分析依据为源码、函数调用和 `.uvprojx` 源文件清单，不以旧编译产物推断当前功能。

## 1. 入口与执行时序

[`Core/Src/main.c`](../software/sizujixgou1123/Core/Src/main.c) 的执行顺序：

```text
HAL_Init → SystemClock_Config
→ GPIO / DMA / CAN1 / CAN2 / USART1 / UART7 / TIM6 / TIM7 / SPI5 初始化
→ can_bsp_init
→ USART1 ReceiveToIdle DMA → 启动 TIM6 中断
→ mpu_device_init → quadruped_init
→ while (1)
    每当 HAL_GetTick 差值 ≥ 10 ms：
        mpu_get_data
        update_control_from_remote
        update_quadruped_gait(0.01)
    每 500 ms 翻转 LED
```

当前调度是裸机轮询。TIM2 回调负责 `HAL_IncTick()`；TIM6 虽然启动，但 `HAL_TIM_PeriodElapsedCallback()` 未实现 TIM6 的控制任务。旧输出目录中的 `freertos.o`、`tasks.o` 不代表当前主程序在运行 RTOS。

这里的 100 Hz 是目标频率：`last_tick = now`，传入的 `dt` 固定为 0.01。发送电机命令的阻塞延时和传感器读取可能造成超期，超期后没有追赶执行或真实时间步长修正。

## 2. 遥控输入层

[`MDK-ARM/SBUS/sbus.c`](../software/sizujixgou1123/MDK-ARM/SBUS/sbus.c) 在 USART1 的 IDLE/DMA 回调中解析 SBUS 数据：检查首字节 `0x0F`、尾字节 `0x00`，从 11-bit 打包数据提取 10 通道，统一减去 992。

[`update_control_from_remote()`](../software/sizujixgou1123/MDK-ARM/leg/leg.c) 将通道转换为步态参数：前后步幅 `S`、转向步幅 `yaw_s`、踏步标志、抬腿高度 `H`、周期 `T_cycle` 和基准腿高。步幅采用 `0.8 × 旧值 + 0.2 × 目标值` 平滑；它们是毫米尺度的轨迹幅度，不是 m/s 的速度命令。

CH5 仅控制前后/转向，CH6 独立触发原地踏步。`side_s` 虽然声明在 `GaitConfig`，当前没有进入轨迹生成。

## 3. 足端轨迹和机身姿态补偿

主要逻辑集中在 [`MDK-ARM/leg/leg.c`](../software/sizujixgou1123/MDK-ARM/leg/leg.c)。

每条腿的步幅：左侧 `S + yaw_s`，右侧 `S - yaw_s`。相位表达式为：

```c
phase = timer / T_cycle + ((i == 1 || i == 3) ? 0.5f : 0.0f);
```

因此是 0/2 同相、1/3 同相，配合 LF/LH/RH/RF 腿序形成对角配对。旧注释所写的“0、3 同相”与表达式不同。

摆动相占半周期，归一化时间为 `s`，`sigma = 2πs`：

```text
x = -步幅/2 + 步幅 × (sigma - sin(sigma)) / (2π)
y = y_base - H × (1 - cos(sigma)) / 2
```

支撑相令 x 从正半步幅线性回到负半步幅，y 保持基准高度并叠加补偿。停止行走时 timer 清零，各腿 x = 0，但仍更新姿态补偿和电机位置。

姿态链路为 `mpu_get_data → IMUupdate1 → imu.pit0/rol0 → 腿高补偿`。每轴按 2 mm/° 放大，经过系数 0.4 的低通及 20 mm/s 变化率限制，再按前后、左右腿加减；每腿合成补偿限制为 ±100 mm。行走时只在支撑相补偿，静止时四腿均补偿。

`y_base += (target_y_base - y_base) * 1.0f` 实际一步到达目标，不具备注释暗示的渐变效果。IMU 反馈单位是度，几何 IK 和电机目标使用弧度，二者不可混用。

## 4. 单腿 IK 与关节补偿

[`leg_set_position_idx()`](../software/sizujixgou1123/MDK-ARM/leg/leg.c) 使用 `L1 = 100`、`L2_EFF = 200 + 45 = 245` mm 和 π/2 偏置。

先做左右镜像，再以足端距离 L、方向角 psi 和余弦定理求 phi：

```text
L² = cx² + cy²
psi = asin(-cx/L)
phi = acos((L² + L1² - L2_EFF²)/(2 L1 L))
pos1 = -(phi - psi) + π/2
pos2 = -(phi + psi) + π/2
```

这描述源码采用的几何模型；完整并联连杆闭环、装配分支和实际零位仍须与机械图纸及单腿测量对照。

附加力矩公式是 `tau = stiffness × (目标角 - 反馈角) - 0.1 × 反馈速度`。刚度为 `2 × clamp(213.2/y, 0.8, 1.5)`，力矩限幅 ±12，位置在镜像前限幅 −1.2～1.4 rad，最终传入 `motor_controlmode(..., 13, 0.9)`。右侧位置和力矩发送前取负。

因此当前是**位置轨迹 + 关节空间弹簧阻尼补偿 + 电机内部 PD**。足端力通过 `JᵀF` 分配的实验性 VMC 没有进入这条运行链路。

## 5. CAN 与电机协议层

[`CyberGear/bsp_CAN.c`](../software/sizujixgou1123/MDK-ARM/CyberGear/bsp_CAN.c) 启动两条总线，用过滤器组 0、14 接收 FIFO0 中断，当前掩码放行全部 ID。

[`CyberGear/cybergear.c`](../software/sizujixgou1123/MDK-ARM/CyberGear/cybergear.c) 封装使能、停止、设零位、参数写入、运控和反馈解码。运控帧将位置、速度、Kp、Kd 各编码为 16 位并填入 8 字节载荷，力矩编码进入扩展 ID。主机 ID 定义为 `0xFD`。

`can_txd(hcan)` 是头文件宏，展开为 `HAL_CAN_AddTxMessage(hcan, &txMsg, tx_data, &send_mail_box)`，所以函数内的局部 `tx_data` 会实际传入 HAL，并非丢失载荷。当前真正的问题是 `motor_controlmode()` 在等待邮箱前后调用该宏两次，存在重复发送和首发失败未处理的问题。

接收回调根据总线及电机 ID 分发给 `mi_motor_can1/2[0..3]`，更新角度、速度、力矩、温度和错误位。数组实际声明容量为 5，每条总线只初始化 4 个元素。

## 6. 当前目标之外的文件

`.uvprojx` 中的主要业务源文件是 `leg.c`、`cybergear.c`、`bsp_CAN.c`、`sbus.c`、`mpu6500.c`、`process.c`、`mahony.c`、`ist8310.c`、`kalman.c`。加入工程不意味着所有函数均被调用：姿态实际入口是 `IMUupdate1()`。

| 文件 | 检查结果 |
| --- | --- |
| `leg/quadruped_vmc.c/.h` | 未加入当前编译目标；尝试足端弹簧力与 Jacobian 转置控制 |
| `leg/gait_planner.c` | 未加入目标；包含不存在的 `leg0_config.h`，依赖 `leg0` |
| `leg/robot_config.c` | 当前为空文件，不能视作集中参数管理已经实现 |
| `RTE/jump.c/.h` | 未加入当前编译目标 |
| `dm_IMU.c`、`dm_imu.h`、`SBUS/dm_imu.*`、`imu.c` | 当前工程未选用的其他 IMU 相关文件 |

VMC 实验代码的头文件写 12 kg，而 `.c` 内定义 10 kg；足端误差直接使用毫米，刚度注释为 N/m，尚未统一单位。其 Jacobian 表达式也需要按真实并联机构闭环重新推导，不能直接视作验证过的整机力控。

## 7. 已确认的问题与待验证项

以下为静态审查结果，本次归档未修改控制源码。优先级表示修复顺序，不表示实机已复现。

| 优先级 | 位置与证据 | 影响及处理方向 |
| --- | --- | --- |
| P0 | `sbus.h` 定义 `BUFF_SIZE=25`、`sbus.c` 分配 25 字节，但 main 与回调都启动 `BUFF_SIZE*2` 的 DMA | DMA 可写到缓冲区之外；统一容量与长度，在完整帧校验后解析 |
| P0 | `update_control_from_remote()` 的离线分支只清零步幅，没有 return；后续读取旧通道 | 旧通道可以重新恢复运动；离线必须终止控制输入处理并清理踏步状态 |
| P0 | `sbus_frame_parse()` 仅用 `buf[23] == 0x0C` 判离线，无接收超时 | 断线不再收帧时可一直保持在线；应按位检查故障标志并加入有效帧时间戳 |
| P1 | `Motor_Data_Handler()` 用 `MIN_P=-720/MAX_P=720` 解码 Angle，IK 目标直接以 rad 与它相减 | 反馈与控制数值尺度不一致；统一单位后重新验证补偿，不可只调增益 |
| P1 | `motor_controlmode()` 连续两处 `can_txd`，返回值未检查，末尾 `HAL_Delay(1)` | 命令重复、丢帧不可见、周期预算被占用；改为单次受控发送并统计失败 |
| P1 | SBUS 回调在解析前重启 DMA，且允许 `Size < 25` 时进入解析 | 读写竞争及残留数据可能被当作完整帧；增加帧长验证、快照/双缓冲和重同步 |
| P1 | `check_and_recover_motors()` 遍历 CAN1 的 0..7，数组容量仅 5，且遗漏 CAN2 | 潜在越界；当前主循环未调用，接入前必须修复为两总线各 4 个，不能自动无条件恢复 |
| P1 | 右侧发送目标取反，但误差计算直接使用未转换的右侧反馈 | 即便修正角度单位仍可能符号错误；定义统一关节坐标，反馈与目标采用同一转换 |
| P1 | IK 限制 L 后仍在余弦公式中使用原始 L_sq | 越界足端目标没有被一致投影到可达域；同时调整坐标/距离及平方项 |
| P1 | `KalmanFilter()` 用单份 static 状态，`IMUupdate1()` 对 pitch/roll/yaw 连续调用 | 三轴滤波互相污染；每轴使用独立状态 |
| P1 | `IMUupdate1()` 对加速度、磁场和四元数模长直接相除 | 零/无效测量可传播 NaN；增加有效性门限，首帧时间初始化及异常恢复 |
| P1 | `quadruped_init()` 上电即使能、设零位、发零角命令 | 任意上电姿态可能改变机械参考；拆分标定与运行状态 |
| P2 | 高度调整系数是 1，固定 dt，CAN 反馈无帧类型/DLC/时效完整检查 | 高度跃变、时序误差和无效反馈风险；增加速率限制、实际周期统计及反馈验证 |

推荐先修复内存与失联问题，再统一单位和坐标，随后验证时序及 IMU，最后开展 VMC。每一步都应保留单腿/整机测量记录。

## 8. 验证边界

本次属于代码阅读、工程清单核对与文档整理；没有用历史构建日志代替当前编译，没有宣称机器人已达到某个速度、负载、跳跃高度或竞赛成绩。编译与硬件测试步骤见[上机说明](BUILD_AND_BRINGUP.md)。
