#include "leg.h"
#include <stdlib.h>
#include "process.h" 
#include "mpu6500.h"// 里面定义了 imu 结构体
extern imu_t imu;      // 姿态数据
// ================== 全局步态参数初始化 ==================
GaitConfig myGait = {
    .S = 0.0f,
    .side_s = 0.0f,
    .yaw_s = 0.0f,
    .H = 60.0f,
    .T_cycle = 0.2f,
    .y_base = 213.2f,
    .timer = 0.0f,
    .stepping_in_place = 0
};

static float target_y_base = 213.2f;

// ================== 姿态自适应补偿滤波器状态 ==================
static float comp_pitch_lpf = 0.0f;   // 俯仰补偿量平滑值 (mm)
static float comp_roll_lpf  = 0.0f;   // 横滚补偿量平滑值 (mm)
static float last_comp_pitch = 0.0f;  // 用于速率限制
static float last_comp_roll  = 0.0f;
// ================== 工具 ==================
static inline float clamp_f(float v, float min, float max)
{
    return (v < min) ? min : ((v > max) ? max : v);
}

void quadruped_init(void)
{
    HAL_Delay(300);

    // 初始化 CAN1 电机 (ID 1-4)
    for (uint8_t i = 0; i < 4; i++) {
			 
        init_cybergear(&mi_motor_can1[i], i + 1, Motion_mode, &hcan1);  // 运控模式
        HAL_Delay(300);
        start_cybergear(&mi_motor_can1[i]);
			  Set_Cyber_ZeroPos(&mi_motor_can1[i]);  // 可选
        HAL_Delay(100);
        //限速、限流、Kp
        Set_Motor_Parameter(&mi_motor_can1[i], Limit_Spd, 30.0f, 'f');
        Set_Motor_Parameter(&mi_motor_can1[i], Limit_Cur, 12.0f, 'f');
        Set_Motor_Parameter(&mi_motor_can1[i], Loc_Kp, 15.0f, 'f');
        motor_controlmode(&mi_motor_can1[i], 0.0f, 0.0f, 0.0f, 8.0f, 0.5f);
        HAL_Delay(5);
        
    }
    for (uint8_t i = 0; i < 4; i++) {
			 
        init_cybergear(&mi_motor_can2[i], i + 5, Motion_mode, &hcan2);  // 运控模式
        HAL_Delay(300);
        start_cybergear(&mi_motor_can2[i]);
			  Set_Cyber_ZeroPos(&mi_motor_can2[i]);  // 可选
        HAL_Delay(100);
        //限速、限流、Kp
        Set_Motor_Parameter(&mi_motor_can2[i], Limit_Spd, 30.0f, 'f');
        Set_Motor_Parameter(&mi_motor_can2[i], Limit_Cur, 12.0f, 'f');
        Set_Motor_Parameter(&mi_motor_can2[i], Loc_Kp, 15.0f, 'f');
        motor_controlmode(&mi_motor_can2[i], 0.0f, 0.0f, 0.0f, 8.0f, 0.5f);
        HAL_Delay(5);
        
    }
    
		 
		 
}

// ================== 单腿 IK + 简单力控 (弹簧阻尼模型) ==================
/*void leg_set_position_idx(uint8_t leg_idx, float x, float y)
{
    // ---------- 1. IK 解算 (得到目标关节角度) ----------
    float cx = -x;
    float cy = y;
    if (leg_idx >= 2) cx = -cx;

    float L_sq = cx * cx + cy * cy;
    float L = sqrtf(L_sq);
    L = clamp_f(L, fabsf(L1 - L2_EFF) + 5.0f, (L1 + L2_EFF) * 0.95f);

    float psi = asinf(clamp_f(-cx / L, -1.0f, 1.0f));
    float phi = acosf(clamp_f((L_sq + L1 * L1 - L2_EFF * L2_EFF) / (2.0f * L1 * L), -1.0f, 1.0f));

    float theta1 = -(phi - psi);
    float theta2 = -(phi + psi);

    // 加上机械偏移，得到目标关节角度（弧度）
    float pos1_rad = theta1 + OFFSET_ANGLE;
    float pos2_rad = theta2 + OFFSET_ANGLE;

    // ---------- 2. 获取当前实际关节角度 (用于反馈) ----------
    float actual_rad1 = 0.0f;
    float actual_rad2 = 0.0f;

    if (leg_idx < 2) {
        // 从CAN1读取实际角度
        actual_rad1 = mi_motor_can1[leg_idx * 2].Angle;
        actual_rad2 = mi_motor_can1[leg_idx * 2 + 1].Angle;
    } else {
        uint8_t base = (leg_idx - 2) * 2;
        // 从CAN2读取实际角度
        actual_rad1 = mi_motor_can2[base].Angle;
        actual_rad2 = mi_motor_can2[base + 1].Angle;
    }

    // ---------- 3. 计算简单力控：弹簧阻尼力 ----------
    // 刚度系数 (Stiffness)：让机器人像个弹簧
    float k_spring = -0.2f;  // 值越大越硬，越小越软
    // 阻尼系数 (Damping)：吸收震动
    float d_damping = -1.0f;  // 值越大，运动越平稳，但反应越迟钝

    // 计算位置误差 (目标 - 实际)
    float error1 = pos1_rad - actual_rad1;
    float error2 = pos2_rad - actual_rad2;

    // 计算力矩前馈 (弹簧力 = k * error)
    float torque_feedforward1 = k_spring * error1;
    float torque_feedforward2 = k_spring * error2;

    // 注意：`motor_controlmode` 内部已经包含位置和速度的 PID 环。
    // 我们这里在 `torque` 参数里加一个弹簧力前馈，相当于给电机一个“预拉力”。
    // 这样机器人就会有弹性，不会僵死。

    // ---------- 4. 发送指令 ----------
    if (leg_idx < 2) {
       
        motor_controlmode(&mi_motor_can1[leg_idx * 2], torque_feedforward1, pos1_rad, 0.0f, 8.0f, 0.7f);
        motor_controlmode(&mi_motor_can1[leg_idx * 2 + 1], torque_feedforward2, pos2_rad, 0.0f, 8.0f, 0.7f);
    } else {
        uint8_t base = (leg_idx - 2) * 2;
  
        motor_controlmode(&mi_motor_can2[base], -torque_feedforward1, -pos1_rad, 0.0f, 8.0f, 0.7f);
        motor_controlmode(&mi_motor_can2[base + 1], -torque_feedforward2, -pos2_rad, 0.0f, 8.0f, 0.7f);
    }
}*/
void leg_set_position_idx(uint8_t leg_idx, float x, float y)
{
    // ---------- IK 解算（保持不变） ----------
    float cx = -x;
    float cy = y;
    if (leg_idx >= 2) cx = -cx;

    float L_sq = cx * cx + cy * cy;
    float L = sqrtf(L_sq);
    L = clamp_f(L, fabsf(L1 - L2_EFF) + 5.0f, (L1 + L2_EFF) * 0.95f);

    float psi = asinf(clamp_f(-cx / L, -1.0f, 1.0f));
    float phi = acosf(clamp_f((L_sq + L1 * L1 - L2_EFF * L2_EFF) / (2.0f * L1 * L), -1.0f, 1.0f));

    float theta1 = -(phi - psi);
    float theta2 = -(phi + psi);

    float pos1_rad = theta1 + OFFSET_ANGLE;
    float pos2_rad = theta2 + OFFSET_ANGLE;

    // ---------- 获取实际角度 ----------
    float actual_rad1 = 0.0f, actual_rad2 = 0.0f;
    float actual_vel1 = 0.0f, actual_vel2 = 0.0f;  // 需要获取实际速度，如果没有则用差分估算

    if (leg_idx < 2) {
        actual_rad1 = mi_motor_can1[leg_idx * 2].Angle;
        actual_rad2 = mi_motor_can1[leg_idx * 2 + 1].Angle;
        // 假设电机结构体中也有速度：.Speed
        actual_vel1 = mi_motor_can1[leg_idx * 2].Speed;
        actual_vel2 = mi_motor_can1[leg_idx * 2 + 1].Speed;
    } else {
        uint8_t base = (leg_idx - 2) * 2;
        actual_rad1 = mi_motor_can2[base].Angle;
        actual_rad2 = mi_motor_can2[base + 1].Angle;
        actual_vel1 = mi_motor_can2[base].Speed;
        actual_vel2 = mi_motor_can2[base + 1].Speed;
    }

    // ---------- 改进的力控：正刚度 + 阻尼 ----------
    // 刚度系数 (Stiffness)：根据当前腿的垂直高度 y 动态调整，使支撑力平稳
    // 基本刚度：当 y = 213.2mm 时，Kp_spring = 1.5 Nm/rad（经验值）
    float base_stiffness = 2.0f;
    // 高度补偿因子：y 越大（腿越长）刚度适当增大，防止“软”；y 越小（腿越短）刚度减小，防止“蹦”
    float height_factor = 213.2f / y;   // y 为当前目标高度，注意避免除零
    height_factor = clamp_f(height_factor, 0.8f, 1.5f);
    float stiffness = base_stiffness * height_factor;

    // 阻尼系数 (Damping)：抑制震荡，与速度成正比
    float damping = 0.1f;   // 0.05~0.15 可调

    float error1 = pos1_rad - actual_rad1;
    float error2 = pos2_rad - actual_rad2;

    // 弹簧力矩 = 刚度 * 误差
    float torque_spring1 = stiffness * error1;
    float torque_spring2 = stiffness * error2;

    // 阻尼力矩 = 阻尼系数 * (0 - 实际速度)  即抵抗运动
    float torque_damp1 = -damping * actual_vel1;
    float torque_damp2 = -damping * actual_vel2;

    // 总前馈力矩 = 弹簧力矩 + 阻尼力矩
    float torque_feedforward1 = torque_spring1 + torque_damp1;
    float torque_feedforward2 = torque_spring2 + torque_damp2;

    // 安全限幅（防止过大电流）
    const float MAX_TORQUE = 12.0f;   // 单位 Nm（根据电机实际能力调整）
    torque_feedforward1 = clamp_f(torque_feedforward1, -MAX_TORQUE, MAX_TORQUE);
    torque_feedforward2 = clamp_f(torque_feedforward2, -MAX_TORQUE, MAX_TORQUE);

    // 关节角度限幅（防止撞机械限位）
    const float JOINT_MAX = 1.4f;   // 约80度
    const float JOINT_MIN = -1.2f;  // 约-70度
    pos1_rad = clamp_f(pos1_rad, JOINT_MIN, JOINT_MAX);
    pos2_rad = clamp_f(pos2_rad, JOINT_MIN, JOINT_MAX);

    // ---------- 发送指令 ----------
    // 注意：motor_controlmode 的力矩参数为前馈，位置环内部仍有 Kp（需降低）
    if (leg_idx < 2) {
        motor_controlmode(&mi_motor_can1[leg_idx * 2],     torque_feedforward1, pos1_rad, 0.0f, 13.0f, 0.9f);
        motor_controlmode(&mi_motor_can1[leg_idx * 2 + 1], torque_feedforward2, pos2_rad, 0.0f, 13.0f, 0.9f);
    } else {
        uint8_t base = (leg_idx - 2) * 2;
        // 右腿的力矩符号需要反相（因为你的镜像处理
        motor_controlmode(&mi_motor_can2[base],     -torque_feedforward1, -pos1_rad, 0.0f, 13.0f, 0.9f);
        motor_controlmode(&mi_motor_can2[base + 1], -torque_feedforward2, -pos2_rad, 0.0f, 13.0f, 0.9f);
    }
}
// ================== 核心步态更新（集成姿态自适应补偿） ==================
void update_quadruped_gait(float dt)
{
    // 1. 获取当前姿态角（单位：度，已通过卡尔曼滤波平滑）
    //    假设 imu 结构体在 process.c 中定义并更新，pit0/rol0 是滤波后的值
    float pitch_deg = imu.pit0;   // 俯仰: 正 = 低头（前腿下沉）
    float roll_deg  = imu.rol0;   // 横滚: 正 = 左倾（左腿下沉）

    // 2. 自适应补偿参数（可调，建议从小开始）
    const float PITCH_GAIN = 2.0f;    // 俯仰补偿增益 (mm/度)
    const float ROLL_GAIN  = 2.0f;    // 横滚补偿增益 (mm/度)
    const float MAX_COMP   = 100.0f;    // 单腿最大补偿量 (mm)
    const float COMP_RATE_LIMIT = 20.0f; // 补偿量变化速率限制 (mm/s)

    // 3. 原始补偿量（毫米）
    float raw_comp_pitch = pitch_deg * PITCH_GAIN;
    float raw_comp_roll  = roll_deg  * ROLL_GAIN;

    // 4. 一阶低通滤波平滑
    float alpha = 0.4f;   // 滤波系数，0.2~0.6 可调
    comp_pitch_lpf = alpha * raw_comp_pitch + (1.0f - alpha) * comp_pitch_lpf;
    comp_roll_lpf  = alpha * raw_comp_roll  + (1.0f - alpha) * comp_roll_lpf;

    // 5. 速率限制（防止突变）
    float max_delta = COMP_RATE_LIMIT * dt;
    float delta_pitch = comp_pitch_lpf - last_comp_pitch;
    float delta_roll  = comp_roll_lpf - last_comp_roll;
    if (delta_pitch > max_delta) delta_pitch = max_delta;
    if (delta_pitch < -max_delta) delta_pitch = -max_delta;
    if (delta_roll > max_delta) delta_roll = max_delta;
    if (delta_roll < -max_delta) delta_roll = -max_delta;
    comp_pitch_lpf = last_comp_pitch + delta_pitch;
    comp_roll_lpf  = last_comp_roll  + delta_roll;
    last_comp_pitch = comp_pitch_lpf;
    last_comp_roll  = comp_roll_lpf;

    // 6. 高度平滑过渡
    myGait.y_base += (target_y_base - myGait.y_base) * 1.0f;

    // 7. 判断是否运动
    uint8_t is_moving = (fabsf(myGait.S) > 1.0f || fabsf(myGait.yaw_s) > 1.0f || myGait.stepping_in_place);

   if (!is_moving) {
    myGait.timer = 0.0f;
    // 仍然进行姿态补偿，但不进行步态周期推进
    for (uint8_t i = 0; i < 4; i++) {
        float ty = myGait.y_base;
        float comp = 0.0f;
        if (i == 0 || i == 3) comp -= comp_pitch_lpf;
        else comp += comp_pitch_lpf;
        if (i == 0 || i == 1) comp -= comp_roll_lpf;
        else comp += comp_roll_lpf;
        if (comp > MAX_COMP) comp = MAX_COMP;
        if (comp < -MAX_COMP) comp = -MAX_COMP;
        ty += comp;
        leg_set_position_idx(i, 0.0f, ty);
    }
    return;
}

    // 9. 运动状态：步态计时器更新
    myGait.timer += dt;
    if (myGait.timer >= myGait.T_cycle) {
        myGait.timer -= myGait.T_cycle;
    }

    // 10. 遍历四条腿，生成轨迹并施加支撑相补偿
    for (uint8_t i = 0; i < 4; i++) {
        float current_v = (i < 2) ? (myGait.S + myGait.yaw_s) : (myGait.S - myGait.yaw_s);
        // 相位偏移：对角步态 (0和3同相，1和2同相)
        float phase = myGait.timer / myGait.T_cycle + ((i == 1 || i == 3) ? 0.5f : 0.0f);
        if (phase >= 1.0f) phase -= 1.0f;

        float tx, ty;

        if (phase < 0.5f) { // 摆动相：不加姿态补偿
            float s = phase / 0.5f;
            float sigma = 2.0f * M_PI * s;
            tx = -current_v * 0.5f + current_v * (sigma - sinf(sigma)) / (2.0f * M_PI);
            ty = myGait.y_base - myGait.H * (1.0f - cosf(sigma)) * 0.5f;
        } else { // 支撑相：加入姿态补偿
            float s = (phase - 0.5f) / 0.5f;
            tx = current_v * 0.5f - current_v * s;
            ty = myGait.y_base;

            // 计算本条腿的总补偿量
            float comp = 0.0f;
            // 俯仰补偿：前腿(0,3)减，后腿(1,2)加
            if (i == 0 || i == 3)
                comp -= comp_pitch_lpf;
            else
                comp += comp_pitch_lpf;
            // 横滚补偿：左腿(0,1)减，右腿(2,3)加
            if (i == 0 || i == 1)
                comp -= comp_roll_lpf;
            else
                comp += comp_roll_lpf;

            // 限幅
            if (comp > MAX_COMP) comp = MAX_COMP;
            if (comp < -MAX_COMP) comp = -MAX_COMP;

            ty += comp;
        }

        // 发送腿位置指令
        leg_set_position_idx(i, tx, ty);
    }
}
// ================== 遥控器处理 (增加回中立即停止逻辑) ==================
void update_control_from_remote(remoter_t *remote)
{
    if (!remote->online) {
        // 离线保护：清零所有运动
        myGait.S = 0.0f;
        myGait.yaw_s = 0.0f;
       
    }

    // ---------- 核心修复：检测遥控完全归零，立即强制停止 ----------
    // 判断 CH1 (速度) 和 CH0 (转向) 是否都在死区范围内
    if (abs(remote->rc.ch[2]) < 40 && abs(remote->rc.ch[0]) < 40) {
        myGait.S = 0.0f;
        myGait.yaw_s = 0.0f;
        // 注意：不强制清除 stepping_in_place，因为它由 ch5 开关单独控制
    }

    // CH5（ch[4]）作为运动使能开关
    if (remote->rc.ch[4] != 0) {
        
        // 前后速度
        int16_t raw_ch1 = remote->rc.ch[2];
        float target_S;
        if (abs(raw_ch1) < 40) {
            target_S = 0.0f;
        } else {
            target_S = -(float)raw_ch1 * (90.0f / 670.0f);
        }
        // 依然保留一点点平滑，以免跳变
        myGait.S = myGait.S * 0.8f + target_S * 0.2f;

        // 转向控制
        int16_t raw_ch0 = remote->rc.ch[0];
        float target_yaw;
        if (abs(raw_ch0) < 40) {
            target_yaw = 0.0f;
        } else {
            target_yaw = (float)raw_ch0 * (80.0f / 670.0f);
        }
        myGait.yaw_s = myGait.yaw_s * 0.8f + target_yaw * 0.2f;

    } else {
        // 使能关闭：强制清零
        myGait.S = 0.0f;
        myGait.yaw_s = 0.0f;
    }

    // 原地踏步
    myGait.stepping_in_place = (remote->rc.ch[5] > 500);

		// ---------- 新增：CH6（ch[5]）控制抬腿高度 ----------
    // CH6 > 500 时抬腿高度为 130mm，否则为默认 60mm
    if (remote->rc.ch[6] > 500) {
        myGait.H = 130.0f;   // 高抬腿模式
			  myGait.T_cycle = 0.3f ;
    } else {
        myGait.H = 60.0f;
        myGait.T_cycle = 0.2f		;	// 默认抬腿高度
    }
    // 高度切换
    int16_t raw_ch8 = remote->rc.ch[7];
    if (raw_ch8 > 500) {
        target_y_base = 190.0f; // 低姿态
    } else if (raw_ch8 < -500) {
        target_y_base = 270.0f; // 高姿态
    } else {
        target_y_base = 213.2f; // 默认高度
    }

    // 安全限幅
    myGait.S = clamp_f(myGait.S, -90.0f, 90.0f);
    myGait.yaw_s = clamp_f(myGait.yaw_s, -60.0f, 60.0f);
}
// ================== 电机错误自动恢复 ==================
void check_and_recover_motors(void)
{
    // 遍历 CAN1 上的 8 个电机
    for (uint8_t i = 0; i < 8; i++) {
        if (mi_motor_can1[i].error_code != 0) {
            // 1. 停止并清除错误标志 (clear_error = 1)
            stop_cybergear(&mi_motor_can1[i], 1);
            HAL_Delay(1);
            
            // 2. 重新使能电机
            start_cybergear(&mi_motor_can1[i]);
            HAL_Delay(1);
            
            // 3. 发送一个零位指令，让电机回到安全位置
            motor_controlmode(&mi_motor_can1[i], 0.0f, 0.0f, 0.0f, 5.0f, 0.05f);
            
            // 4. 清除结构体内的错误码
            mi_motor_can1[i].error_code = 0;
            
            // 可选：打印调试信息（通过串口）
            // printf("Recovered motor ID %d, error was %d\r\n", mi_motor_can1[i].CAN_ID, old_error);
        }
    }
}
