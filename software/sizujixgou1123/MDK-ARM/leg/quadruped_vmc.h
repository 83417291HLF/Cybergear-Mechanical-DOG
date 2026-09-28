#ifndef __QUADRUPED_VMC_H
#define __QUADRUPED_VMC_H

#include "leg.h"

// ================== VMC 虚拟弹簧核心参数 ==================
// 这些参数决定了机器人的“手感”：硬还是软
#define VMC_KP_Y          1500.0f  // 支撑刚度 (N/m)，越大腿越硬
#define VMC_KD_Y          60.0f    // 垂直阻尼 (N·s/m)，防止起跳和震荡
#define VMC_KP_X          800.0f   // 前后刚度 (N/m)
#define VMC_KD_X          30.0f    // 前后阻尼

// ================== 物理模型参数 ==================
#define ROBOT_MASS        12.0f    // 机器人总质量 (kg)
#define GRAVITY_ACCEL     9.81f    // 重力加速度
#define STAND_FORCE_FF    ((ROBOT_MASS * GRAVITY_ACCEL) / 4.0f) // 单腿静止支撑力 (N)

// ================== 运控模式 PD 增益 ==================
// 这是小米电机内部控制器的增益，用于辅助 VMC 力矩
#define MOTOR_VMC_KP      15.0f    // 运控模式 Kp
#define MOTOR_VMC_KD      1.5f     // 运控模式 Kd

// ================== 函数原型 ==================

/**
 * @brief  VMC 控制核心：计算虚拟弹簧力并映射至电机力矩
 * @param  leg_idx: 腿编号 (0:LF, 1:LH, 2:RH, 3:RF)
 * @param  target_x: 目标足端 X 坐标 (mm)
 * @param  target_y: 目标足端 Y 坐标 (mm)
 * @param  real_x: 当前实际足端 X 坐标 (mm, 可由 FK 正运动学获得)
 * @param  real_y: 当前实际足端 Y 坐标 (mm, 可由 FK 正运动学获得)
 */
void leg_vmc_control(uint8_t leg_idx, float target_x, float target_y, float real_x, float real_y);

/**
 * @brief  正运动学 (Forward Kinematics)：由电机角度推算足端坐标
 * @param  theta1: 髋关节角度 (rad)
 * @param  theta2: 膝关节角度 (rad)
 * @param  x: 输出 X 坐标指针
 * @param  y: 输出 Y 坐标指针
 */
void leg_forward_kinematics(float theta1, float theta2, float *x, float *y);

/**
 * @brief  更新四足机器人所有腿的 VMC 状态 (通常在 200Hz-500Hz 循环调用)
 */
void update_quadruped_vmc(float dt);

#endif // __QUADRUPED_VMC_H
