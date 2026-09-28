#include "leg.h"

// 虚拟弹簧参数 (需要根据机身重量调试)
#define VMC_KP_Y  1500.0f  // 垂直方向刚度 (N/m)
#define VMC_KD_Y  50.0f    // 垂直方向阻尼
#define VMC_KP_X  800.0f   // 水平方向刚度
#define BODY_MASS 10.0f    // 机器人重量 (kg)
#define GRAVITY   9.81f

// 计算雅可比转置并发送力控指令
void leg_vmc_control(uint8_t leg_idx, float target_x, float target_y, float current_x, float current_y)
{
    // 1. 计算虚拟弹簧力 (足端力 Fx, Fy)
    // F = Kp * (pos_error) - Kd * velocity
    float Fx = VMC_KP_X * (target_x - current_x);
    float Fy = VMC_KP_Y * (target_y - current_y) + (BODY_MASS * GRAVITY / 4.0f); // 加上重力补偿

    // 2. 逆运动学求解目标角度 (用于运控模式的 PD 辅助)
    // 这里复用你之前的 IK 计算逻辑
    float L_sq = target_x * target_x + target_y * target_y;
    float L = sqrtf(L_sq);
    L = clamp(L, fabsf(L1 - L2_EFF) + 2.0f, (L1 + L2_EFF) * 0.98f);
    
    float psi = asinf(clamp(-target_x / L, -1.0f, 1.0f));
    float phi = acosf(clamp((L_sq + L1 * L1 - L2_EFF * L2_EFF) / (2.0f * L1 * L), -1.0f, 1.0f));
    
    float theta1 = -(phi - psi) + OFFSET_ANGLE;
    float theta2 = -(phi + psi) + OFFSET_ANGLE;

    // 3. 雅可比矩阵转置计算：Tau = J^T * F
    float s1 = sinf(theta1 - OFFSET_ANGLE);
    float c1 = cosf(theta1 - OFFSET_ANGLE);
    float s12 = sinf(theta1 - OFFSET_ANGLE + theta2 - OFFSET_ANGLE);
    float c12 = cosf(theta1 - OFFSET_ANGLE + theta2 - OFFSET_ANGLE);

    // 雅可比转置矩阵元素 (单位换算为米)
    float l1_m = L1 / 1000.0f;
    float l2_m = L2_EFF / 1000.0f;

    float jt11 = l1_m * c1 + l2_m * c12;  // dX/dtheta1
    float jt12 = l1_m * s1 + l2_m * s12;  // dY/dtheta1
    float jt21 = l2_m * c12;             // dX/dtheta2
    float jt22 = l2_m * s12;             // dY/dtheta2

    // 计算电机力矩 (N·m)
    float tau1 = jt11 * Fx + jt12 * Fy;
    float tau2 = jt21 * Fx + jt22 * Fy;

    // 4. 执行运控指令
    // 注意：运控模式下我们给一个较小的 Kp (例如 10~20) 作为软弹簧辅助，主要的力由 tau 驱动
    float motor_kp = 20.0f; 
    float motor_kd = 1.2f;

    if (leg_idx < 2) {
        uint8_t base = leg_idx * 2;
        motor_controlmode(&mi_motor_can1[base],   tau1, theta1, 0, motor_kp, motor_kd);
        motor_controlmode(&mi_motor_can1[base+1], tau2, theta2, 0, motor_kp, motor_kd);
    } else {
        float mirror = -1.0f;
        uint8_t base = (leg_idx - 2) * 2;
        motor_controlmode(&mi_motor_can2[base],   mirror * tau1, mirror * theta1, 0, motor_kp, motor_kd);
        motor_controlmode(&mi_motor_can2[base+1], mirror * tau2, mirror * theta2, 0, motor_kp, motor_kd);
    }
}
