#ifndef __DM_IMU_MAP_H
#define __DM_IMU_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 线性映射表 ====================
// 加速度范围
#define ACCEL_CAN_MAX        (235.2f)
#define ACCEL_CAN_MIN        (-235.2f)

// 陀螺仪范围
#define GYRO_CAN_MAX         (34.88f)
#define GYRO_CAN_MIN         (-34.88f)

// 欧拉角范围
#define PITCH_CAN_MAX        (90.0f)
#define PITCH_CAN_MIN        (-90.0f)
#define ROLL_CAN_MAX         (180.0f)
#define ROLL_CAN_MIN         (-180.0f)
#define YAW_CAN_MAX          (180.0f)
#define YAW_CAN_MIN          (-180.0f)

// 四元数范围
#define QUATERNION_CAN_MAX   (1.0f)
#define QUATERNION_CAN_MIN   (-1.0f)

// ==================== 数据转换函数 ====================

/**
 * @brief 浮点数转换为无符号整数
 * @param x_float: 待转换浮点数
 * @param x_min: 范围最小值
 * @param x_max: 范围最大值
 * @param bits: 目标无符号整数位数
 * @retval 无符号整数结果
 */
static inline int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 无符号整数转换为浮点数
 * @param x_int: 待转换整数
 * @param x_min: 范围最小值
 * @param x_max: 范围最大值
 * @param bits: 无符号整数位数
 * @retval 浮点数结果
 */
static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

#ifdef __cplusplus
}
#endif

#endif
