#ifndef __DM_IMU_H
#define __DM_IMU_H


#include "stm32f4xx_hal.h"
#include "bsp_CAN.h"
#include "cybergear.h"

/* IMU主动发送数据类型标签 */
typedef enum
{
    IMU_DATA_TAG_ACCEL = 1,
    IMU_DATA_TAG_GYRO  = 2,
    IMU_DATA_TAG_EULER = 3,
    IMU_DATA_TAG_QUAT  = 4,
    IMU_DATA_TAG_TEMP  = 5
} imu_data_tag_e;

/* IMU CAN 数据结构体 */
typedef struct
{
    uint8_t can_id;

    CAN_HandleTypeDef *can_handle;

    float accel[3];
    float gyro[3];

    float pitch;
    float roll;
    float yaw;

    float q[4];

    float temperature;

} imu_can_t;

/* 全局IMU对象 */
extern imu_can_t gIMU;

/* 初始化接口 */
void IMU_CAN1_Init(uint8_t can_id, CAN_HandleTypeDef *hcan);

/* 寄存器读写接口 */
void IMU_CAN1_WriteReg(uint8_t reg_id, uint32_t data);
void IMU_CAN1_ReadReg(uint8_t reg_id);

/* 基础功能接口 */
void IMU_CAN1_Reboot(void);
void IMU_CAN1_AccelCalibration(void);
void IMU_CAN1_GyroCalibration(void);

/* 请求数据接口 */
void IMU_CAN1_RequestAccel(void);
void IMU_CAN1_RequestGyro(void);
void IMU_CAN1_RequestEuler(void);
void IMU_CAN1_RequestQuaternion(void);

/* CAN接收解析回调 */
void IMU_CAN1_RxCallback(CAN_RxHeaderTypeDef *rx_header, uint8_t *pData);

/* 已重命名的映射函数接口 */
int   imu_float_to_uint(float x_float, float x_min, float x_max, int bits);
float imu_uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif
