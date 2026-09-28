#include "dm_imu.h"
#include "string.h"
#include "bsp_CAN.h"
imu_can_t gIMU = {0};

/**
************************************************************************
* @brief: imu_float_to_uint: 浮点数转换为无符号整数函数（IMU专用版本）
************************************************************************
**/
int imu_float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief: imu_uint_to_float: 无符号整数转换为浮点数函数（IMU专用版本）
************************************************************************
**/
float imu_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
  * @brief  初始化IMU CAN1接口
 **/
void IMU_CAN1_Init(uint8_t can_id, CAN_HandleTypeDef *hcan)
{
    gIMU.can_id = can_id;
    gIMU.can_handle = hcan;

    memset(gIMU.accel, 0, sizeof(gIMU.accel));
    memset(gIMU.gyro, 0, sizeof(gIMU.gyro));
    memset(gIMU.q,     0, sizeof(gIMU.q));

    gIMU.pitch = 0;
    gIMU.roll  = 0;
    gIMU.yaw   = 0;

    gIMU.temperature = 0;
}

/**
  * @brief  IMU命令发送函数
 **/
static void IMU_CAN1_SendCmd(uint8_t reg_id, uint8_t cmd_type, uint32_t data)
{
    if (gIMU.can_handle == NULL)
        return;

    uint8_t buf[8] = {0};

    buf[0] = 0xCC;
    buf[1] = reg_id;
    buf[2] = cmd_type;
    buf[3] = 0xDD;

    memcpy(&buf[4], &data, 4);

    CAN1_SendData(buf, 8);
}

/**
  * @brief 写寄存器接口
 **/
void IMU_CAN1_WriteReg(uint8_t reg_id, uint32_t data)
{
    IMU_CAN1_SendCmd(reg_id, CMD_WRITE, data);
}

/**
  * @brief 读寄存器接口
 **/
void IMU_CAN1_ReadReg(uint8_t reg_id)
{
    IMU_CAN1_SendCmd(reg_id, CMD_READ, 0);
}

/******************************************************************************
 * 基础功能函数
 *****************************************************************************/
void IMU_CAN1_Reboot(void)
{
    IMU_CAN1_WriteReg(IMU_REG_REBOOT, 0);
}

void IMU_CAN1_AccelCalibration(void)
{
    IMU_CAN1_WriteReg(IMU_REG_ACCEL_CALI, 0);
}

void IMU_CAN1_GyroCalibration(void)
{
    IMU_CAN1_WriteReg(IMU_REG_GYRO_CALI, 0);
}

/******************************************************************************
 * 数据请求函数
 *****************************************************************************/
void IMU_CAN1_RequestAccel(void)
{
    IMU_CAN1_ReadReg(IMU_REG_ACCEL);
}

void IMU_CAN1_RequestGyro(void)
{
    IMU_CAN1_ReadReg(IMU_REG_GYRO);
}

void IMU_CAN1_RequestEuler(void)
{
    IMU_CAN1_ReadReg(IMU_REG_EULER);
}

void IMU_CAN1_RequestQuaternion(void)
{
    IMU_CAN1_ReadReg(IMU_REG_QUAT);
}

/******************************************************************************
 * 以下为 CAN 数据解析部分
 *****************************************************************************/

/**
  * @brief 解析加速度数据
 **/
static void IMU_CAN1_ParseAccel(uint8_t *pData)
{
    uint16_t ax = (pData[3] << 8) | pData[2];
    uint16_t ay = (pData[5] << 8) | pData[4];
    uint16_t az = (pData[7] << 8) | pData[6];

    gIMU.accel[0] = imu_uint_to_float(ax, IMU_ACCEL_MIN, IMU_ACCEL_MAX, 16);
    gIMU.accel[1] = imu_uint_to_float(ay, IMU_ACCEL_MIN, IMU_ACCEL_MAX, 16);
    gIMU.accel[2] = imu_uint_to_float(az, IMU_ACCEL_MIN, IMU_ACCEL_MAX, 16);
}

/**
  * @brief 解析陀螺仪数据
 **/
static void IMU_CAN1_ParseGyro(uint8_t *pData)
{
    uint16_t gx = (pData[3] << 8) | pData[2];
    uint16_t gy = (pData[5] << 8) | pData[4];
    uint16_t gz = (pData[7] << 8) | pData[6];

    gIMU.gyro[0] = imu_uint_to_float(gx, IMU_GYRO_MIN, IMU_GYRO_MAX, 16);
    gIMU.gyro[1] = imu_uint_to_float(gy, IMU_GYRO_MIN, IMU_GYRO_MAX, 16);
    gIMU.gyro[2] = imu_uint_to_float(gz, IMU_GYRO_MIN, IMU_GYRO_MAX, 16);
}

/**
  * @brief 解析欧拉角数据
 **/
static void IMU_CAN1_ParseEuler(uint8_t *pData)
{
    uint16_t p = (pData[3] << 8) | pData[2];
    uint16_t y = (pData[5] << 8) | pData[4];
    uint16_t r = (pData[7] << 8) | pData[6];

    gIMU.pitch = imu_uint_to_float(p, IMU_PITCH_MIN, IMU_PITCH_MAX, 16);
    gIMU.yaw   = imu_uint_to_float(y, IMU_YAW_MIN,   IMU_YAW_MAX,   16);
    gIMU.roll  = imu_uint_to_float(r, IMU_ROLL_MIN,  IMU_ROLL_MAX,  16);
}

/**
  * @brief 解析四元数数据
 **/
static void IMU_CAN1_ParseQuaternion(uint8_t *pData)
{
    uint16_t qw = (pData[3] << 8) | pData[2];
    uint16_t qx = (pData[5] << 8) | pData[4];
    uint16_t qy = (pData[7] << 8) | pData[6];
    uint16_t qz = (pData[9] << 8) | pData[8];

    gIMU.q[0] = imu_uint_to_float(qw, IMU_QUAT_MIN, IMU_QUAT_MAX, 16);
    gIMU.q[1] = imu_uint_to_float(qx, IMU_QUAT_MIN, IMU_QUAT_MAX, 16);
    gIMU.q[2] = imu_uint_to_float(qy, IMU_QUAT_MIN, IMU_QUAT_MAX, 16);
    gIMU.q[3] = imu_uint_to_float(qz, IMU_QUAT_MIN, IMU_QUAT_MAX, 16);
}

/**
  * @brief  CAN中断回调中调用的解析入口
 **/
void IMU_CAN1_RxCallback(CAN_RxHeaderTypeDef *rx_header, uint8_t *pData)
{
    if (rx_header->ExtId != gIMU.can_id)
        return;

    switch(pData[0])
    {
        case IMU_DATA_TAG_ACCEL:
            IMU_CAN1_ParseAccel(pData);
            break;

        case IMU_DATA_TAG_GYRO:
            IMU_CAN1_ParseGyro(pData);
            break;

        case IMU_DATA_TAG_EULER:
            IMU_CAN1_ParseEuler(pData);
            break;

        case IMU_DATA_TAG_QUAT:
            IMU_CAN1_ParseQuaternion(pData);
            break;

        default:
            break;
    }
}
