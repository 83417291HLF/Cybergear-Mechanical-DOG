#ifndef __BSP__CAN_H
#define __BSP__CAN_H

#include "can.h" // 引用CubeMX生成的can.h

/*****************************************************************************
 ** 移植配置修改区
 ****************************************************************************/
// CAN1 ID配置
#define CAN1_RX_ID 0x123 
#define CAN1_TX_ID 0x666 

// CAN2 ID配置 (新增)
#define CAN2_RX_ID 0x234 
#define CAN2_TX_ID 0x777 

/*****************************************************************************
 ** 全局结构体定义
 ****************************************************************************/
typedef struct
{
    // 发送部分
    uint8_t  TxFlag;               // 发送状态标志
    CAN_TxHeaderTypeDef TxHeader;  // HAL库发送结构体
    
    // 接收部分
    uint8_t  RxNum;                // 接收字节数/标志位
    uint8_t  RxData[9];            // 接收缓存（含结束符）
    CAN_RxHeaderTypeDef RxHeader;  // HAL库接收结构体
} xCAN_InfoDef;

// 声明全局变量，供外部 main.c 调用
extern xCAN_InfoDef xCAN1;
extern xCAN_InfoDef xCAN2;

/*****************************************************************************
 ** 声明全局函数
 ****************************************************************************/
// 初始化总函数
void can_bsp_init(void);

// CAN1 函数
void    CAN1_FilterInit(void);
uint8_t CAN1_SendData(uint8_t* msgData, uint8_t len);

// CAN2 函数 (新增)
void    CAN2_FilterInit(void);
uint8_t CAN2_SendData(uint8_t* msgData, uint8_t len);

#endif

