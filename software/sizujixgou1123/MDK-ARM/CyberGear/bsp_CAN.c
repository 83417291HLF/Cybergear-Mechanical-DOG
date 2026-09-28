#include "bsp_CAN.h"
#include "stdio.h"
#include "string.h"
//#include "dm_imu.h"
#include "cybergear.h"
// 实例化两个CAN的数据结构体
xCAN_InfoDef xCAN1 = {0};
xCAN_InfoDef xCAN2 = {0};

/**
 * @brief: CAN总初始化
 * @details: 同时使能CAN1和CAN2，并开启中断接收
 **/
void can_bsp_init(void)
{
    // 初始化 CAN1
    CAN1_FilterInit();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 初始化 CAN2
    CAN2_FilterInit();
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/******************************************************************************
 * 函  数： CAN1_SendData / CAN2_SendData
 * 功  能： 发送数据
 ******************************************************************************/
uint8_t CAN1_SendData(uint8_t *msgData, uint8_t len)
{ 
    static uint32_t TxMailbox = 0;
    if(len > 8) len = 8;

    xCAN1.TxHeader.ExtId = CAN1_TX_ID;
    xCAN1.TxHeader.IDE   = CAN_ID_EXT;
    xCAN1.TxHeader.RTR   = CAN_RTR_DATA;
    xCAN1.TxHeader.DLC   = len;
    xCAN1.TxHeader.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    return HAL_CAN_AddTxMessage(&hcan1, &xCAN1.TxHeader, msgData, &TxMailbox);
}

uint8_t CAN2_SendData(uint8_t *msgData, uint8_t len)
{ 
    static uint32_t TxMailbox = 0;
    if(len > 8) len = 8;

    xCAN2.TxHeader.ExtId = CAN2_TX_ID;
    xCAN2.TxHeader.IDE   = CAN_ID_EXT;
    xCAN2.TxHeader.RTR   = CAN_RTR_DATA;
    xCAN2.TxHeader.DLC   = len;
    xCAN2.TxHeader.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
    return HAL_CAN_AddTxMessage(&hcan2, &xCAN2.TxHeader, msgData, &TxMailbox);
}

/******************************************************************************
 * 函  数： CAN筛选器初始化
 * 备  注： CAN2是CAN1的从设备，共用28个过滤器组。
 ******************************************************************************/
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;              // CAN1使用0号过滤器
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;   // 0-13分配给CAN1, 14-27分配给CAN2
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
}

void CAN2_FilterInit(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 14;             // CAN2从14号过滤器开始
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
}


