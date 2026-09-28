#include "sbus.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"  

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00

uint8_t rx_buff[BUFF_SIZE];
remoter_t remoter;

void print_remote_channels(remoter_t *remote)
{
    char buffer[200];
    sprintf(buffer, "Online: %d\r\n", remote->online);
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    sprintf(buffer, "Channels: ");
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    for(int i = 0; i < 10; i++) {
        sprintf(buffer, "CH%d:%d ", i, remote->rc.ch[i]);
        HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
    sprintf(buffer, "\r\n");
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void print_remote_detailed(remoter_t *remote)
{
    char buffer[300];
    
    sprintf(buffer, "=== Remote Data ===\r\n");
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "Status: %s\r\n", remote->online ? "Online" : "Offline");
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "CH0:%d CH1:%d CH2:%d CH3:%d\r\n", 
            remote->rc.ch[0], remote->rc.ch[1], 
            remote->rc.ch[2], remote->rc.ch[3]);
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "CH4:%d CH5:%d CH6:%d CH7:%d\r\n", 
            remote->rc.ch[4], remote->rc.ch[5], 
            remote->rc.ch[6], remote->rc.ch[7]);
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "CH8:%d CH9:%d\r\n", 
            remote->rc.ch[8], remote->rc.ch[9]);
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "===================\r\n\r\n");
    HAL_UART_Transmit(&huart7, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}





void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;

    if (buf[23] == 0x0C)
        remoter->online = 0;
    else
        remoter->online = 1;

    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF)-992;
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF)-992;
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF)-992;
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF)-992;
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF)-992;
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF)-992;
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF)-992;
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF)-992;
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF)-992;
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF)-992;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == USART1)
	{
		if (Size <= BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
			sbus_frame_parse(&remoter, rx_buff);
//			memset(rx_buff, 0, BUFF_SIZE);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
			memset(rx_buff, 0, BUFF_SIZE);							   
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE*2); // 接收发生错误后重启
		memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存		
	}
}
