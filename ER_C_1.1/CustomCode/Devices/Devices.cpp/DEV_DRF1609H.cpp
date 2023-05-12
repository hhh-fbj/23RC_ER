#include "DEV_DRF1609H.h"
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "System_DataPool.h"
#include "stdarg.h"
#include "stdio.h"
#define STR_LENGTH 30

DRF1609H_classdef::DRF1609H_classdef(void)
{

}

//头帧 ID 特殊0?\APP1~7?\DEV8~F? 功能 尾帧
int time_num;
void DRF1609H_classdef::Receive(uint8_t *data)
{
    if(TJC4827X343_Data[0] == 0x53 && TJC4827X343_Data[4]==0x45 && TJC4827X343_Data[1]==0x01)
    { 
			if(Deal_Flag != 0)
			{
				Backup_Deal_Flag = TJC4827X343_Data[2]<<8 | TJC4827X343_Data[3];
			}
			else
			{
				Deal_Flag = TJC4827X343_Data[2]<<8 | TJC4827X343_Data[3];
			}
			time_num++;
    }
}

void DRF1609H_classdef::TJCPrintf(const char *str, ...)
{
//	if(huart7.gState == HAL_UART_STATE_READY)
//	{
			char buffer[STR_LENGTH]={0};  // 数据长度
			va_list arg_ptr;
			va_start(arg_ptr, str);
			int len = vsnprintf(buffer, STR_LENGTH, str, arg_ptr);
			va_end(arg_ptr);

			buffer[len] = 0xff;
			buffer[len+1] = 0xff;
			buffer[len+2] = 0xff;
			HAL_UART_Transmit_IT(&huart6, (uint8_t *)buffer, len+3);
//	}
    while(huart6.gState != HAL_UART_STATE_READY);	//等待发送完毕
}

void DRF1609H_classdef::communication(void)
{
//	time_num++;
	if(time_num > 0)
	{
		switch (Deal_Flag)
		{
			case 0x0205:
			break;
			
			case 0x0204:
			break;
			
			case 0x0203:
			break;
			
			case 0x0202:
			break;
			
			case 0x0201:
			break;
			
			case 0x0200:
			break;
		
			default:
			break;
		}

		TJCPrintf("n1.val++");
		time_num++;
		Deal_Flag = 0;
		if(Backup_Deal_Flag!=0)
		{
			Deal_Flag = Backup_Deal_Flag;
			Backup_Deal_Flag = 0;
		}
	}
}