#include "DEV_L1TTL.h"
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "System_DataPool.h"
#include "stdarg.h"
#include "stdio.h"
#define STR_LENGTH 40


L1TTL_classdef::L1TTL_classdef(void)
{
	
}



void L1TTL_classdef::L1TTL_Printf(const char *str, ...)
{
			char buffer[STR_LENGTH]={0};  // 数据长度
			va_list arg_ptr;
			va_start(arg_ptr, str);
			int len = vsnprintf(buffer, STR_LENGTH, str, arg_ptr);
			va_end(arg_ptr);

			HAL_UART_Transmit_IT(&huart7, (uint8_t *)buffer, len);
			
    while(huart7.gState != HAL_UART_STATE_READY);	//等待发送完毕
}

int l1_cs;
void L1TTL_classdef::control(void)
{
	switch(l1_cs)
	{
		//距离偏移量
		case 13:L1TTL_Printf("IGET:1");l1_cs=0;break;
		//量程
		case 1:L1TTL_Printf("IGET:2");l1_cs=0;break;
		//波特率
		case 2:L1TTL_Printf("ISET:3");l1_cs=0;break;
		//协议格式类型
		case 3:L1TTL_Printf("IGET:4");l1_cs=0;break;
		//输出距离数字格式
		case 4:L1TTL_Printf("IGET:5");l1_cs=0;break;
		//从机设备地址
		case 5:L1TTL_Printf("IGET:6");l1_cs=0;break;
		//测量输出速率
		case 6:L1TTL_Printf("IGET:7");l1_cs=0;break;
		//上电自动测量标识
		case 7:L1TTL_Printf("IGET:8");l1_cs=0;break;

		//单次测量
		case 8:L1TTL_Printf("iSM");l1_cs=0;break;
		//连续测量
		case 9:L1TTL_Printf("iACM");l1_cs=0;break;
		//快速连续测量
		case 10:L1TTL_Printf("iFACM");l1_cs=0;break;
		//停止测量
		case 11:L1TTL_Printf("iHALT");l1_cs=0;break;
		//激光开启关闭
		case 12:L1TTL_Printf("iLD:X");l1_cs=0;break;
	}
}