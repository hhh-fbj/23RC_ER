#include "DEV_DRF1609H.h"
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "System_DataPool.h"
#include "stdarg.h"
#include "stdio.h"
#define STR_LENGTH 30

//收件ID:K/Y/D/A 头二帧0x53  寄件ID:K/Y/D 功能目标  尾帧0x45
//定时发送数据过来/过去 AK C000
/* 云台
收到控制器数据：YK
选择目标 X001/ X011/ X021/ X003/ X023/ X017/ X123/ X103
*/

DRF1609H_classdef::DRF1609H_classdef(void)
{
	
}

//头帧 ID 特殊0?\APP1~7?\DEV8~F? 功能 尾帧
int time_num;
uint8_t Last_DRF_REC_Data[DRF_REC_SIZE];
uint8_t jl_sum;
void DRF1609H_classdef::Receive(uint8_t *data)
{
    if(DRF_REC_Data[2]=='K' && DRF_REC_Data[1] == 0x53 && DRF_REC_Data[5]==0x45)
    {
			if(Last_DRF_REC_Data[4] == DRF_REC_Data[4] &&\
				Last_DRF_REC_Data[3] == DRF_REC_Data[3])
			{
				jl_sum++;
				if(jl_sum>102){jl_sum=3;}
			}
			else
			{
				jl_sum = 2;
			}
			if(jl_sum<3)
			{
				jl_sum=3;
				switch(DRF_REC_Data[3])
				{
					case 0x58://X 选择
						switch(DRF_REC_Data[4])
						{
							case 001:Clamp.Tar_Ring = Tar_LTen;Clamp.Place_Flag = 1;break;
							case 011:Clamp.Tar_Ring = Tar_MTen;Clamp.Place_Flag = 1;break;
							case 021:Clamp.Tar_Ring = Tar_RTen;Clamp.Place_Flag = 1;break;
							case 003:Clamp.Tar_Ring = Tar_LThirty;Clamp.Place_Flag = 1;break;
							case 023:Clamp.Tar_Ring = Tar_RThirty;Clamp.Place_Flag = 1;break;
							case 017:Clamp.Tar_Ring = Tar_MSeventy;Clamp.Place_Flag = 1;break;
							case 103:Clamp.Tar_Ring = Tar_DLThirty;Clamp.Place_Flag = 1;break;
							case 123:Clamp.Tar_Ring = Tar_DRThirty;Clamp.Place_Flag = 1;break;
						}
					break;
						
					case 0x51://Q 取环
						if(DRF_REC_Data[4]==0)Clamp.Init_Flag = 1;break;
					break;
					
					case 0x43://C 测试
						
					break;
					
					case 0x4B://K 启动
					break;
				}
			}
		}
		for(uint8_t i = 0;i<DRF_REC_SIZE;i++)
		{
			Last_DRF_REC_Data[i]=DRF_REC_Data[i];
		}
		//检测
}

//
//功能[3] 模块[4] 数据[5]~[8] 
uint8_t js_S;
void DRF1609H_classdef::Send(void)
{
	DRF_SEN.data[0]='K';
	DRF_SEN.data[1]= 0x53 ;
	DRF_SEN.data[2]='Y';
	DRF_SEN.data[8]=0x45;
	if(js_S>0){js_S--;}else{js_S=0;DRF_SEN.data[3]=0;DRF_SEN.Pack.Data=0;}
	HAL_UART_Transmit_DMA(&huart8 , DRF_SEN.data, sizeof(DRF_SEN.data));
}

void DRF1609H_classdef::SetSend(void)
{
	js_S=3;
}
