#include "DEV_DRF1609H.h"
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "System_DataPool.h"
#include "stdarg.h"
#include "stdio.h"
#define STR_LENGTH 40

//收件ID:K/Y/D/A 头二帧0x53功能 目标 第几次 尾帧0x45
//定时发送数据过来/过去 AK C000
/* 云台
收到控制器数据：YK
选择目标 X001/ X011/ X021/ X003/ X023/ X017/ X123/ X103
*/

DRF1609H_classdef::DRF1609H_classdef(void)
{
	
}

//
uint32_t last_idea_ctrl,idea_ctrl;
int Last_Num,Last_Tar,Last_Mode,Last_Page;
void DRF1609H_classdef::Receive(uint8_t *data)
{
//	if(DRF_REC_Data[0]=='K')
//	{
//    if(DRF_REC_Data[1] == 0x53 && DRF_REC_Data[6]==0x45)
//		{
//			idea_ctrl = DRF_REC_Data[2]<<24 | DRF_REC_Data[3]<<16 | DRF_REC_Data[4]<<8 | DRF_REC_Data[5]<<0;
//			if(last_idea_ctrl != idea_ctrl)
//			{
//				if(DRF_REC_Data[5]!=Last_Num)
//				{
//					switch(DRF_REC_Data[4])
//					{
//						case 0x00000000:break;
//						case 0xff:Clamp.Init_Flag = 1;break;
//						case 0x01:Clamp.Tar_Ring = Tar_LTen;Clamp.Place_Flag = 1;break;
//						case 0x02:Clamp.Tar_Ring = Tar_MTen;Clamp.Place_Flag = 1;break;
//						case 0x03:Clamp.Tar_Ring = Tar_RTen;Clamp.Place_Flag = 1;break;
//						case 0x04:Clamp.Tar_Ring = Tar_LThirty;Clamp.Place_Flag = 1;break;
//						case 0x05:Clamp.Tar_Ring = Tar_RThirty;Clamp.Place_Flag = 1;break;
//						case 0x06:Clamp.Tar_Ring = Tar_MSeventy;Clamp.Place_Flag = 1;break;
//						case 0x07:Clamp.Tar_Ring = Tar_DLThirty;Clamp.Place_Flag = 1;break;
//						case 0x08:Clamp.Tar_Ring = Tar_DRThirty;Clamp.Place_Flag = 1;break;
//					}
//				}
//			}
//			Last_Num = DRF_REC_Data[5];
//			last_idea_ctrl = idea_ctrl;
//		}
//	}
	if(DRF_REC_Data[0] == 0x53 && DRF_REC_Data[5]==0x45)
	{
		Read_Flag=0;
		switch(DRF_REC_Data[1])
		{
			case 1:
			{
				switch(DRF_REC_Data[2])
				{
					case 0://发射
					{
						if(Last_Tar != DRF_REC_Data[3])
						{
							switch(DRF_REC_Data[3])
							{
								case 0x00000000:break;
								case 0x01:Clamp.Tar_Ring = Tar_LTen;Clamp.Place_Flag = 1;break;
								case 0x02:Clamp.Tar_Ring = Tar_MTen;Clamp.Place_Flag = 1;break;
								case 0x03:Clamp.Tar_Ring = Tar_RTen;Clamp.Place_Flag = 1;break;
								case 0x04:Clamp.Tar_Ring = Tar_LThirty;Clamp.Place_Flag = 1;break;
								case 0x05:Clamp.Tar_Ring = Tar_RThirty;Clamp.Place_Flag = 1;break;
								case 0x06:Clamp.Tar_Ring = Tar_MSeventy;Clamp.Place_Flag = 1;break;
								case 0x07:Clamp.Tar_Ring = Tar_DLThirty;Clamp.Place_Flag = 1;break;
								case 0x08:Clamp.Tar_Ring = Tar_DRThirty;Clamp.Place_Flag = 1;break;
								case 0x0A:Clamp.Tar_Ring = Tar_MTwenty_Five;Clamp.Place_Flag = 1;break;
							}
						}
						else if(Last_Num != DRF_REC_Data[4])
						{
							switch(DRF_REC_Data[3])
							{
								case 0x00000000:break;
								case 0x01:Clamp.Tar_Ring = Tar_LTen;Clamp.Place_Flag = 1;break;
								case 0x02:Clamp.Tar_Ring = Tar_MTen;Clamp.Place_Flag = 1;break;
								case 0x03:Clamp.Tar_Ring = Tar_RTen;Clamp.Place_Flag = 1;break;
								case 0x04:Clamp.Tar_Ring = Tar_LThirty;Clamp.Place_Flag = 1;break;
								case 0x05:Clamp.Tar_Ring = Tar_RThirty;Clamp.Place_Flag = 1;break;
								case 0x06:Clamp.Tar_Ring = Tar_MSeventy;Clamp.Place_Flag = 1;break;
								case 0x07:Clamp.Tar_Ring = Tar_DLThirty;Clamp.Place_Flag = 1;break;
								case 0x08:Clamp.Tar_Ring = Tar_DRThirty;Clamp.Place_Flag = 1;break;
								case 0x0A:Clamp.Tar_Ring = Tar_MTwenty_Five;Clamp.Place_Flag = 1;break;
							}
						}
					}
					break;
					
					case 1:
					{
						if(Last_Tar != DRF_REC_Data[3])
						{
							switch(DRF_REC_Data[3])
							{
								case 0x00000000:break;
								case 0x01:Clamp.Tar_Ring = Tar_LTen;break;
								case 0x02:Clamp.Tar_Ring = Tar_MTen;;break;
								case 0x03:Clamp.Tar_Ring = Tar_RTen;break;
								case 0x04:Clamp.Tar_Ring = Tar_LThirty;break;
								case 0x05:Clamp.Tar_Ring = Tar_RThirty;break;
								case 0x06:Clamp.Tar_Ring = Tar_MSeventy;break;
								case 0x07:Clamp.Tar_Ring = Tar_DLThirty;break;
								case 0x08:Clamp.Tar_Ring = Tar_DRThirty;break;
								case 0x0A:Clamp.Tar_Ring = Tar_MTwenty_Five;break;
							}
						}
						else if(Last_Num != DRF_REC_Data[4])
						{
							switch(DRF_REC_Data[3])
							{
								case 0x00000000:break;
								case 0x01:Clamp.Tar_Ring = Tar_LTen;break;
								case 0x02:Clamp.Tar_Ring = Tar_MTen;break;
								case 0x03:Clamp.Tar_Ring = Tar_RTen;break;
								case 0x04:Clamp.Tar_Ring = Tar_LThirty;break;
								case 0x05:Clamp.Tar_Ring = Tar_RThirty;break;
								case 0x06:Clamp.Tar_Ring = Tar_MSeventy;break;
								case 0x07:Clamp.Tar_Ring = Tar_DLThirty;break;
								case 0x08:Clamp.Tar_Ring = Tar_DRThirty;break;
								case 0x0A:Clamp.Tar_Ring = Tar_MTwenty_Five;break;
							}
						}
					}
					break;
				}
			}
			Read_Flag=1;
			break;
			
			case 2:
			{
				switch(DRF_REC_Data[2])
				{
					case 1://瞄准
					{
						if(Last_Tar != DRF_REC_Data[3])
						{
							switch(DRF_REC_Data[3])
							{
								case 0x00000000:break;
								case 0x01:Clamp.Tar_Ring = Tar_LTen;break;
								case 0x02:Clamp.Tar_Ring = Tar_MTen;;break;
								case 0x03:Clamp.Tar_Ring = Tar_RTen;break;
								case 0x04:Clamp.Tar_Ring = Tar_LThirty;break;
								case 0x05:Clamp.Tar_Ring = Tar_RThirty;break;
								case 0x06:Clamp.Tar_Ring = Tar_MSeventy;break;
								case 0x07:Clamp.Tar_Ring = Tar_DLThirty;break;
								case 0x08:Clamp.Tar_Ring = Tar_DRThirty;break;
								case 0x0A:Clamp.Tar_Ring = Tar_MTwenty_Five;break;
							}
						}
						else if(Last_Num != DRF_REC_Data[4])
						{
							switch(DRF_REC_Data[3])
							{
								case 0x00000000:break;
								case 0x01:Clamp.Tar_Ring = Tar_LTen;break;
								case 0x02:Clamp.Tar_Ring = Tar_MTen;break;
								case 0x03:Clamp.Tar_Ring = Tar_RTen;break;
								case 0x04:Clamp.Tar_Ring = Tar_LThirty;break;
								case 0x05:Clamp.Tar_Ring = Tar_RThirty;break;
								case 0x06:Clamp.Tar_Ring = Tar_MSeventy;break;
								case 0x07:Clamp.Tar_Ring = Tar_DLThirty;break;
								case 0x08:Clamp.Tar_Ring = Tar_DRThirty;break;
								case 0x0A:Clamp.Tar_Ring = Tar_MTwenty_Five;break;
							}
						}
					}
					Read_Flag=1;
					break;
					
					case 2://初始化
					{
						if(Last_Mode != DRF_REC_Data[2])
						{
							Clamp.Init_Flag = 1;
							Clamp.Pick_Flag = 0;
							Clamp.Place_Point_Flag = 0;
							Clamp.Place_Flag = 0;
							Clamp.step = 0;
						}
					}
					break;
					
					case 3://取环
					{
						if(Last_Mode != DRF_REC_Data[2])
						{
							if(Clamp.Init_Flag || Clamp.Pick_Flag ||\
							 Clamp.Place_Point_Flag || Clamp.Place_Flag){}
							 else
							 {
								 Clamp.Pick_Flag = 1;
							 }
						}
					}
					break;
				}
				Read_Flag=1;
			}
			break;
		}
		Last_Page = DRF_REC_Data[1];
		Last_Mode = DRF_REC_Data[2];
		Last_Tar = DRF_REC_Data[3];
		Last_Num = DRF_REC_Data[4];
		last_idea_ctrl = idea_ctrl;
		DevicesMonitor.Update(Frame_VISION);
		
	}
}

void DRF1609H_classdef::DRT_Printf(const char *str, ...)
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
			HAL_UART_Transmit_IT(&huart8, (uint8_t *)buffer, len+3);
//	}
    while(huart8.gState != HAL_UART_STATE_READY);	//等待发送完毕
}

////
////功能[3] 模块[4] 数据[5]~[8] 
//uint8_t js_S;
//void DRF1609H_classdef::Send(uint32_t idea)
//{
//	DRF_SEN_Data[0] = 'K';
//	DRF_SEN_Data[1] = 0x53;
//	DRF_SEN_Data[2] = idea>>24;
//	DRF_SEN_Data[3] = idea>>16;
//	DRF_SEN_Data[4] = idea>>8;
//	DRF_SEN_Data[5] = idea>>0;
//	DRF_SEN_Data[6] = 0x45;
//	HAL_UART_Transmit_DMA(&huart8 , DRF_SEN_Data, DRF_SEN_SIZE);
//}
