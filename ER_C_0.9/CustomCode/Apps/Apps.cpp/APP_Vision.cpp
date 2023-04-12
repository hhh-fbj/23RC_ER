#include "APP_Vision.h"
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "cmath"

#include "System_DataPool.h"



/* Private macros ------------------------------------------------------------*/
// --- 是否使用卡尔曼
#define USE_YAW_KALMAN  1
#define USE_PIT_KALMAN	0

#define CRC_CHECK_LEN  14
// --- 卡尔曼滤波结果
#define KF_ANGLE 0
#define KF_SPEED 1
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private function declarations ---------------------------------------------*/
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);
const unsigned char MyCRC8Tab[] = {	
		0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
		157,195,33,127,252,162,64,30, 95,1,227,189,62,96,130,220,
		35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
		190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
		70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
		219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
		101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
		248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
		140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
		17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
		175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
		50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
		202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
		87,9,235,181,54,104,138,212,149,203, 41,119,244,170,72,22,
		233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
		116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53
	};

void Bubble_sort(float arr[], int size);
/* function prototypes -------------------------------------------------------*/
/**
 * @brief      Initialize Vision Class
 * @param[in]  None
 * @retval     None
 */
Vision_classdef::Vision_classdef()
{
	
}



/**
 * @brief      打包数据发送至PC
 * @param[in]  Pack
 * @retval     None
 */
void Vision_classdef::SendToPC(VisionSendMsg_u *pack2vision)
{
	if(aim == 1){Vision.Use_Flag = 0;}
//	pack2vision->Pack.start_tag = 'S';
	pack2vision->Pack.detection = aim;
//	pack2vision->Pack.crc = Checksum_CRC8(Send_Msg.data+1, sizeof(Send_Msg.Pack)-3);
//	pack2vision->Pack.end_tag = 'E';

	HAL_UART_Transmit_DMA(&huart6 , pack2vision->data, sizeof(pack2vision->Pack));
}
/**
 * @brief      对接收的数据解包
 * @param[in]  data
 * @retval     None
 */

void Vision_classdef::RecvFromPC(uint8_t *data, uint16_t ReceiveLen)
{
	float qingling[20]={0};
	float duibi[20]={0};
	int	zongshu[20]={0},max=0,chuzongzhi=0;
	DevicesMonitor.Get_FPS(&TIME, &FPS);
	CRCBuf = Checksum_CRC8(Recv_Msg.data+1, sizeof(Recv_Msg.Pack)-3);
	if(Recv_Msg.Pack.start_Tag == 'S' && Recv_Msg.Pack.end_tag == 'E' && Send_Msg.Pack.detection==1)
	{
		if(Recv_Msg.Pack.Yaw == 0 || abs(Recv_Msg.Pack.Yaw) >= 360)
		{
				
		}
		else
		{
			Yaw_Reserve[Reserve_Num] = Recv_Msg.Pack.Yaw;
			Depth_Reserve[Reserve_Num] = Recv_Msg.Pack.Depth;
			Reserve_Num++;
			if(Reserve_Num==20)
			{
				Use_Yaw = 0;
				Reserve_Num = 0;
				Send_Msg.Pack.detection = aim = 0;
				// for(int i=0;i<20;i++)
				// {
				// 	Use_Yaw += Yaw_Reserve[i];
				// 	Use_Depth += Depth_Reserve[i];
				// }
				// Use_Yaw/=20;Use_Depth/=20;
				// Bubble_sort(Yaw_Reserve,20);
				// Use_Yaw = (Yaw_Reserve[10]+Yaw_Reserve[11])/2;
				
				// 	Use_Yaw += Yaw_Reserve[2];
				// 	Use_Yaw += Yaw_Reserve[4];
				// 	Use_Yaw += Yaw_Reserve[6];
				// 	Use_Yaw += Yaw_Reserve[8];
				// 	Use_Yaw += Yaw_Reserve[10];
				// 	Use_Yaw += Yaw_Reserve[12];
				// 	Use_Yaw += Yaw_Reserve[14];
				// 	Use_Yaw += Yaw_Reserve[16];
				// 	Use_Yaw += Yaw_Reserve[18];
				// 	Use_Yaw/=9;
				
				// Bubble_sort(Yaw_Reserve,20);
				for(int j = 0; j < 20 ;j++)
				{
						qingling[j] = duibi[j] = Yaw_Reserve[j];
				}
				
				for(int i=0; i<20 ; i++)
				{
					if(qingling[i] != 0)
					{
						for(int l=0; l<20 ; l++)
						{
							if(duibi[i] >= Yaw_Reserve[l]+1 || duibi[i] <= Yaw_Reserve[l]-1)
							{
							}
							else
							{
								qingling[l] = 0;
								zongshu[i]++;
							}
						}
					}
					else
					{
					}
				}
						
				max = zongshu[0];chuzongzhi = 0;
				for(int k = 0; k<20 ; k++)
				{
					if(zongshu[k] > max){max = zongshu[k];chuzongzhi = k;}
				}
						
				for(int m = 0; m<20 ; m++)
				{
					if(duibi[chuzongzhi] >= Yaw_Reserve[m]+1 || duibi[chuzongzhi] <= Yaw_Reserve[m]-1)
					{
					}
					else
					{
						Use_Yaw+=Yaw_Reserve[m];
					}
				}
				Use_Yaw/=max;
			
				Use_Flag = 1;
			}
		}
	}
}

void Vision_classdef::Aim_LockDetection(void)
{
	aim = 1;
	Use_Flag = 0;
	Reserve_Num = 0;
	Use_Yaw = 0;
	Use_Depth = 0;
}


/**
  * @Data       2019-04-02 17:44
  * @brief      CRC8校验
  * @param[in]  buf
  * @param[in]  len
  * @retval     None
  */
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len)
{	
	uint8_t check = 0;

	while(len--)
	{
		check = MyCRC8Tab[check^(*buf++)];
	}
	return (check) & 0xff;
}


void Bubble_sort(float arr[], int size)
{
	int j,i,tem;
	for (i = 0; i < size-1;i ++)//size-1是因为不用与自己比较，所以比的数就少一个
	{
		int count = 0;
		for (j = 0; j < size-1 - i; j++)	//size-1-i是因为每一趟就会少一个数比较
		{
			if (arr[j] > arr[j+1])//这是升序排法，前一个数和后一个数比较，如果前数大则与后一个数换位置
			{
				tem = arr[j];
				arr[j] = arr[j+1];
				arr[j+1] = tem;
				count = 1;
			}
		}
		if (count == 0)			//如果某一趟没有交换位置，则说明已经排好序，直接退出循环
		break;	
	}
}


 
