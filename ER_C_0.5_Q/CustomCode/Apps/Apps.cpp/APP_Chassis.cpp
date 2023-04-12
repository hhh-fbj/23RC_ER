
/**
 ------------------------------------------------------------------------------
 * @file    APP_Chassis.cpp
 * @author  Shake
 * @brief   底盘控制
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "APP_Chassis.h"
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "arm_math.h"

#include "ALGO_Kalman.h"

#include "DEV_CIMU.h"

#include "System_DataPool.h"
#include "Wolf_Infantry.h"


/* Private define ------------------------------------------------------------*/
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
#define CHASSIS_MAX_SPEED 9000  // 底盘驱动??最大速度
#define CHASSIS_MAX_VW    9000  // 底盘旋转最大速度

#define CHASSIS_SPEED_L   9000
#define CHASSIS_SPEED_M   12000
#define CHASSIS_SPEED_H   20000


float Radius = 1.0f;  // 圆心??
extKalman_t Kalman_CHASFollow_Speed;

/*------------------------------------------------------------ 初???化 ------------------------------------------------------------*/
Chassis_classdef::Chassis_classdef()
{
	//??杰明电调 速度PID——测??
    DRV_PID[RF_Drv].SetPIDParam(10.0f, 0.0f, 0.0f, 4000, 20000, 0.002f);
    DRV_PID[LF_Drv].SetPIDParam(10.0f, 0.0f, 0.0f, 4000, 20000, 0.002f);
    DRV_PID[LB_Drv].SetPIDParam(10.0f, 0.0f, 0.0f, 4000, 20000, 0.002f);
    DRV_PID[RB_Drv].SetPIDParam(10.0f, 0.0f, 0.0f, 4000, 20000, 0.002f);
}


/*------------------------------------------------------------ 控制 ------------------------------------------------------------*/
//底盘总控制函??
int time_send;
int Three_Speed_Time;
void Chassis_classdef::Control()
{
	time_send++;
    float Param = 1.0f;
    float MaxSpeed = 0.0f;
    //思考下CAN的帧率与驱动总帧率是否相同
    if(DevicesMonitor.Get_State(CHASSIS_AUSARTC_RFLB_MONITOR) ||
        DevicesMonitor.Get_State(CHASSIS_AUSARTC_LFRB_MONITOR) ||
        DevicesMonitor.Get_State(CHASSIS_DRV1_MONITOR) ||
        DevicesMonitor.Get_State(CHASSIS_DRV2_MONITOR) ||
        DevicesMonitor.Get_State(CHASSIS_DRV3_MONITOR) ||
        DevicesMonitor.Get_State(CHASSIS_DRV4_MONITOR))
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            Tar_speed[i] = 0;
        }
        return ;
    }

//    if(Recv_Msg.Pack.mode != 0 && time>0)
//    {
//				time = 0;
//        // 寻找最大速度
//        for (uint8_t i = 0; i < 4; i++)
//        {
//        //  RUD_PID[i][PID_Outer].Current = Vw; 
//            if (abs(Recv_Msg.Pack.speed[i]) > MaxSpeed)
//            {
//                MaxSpeed = abs(Recv_Msg.Pack.speed[i]);
//            }
//        }

//        // 速度分配  
//        if (MaxSpeed > CHASSIS_SPEED_H)
//        {
//            Param = (float)CHASSIS_SPEED_H / MaxSpeed;
//        }

//        Tar_speed[RF_Drv] = Recv_Msg.Pack.speed[RF_Drv] * Param;
//        Tar_speed[LF_Drv] = Recv_Msg.Pack.speed[LF_Drv] * Param;
//        Tar_speed[LB_Drv] = Recv_Msg.Pack.speed[LB_Drv] * Param;
//        Tar_speed[RB_Drv] = Recv_Msg.Pack.speed[RB_Drv] * Param;

//        DRV_Motor[RF_Drv].Set_rpm(&hcan1, Tar_speed[RF_Drv]);
//        DRV_Motor[LF_Drv].Set_rpm(&hcan2, Tar_speed[LF_Drv]);
//        DRV_Motor[LB_Drv].Set_rpm(&hcan2, Tar_speed[LB_Drv]);
//        DRV_Motor[RB_Drv].Set_rpm(&hcan1, Tar_speed[RB_Drv]);
//        
//    }
//		else if(Recv_Msg.Pack.mode == 0)
//		{
//			for (uint8_t i = 0; i < 4; i++)
//			{
//				Tar_speed[i] = 0;
//			}
//		}
//		else
//		{
//			for (uint8_t i = 0; i < 4; i++)
//			{
//				Tar_speed[i] = 0;
//			}
//			DRV_Motor[RF_Drv].Set_rpm(&hcan2, Tar_speed[RF_Drv]);
//			DRV_Motor[LF_Drv].Set_rpm(&hcan2, Tar_speed[LF_Drv]);
//			DRV_Motor[LB_Drv].Set_rpm(&hcan2, Tar_speed[LB_Drv]);
//			DRV_Motor[RB_Drv].Set_rpm(&hcan2, Tar_speed[RB_Drv]);
//		}

        // 寻找最大速度
        for (uint8_t i = 0; i < 4; i++)
        {
        //  RUD_PID[i][PID_Outer].Current = Vw; 
            if (abs(Three_Speed[i]) > MaxSpeed)
            {
                MaxSpeed = abs(Three_Speed[i]);
            }
        }

        // 速度分配  
        if (MaxSpeed > CHASSIS_SPEED_H)
        {
            Param = (float)CHASSIS_SPEED_H / MaxSpeed;
        }

        Tar_speed[RF_Drv] = Three_Speed[RF_Drv] * Param;
        Tar_speed[LB_Drv] = Three_Speed[LB_Drv] * Param;
        Tar_speed[LF_Drv] = Three_Speed[LF_Drv] * Param;
        Tar_speed[RB_Drv] = Three_Speed[RB_Drv] * Param;
				
				
        if(Three_Speed[0]==0&&Three_Speed[0]==0&&Three_Speed[0]==0&&Three_Speed[0]==0)
        {
            Three_Speed_Time++;
        }
        else
        {
            Three_Speed_Time=0;
        }
        
        if(time_send % 2 == 0)// && Three_Speed_Time < 150)
        {
                DRV_Motor[RF_Drv].Set_rpm(&hcan2, Tar_speed[RF_Drv]);
                DRV_Motor[LB_Drv].Set_rpm(&hcan2, Tar_speed[LB_Drv]);
        }
//        else if(Three_Speed_Time < 150)
				else
        {
                DRV_Motor[LF_Drv].Set_rpm(&hcan2, Tar_speed[LF_Drv]);
                DRV_Motor[RB_Drv].Set_rpm(&hcan2, Tar_speed[RB_Drv]);
        }
			
        if(time_send>1000000){time_send=0;}
}


void Chassis_classdef::Receive_AusartC()
{	
    CRCBuf = Checksum_CRC8(Recv_Msg.data+1, sizeof(Recv_Msg.Pack)-3);
    if(Recv_Msg.Pack.start_tag == 'S' && Recv_Msg.Pack.end_tag == 'E' && CRCBuf == Recv_Msg.Pack.crc)
    {
        if(Recv_Msg.Pack.mode == 0 || Recv_Msg.Pack.mode == 2)
        {
            for(int i = 0;i<4;i++)
            {
                Recv_Msg.Pack.speed[i] = 0;
            }
        }
				time++;
    }
    else
    {
		time = 0;
        Recv_Msg.Pack.mode = 0;
        for(int i = 0;i<4;i++)
        {
            Recv_Msg.Pack.speed[i] = 0;
        }
    }
}

/**
  * @Data       2019-04-02 17:44
  * @brief      CRC8校验
  * @param[in]  buf
  * @param[in]  len
  * @retval     None
  */
unsigned char Chassis_classdef::Checksum_CRC8(uint8_t *buf,uint16_t len)
{	
	uint8_t check = 0;

	while(len--)
	{
		check = MyCRC8Tab[check^(*buf++)];
	}

	return (check) & 0xff;
}


void Chassis_classdef::CAN_Recv_RFLB(uint8_t can_rx_data[])
{
    RS1.data[0] = can_rx_data[0];
    RS1.data[1] = can_rx_data[1];
    RS1.data[2] = can_rx_data[2];
    RS1.data[3] = can_rx_data[3];
    RS1.data[4] = can_rx_data[4];
    RS1.data[5] = can_rx_data[5];
    RS1.data[6] = can_rx_data[6];
    RS1.data[7] = can_rx_data[7];
	Three_Speed[0] = RS1.speed[0];
	Three_Speed[2] = RS1.speed[1];
}

void Chassis_classdef::CAN_Recv_LFRB(uint8_t can_rx_data[])
{
    RS2.data[0] = can_rx_data[0];
    RS2.data[1] = can_rx_data[1];
    RS2.data[2] = can_rx_data[2];
    RS2.data[3] = can_rx_data[3];
    RS2.data[4] = can_rx_data[4];
    RS2.data[5] = can_rx_data[5];
    RS2.data[6] = can_rx_data[6];
    RS2.data[7] = can_rx_data[7];
	Three_Speed[1] = RS2.speed[0];
	Three_Speed[3] = RS2.speed[1];
}
