
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

#include "BSP_CAN.h"

#include "DEV_AIMU.h"

#include "System_DataPool.h"
#include "Wolf_Infantry.h"

#include "APP_Auto.h"

/* Private define ------------------------------------------------------------*/
#define USE_RUDDER_RESET 1
#define REDUCTION_RATIOE 3//1:3
#define VALUE_CIRCLE 98304//32,768
#define DEGREE_TURN_CIRCLE 273.06666666666666666666666666667

#define CHASSIS_MAX_SPEED 12000  // 底盘驱动轮最大速度
#define CHASSIS_MAX_VW    0.6*CHASSIS_MAX_SPEED  // 底盘旋转最大速度


/*------------------------------------------------------------ 初始化 ------------------------------------------------------------*/
Chassis_classdef::Chassis_classdef()
{
	//转向轮 2006
	/*--- 外环 位置 PID -------------------------------------------------------------------------*/
    RUD_PID[Front_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
    RUD_PID[Left_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
    RUD_PID[Right_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
//    /*--- 内环 速度 PID -------------------------------------------------------------------------*/
    RUD_PID[Front_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[Front_Rud][PID_Inner].I_SeparThresh = 800;
    RUD_PID[Left_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[Left_Rud][PID_Inner].I_SeparThresh = 800;
    RUD_PID[Right_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[Right_Rud][PID_Inner].I_SeparThresh = 800;
	
	//全场定位
	/*--- 外环 位置 PID -------------------------------------------------------------------------*/
    POS_PID[Posture_X][PID_Outer].SetPIDParam(2.5f, 0.0f, 0.0f, 4000, 15000, 0.002f);POS_PID[Posture_X][PID_Outer].DeadZone = 1;//4 11 0.0003
    POS_PID[Posture_X][PID_Outer].a_p = 4;POS_PID[Posture_X][PID_Outer].b_p = 11;POS_PID[Posture_X][PID_Outer].c_p = 0.0002;
    POS_PID[Posture_Y][PID_Outer].SetPIDParam(2.8f, 0.0f, 0.0f, 4000, 15000, 0.002f);POS_PID[Posture_Y][PID_Outer].DeadZone = 1;//3.5		6 0.003	
    POS_PID[Posture_Y][PID_Outer].a_p = 3.5;POS_PID[Posture_Y][PID_Outer].b_p = 6;POS_PID[Posture_Y][PID_Outer].c_p = 0.0003;
    POS_PID[Posture_Z][PID_Outer].SetPIDParam(150.0f, 0.0f, 0.0f, 2000, 15000, 0.002f);POS_PID[Posture_Z][PID_Outer].DeadZone = 0.5;
    /*--- 内环 速度 PID 没用 -------------------------------------------------------------------------*/
    POS_PID[Posture_X][PID_Inner].SetPIDParam(0.0f, 0.0f, 0.0f, 1000, 10000, 0.002f);
    POS_PID[Posture_Y][PID_Inner].SetPIDParam(0.0f, 0.0f, 0.0f, 1000, 10000, 0.002f);
    POS_PID[Posture_Z][PID_Inner].SetPIDParam(0.0f, 0.0f, 0.0f, 1000, 10000, 0.002f);

    //修复环
    /*--- 外环 位置 PID -------------------------------------------------------------------------*/
    Repair_PID[PID_Outer].SetPIDParam(360.0f, 0.0f, 0.0f, 4000, 10000, 0.002f);Repair_PID[PID_Outer].DeadZone = 1;
    /*--- 内环 速度 PID 没用 -------------------------------------------------------------------------*/
    Repair_PID[PID_Inner].SetPIDParam(0.0, 0.0f, 0.0f, 1000, 10000, 0.002f);Repair_PID[PID_Inner].DeadZone = 1;

    //激光双环——前进环/旋转环
    Laser_PID[0].SetPIDParam(0.48, 0.012f, 0.0f, 200, 660, 0.002f);Laser_PID[0].DeadZone = 10;
    Laser_PID[1].SetPIDParam(0.5f, 0.0f, 0.0f, 4000, 10000, 0.002f);Laser_PID[1].DeadZone = 30;

		//全场定位 new idea
		POS_X_PID.SetPIDParam(5.0f, 0.0f, 0.0f, 4000, 15000, 0.002f);POS_X_PID.DeadZone = 1;//4 11 0.0003
    POS_X_PID.a_p = 8;POS_X_PID.b_p = 14;POS_X_PID.c_p = 0.00001;

    for(int i=0;i<3;i++)
    {
        FRONT[i] = 0;//为0时，轮子朝前
        XYZ_PreTar_Angle[i] = 90;//默认指向为XYZ_PreTar_Angle设定值
    }

    Pos_Target[0] = 0;
    Pos_Target[1] = 0;
    Pos_Target[2] = 0;

    AF_WtoXY_Stand = 3;
}


/*------------------------------------------------------------ 控制 ------------------------------------------------------------*/
//底盘总控制函数
float P_pid[3];
float I_MAX_CS;
void Chassis_classdef::Control()
{
//		DRV_PID[0].SetPIDParam(P_pid[0], P_pid[1], P_pid[2], I_MAX_CS, 10000, 0.002f);
//		DRV_PID[1].SetPIDParam(P_pid[0], P_pid[1], P_pid[2], I_MAX_CS, 10000, 0.002f);
//		DRV_PID[2].SetPIDParam(P_pid[0], P_pid[1], P_pid[2], I_MAX_CS, 10000, 0.002f);
	
    //微动开关检测
    Sensor();
	
    //问题检测
    if(ProblemDetection()){return;}

    //底盘更新数据
    ChassisTar_Update();
		
    Send_Data();
}

void Chassis_classdef::Sensor(void)
{
    //右
    EdgeDete[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);//右
    EdgeDete[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);//左
    //前
    EdgeDete[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);//右
    EdgeDete[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);//左
    //左
    EdgeDete[4] = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_5);//右
    EdgeDete[5] = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);//左
		//后	
    EdgeDete[6] = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7);//右
    EdgeDete[7] = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_2);//左

    Last_Ready_Flag = Ready_Flag;
    Ready_Flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
}

uint8_t Chassis_classdef::ProblemDetection(void)
{
    if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
    {
        for(uint8_t i = 0 ; i < 3 ; i++)
        {
            RUD_Motor[i].Out = 0;
            RUD_PID[i][PID_Outer].Reset();
            RUD_PID[i][PID_Inner].Reset();
						DRV_PID[i].Reset();
						DRV_Motor[i].Out = 0;
            Cal_Speed[i] = 0;
            Buzzer.error = 0;
        }
				Auto.CH_F_X=1,Auto.CH_HL_X = 1,Auto.CH_HR_X = 1;
        Send_Data();
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
        return 1;
    }
    else if(DevicesMonitor.Get_State(CHAS_RUD1_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUD2_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUD3_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider1_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider2_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider3_MONITOR) == Off_line || \
				DevicesMonitor.Get_State(CHASSIS_DRV1_MONITOR) || \
        DevicesMonitor.Get_State(CHASSIS_DRV2_MONITOR) || \
        DevicesMonitor.Get_State(CHASSIS_DRV3_MONITOR))
    {
        for(uint8_t i = 0 ; i < 3 ; i++)
        {
            RUD_Motor[i].	Out = 0;
            RUD_PID[i][PID_Outer].Reset();
            RUD_PID[i][PID_Inner].Reset();
						DRV_PID[i].Reset();
						DRV_Motor[i].Out = 0;
            Cal_Speed[i] = 0;
            Buzzer.error = 0;
						Auto.CH_F_X=1,Auto.CH_HL_X = 1,Auto.CH_HR_X = 1;
        }
        Send_Data();
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
        return 1;
    }
    else
    {
        return 0;
    }
}

float P;
float X_cs_speed;
float zhuan_jiao_chi;
float zhongjin_zhu_houtui_x;
float zhongjin_zhu_houtui_y;
float zhongjin_zhu_houtui_z;
void Chassis_classdef::ChassisTar_Update()
{
    switch ((int)Mode)
    {
        case CHAS_TransiMode:
				{
            Pos_Target[0] = POS_PID[Posture_X][PID_Outer].Target = Auto.Posture.POS_X();//Auto.Posture.POS_W();
            Pos_Target[1] = POS_PID[Posture_Y][PID_Outer].Target = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
            Pos_Target[2] = POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();//Auto.Posture.POS_W();
						zhuan_jiao_chi = Auto.Posture.POS_W();
							zhongjin_zhu_houtui_x=Auto.Posture.POS_X();
							zhongjin_zhu_houtui_y=Auto.Posture.POS_Y();
							zhongjin_zhu_houtui_z=Auto.Posture.POS_W();
            Auto.Vx = Auto.Vy = Auto.Vw = 0;
            if(DR16.Get_LY_Norm()>-110)
						{
							Mode = Next_Mode;
						}
            for(uint8_t i = 0 ; i < 3 ; i++)
            {
                RUD_Motor[i].Out = 0;
                RUD_PID[i][PID_Outer].Reset();
                RUD_PID[i][PID_Inner].Reset();
                Cal_Speed[i] = 0;
								DRV_PID[i].Reset();
								DRV_Motor[i].Out = 0;
            }
            Last_Mode = CHAS_TransiMode;
				}
        break;

        case CHAS_ControlMode://视觉控制底盘
            Process(0, 0, 0);
            Last_Mode = CHAS_ControlMode;
        break;
    
        case CHAS_MoveMode:
				{
						Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
//					  if(CTRL_DR16.Get_ExptVy())
//						{
//							Process(CTRL_DR16.Get_ExptVx(), X_cs_speed, CTRL_DR16.Get_ExptVw());
//						}
//						else
//						{
//							Process(CTRL_DR16.Get_ExptVx(), 0, CTRL_DR16.Get_ExptVw());
//						}
            Last_Mode = CHAS_MoveMode;
				}
        break;

        case CHAS_LockMode:
            Process(0, 0, 0);
            Last_Mode = CHAS_LockMode;
        break;

        case CHAS_AutoMode://跑自动
				{
            Auto.Process();
            switch(NO_PostureMode)
            {
                case 0://取环去中间
                    POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
                    POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
                    POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();

                    Process(POS_PID[Posture_X][PID_Outer].Cal(), POS_PID[Posture_Y][PID_Outer].Cal(), POS_PID[Posture_W][PID_Outer].Cal());
                break;
                
                case 1:
                    Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
                break;
                
                case 2://会摆X
                    Process(0, 0, 0);
                break;
								
                case 4://不会摆X
                    Process(0, 0, 0);
                break;
                    
                case 3:
                    Process(Auto.Vx, Auto.Vy, Auto.Vw);
                break;
								
                case 55://新想法
                    P = atan2(POS_PID[Posture_Y][PID_Outer].Target-Auto.Posture.POS_Y(), POS_PID[Posture_X][PID_Outer].Target-Auto.Posture.POS_X());
                    POS_X_PID.Target = sqrt(pow((POS_PID[Posture_Y][PID_Outer].Target-Auto.Posture.POS_Y()),2)+pow((POS_PID[Posture_X][PID_Outer].Target-Auto.Posture.POS_X()),2));
                    POS_X_PID.Current = 0;
//										if(P<0){P+=2*PI;}
                    POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
								
										
                    //根据yaw轴限xy轴速度
                    AF_WtoXY = AF_WtoXY_Stand/abs(POS_PID[Posture_W][PID_Outer].Target - POS_PID[Posture_W][PID_Outer].Current);
                    if(abs(AF_WtoXY)>=1){AF_WtoXY = 1;}
//										Process(POS_X_PID.Cal()*arm_sin((90-Auto.Posture.POS_W())*0.01745329251994329576923690768489-P), \
//                    POS_X_PID.Cal()*arm_cos((90-Auto.Posture.POS_W())*0.01745329251994329576923690768489-P), POS_PID[Posture_W][PID_Outer].Cal());

											Process((POS_X_PID.Cal()*arm_cos(P+Auto.Posture.POS_W()*0.01745329251994329576923690768489)), \
											POS_X_PID.Cal()*arm_sin(P+Auto.Posture.POS_W()*0.01745329251994329576923690768489), POS_PID[Posture_W][PID_Outer].Cal());

									break;

                case 66://初始去取环
                    POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
                    POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
                    POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();
                    //根据yaw轴限xy轴速度
                    AF_WtoXY = AF_WtoXY_Stand/abs(POS_PID[Posture_W][PID_Outer].Target - POS_PID[Posture_W][PID_Outer].Current);
                    if(abs(AF_WtoXY)>=1){AF_WtoXY = 1;}
                    Process(AF_WtoXY*AF_WtoXY*POS_PID[Posture_X][PID_Outer].Cal()-880, AF_WtoXY*AF_WtoXY*POS_PID[Posture_Y][PID_Outer].Cal(), POS_PID[Posture_W][PID_Outer].Cal());
                break;
                
                case 99:
                    Process(0, 0, 0);
                break;
                
            }
            Last_Mode = CHAS_AutoMode;
				}
        break;

        case CHAS_IMUMode://使用C陀螺仪/全场定位陀螺仪进行修正
				{
            if(Last_Mode != CHAS_IMUMode){Repair_PID[PID_Outer].Target = RecvCan_Msg.Pack.Yaw_Z;}//Auto.Posture.POS_W();}
            if((CTRL_DR16.Get_ExptVx() != 0 || CTRL_DR16.Get_ExptVy() != 0) && CTRL_DR16.Get_ExptVw() == 0)
            {
                Repair_PID[PID_Outer].Current = RecvCan_Msg.Pack.Yaw_Z;//Auto.Posture.POS_W();
                Repair_PID[PID_Inner].Target = Repair_PID[PID_Outer].Cal();
                Repair_PID[PID_Inner].Current = RecvCan_Msg.Pack.Gz;
                Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), -Repair_PID[PID_Inner].Cal());//*(sqrt(pow(CTRL_DR16.Get_ExptVx(),2)+pow(CTRL_DR16.Get_ExptVy(),2))/16970));
            }
            else
            {
                Repair_PID[PID_Outer].Target = RecvCan_Msg.Pack.Yaw_Z;//.POS_W();
                Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
            }
            Last_Mode = CHAS_IMUMode;
				}
        break;

        case CHAS_LaserMode://使用激光测距进行修正
				{
            Laser_PID[0].Target = 32768+32665;
            Laser_PID[0].Current = Auto.Analog.LaserRanging[9]+Auto.Analog.LaserRanging[8];
            Laser_PID[1].Target = 32665-Auto.Analog.LaserRanging[8];
            Laser_PID[1].Current = 32768-Auto.Analog.LaserRanging[9];
            Process(CTRL_DR16.Get_ExptVx(), Laser_PID[0].Cal(), Laser_PID[1].Cal());
            Last_Mode = CHAS_LaserMode;
				}
        break;

        case CHAS_PostureMode://
				{
//            Pos_Target[0] += CTRL_DR16.Get_ExptVx()*0.0001;
//            Pos_Target[1] += CTRL_DR16.Get_ExptVy()*0.0001;
//            Pos_Target[2] = 0;//CTRL_DR16.Get_ExptVw()*0.00001;
//            POS_PID[Posture_X][PID_Outer].Target = Pos_Target[0];//Auto.Posture.POS_W();
//            POS_PID[Posture_Y][PID_Outer].Target = Pos_Target[1];//Auto.Posture.POS_W();
//            POS_PID[Posture_W][PID_Outer].Target = Pos_Target[2];

//            POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
//            POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
//            POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();
//            Process(POS_PID[Posture_X][PID_Outer].Cal(), POS_PID[Posture_Y][PID_Outer].Cal(), POS_PID[Posture_W][PID_Outer].Cal());
						
				
						if(CTRL_DR16.Get_ExptVx()&&CTRL_DR16.Get_ExptVy()){}
							else{POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();}
						if(CTRL_DR16.Get_ExptVw())
						{
							POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
							POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();
							zhuan_jiao_chi = Auto.Posture.POS_W();
						}
						else
						{
							POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
							POS_PID[Posture_W][PID_Outer].Target = zhuan_jiao_chi;
						}
						Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), POS_PID[Posture_W][PID_Outer].Cal()+CTRL_DR16.Get_ExptVw());//CTRL_DR16.Get_ExptVw());

						
						
						
//						P = atan2(CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVx());
//						POS_X_PID.Target = sqrt(pow(CTRL_DR16.Get_ExptVy(),2)+pow(CTRL_DR16.Get_ExptVx(),2));
////										if(P<0){P+=2*PI;}
//						POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
//				
//						if(CTRL_DR16.Get_ExptVx()&&CTRL_DR16.Get_ExptVy()){}
//						else{POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();}
//					if(CTRL_DR16.Get_ExptVw())
//					{
//						POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
//						POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();
//						zhuan_jiao_chi = Auto.Posture.POS_W();
//					}
//					else
//					{
//						POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
//						POS_PID[Posture_W][PID_Outer].Target = zhuan_jiao_chi;
//					}
////						Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), POS_PID[Posture_W][PID_Outer].Cal()+CTRL_DR16.Get_ExptVw());//CTRL_DR16.Get_ExptVw());

//					
//					Process((POS_X_PID.Target*arm_cos(P+Auto.Posture.POS_W()*0.01745329251994329576923690768489)), \
//					POS_X_PID.Target*arm_sin(P+Auto.Posture.POS_W()*0.01745329251994329576923690768489), \
//					POS_PID[Posture_W][PID_Outer].Cal()+CTRL_DR16.Get_ExptVw());


						
            Last_Mode = CHAS_PostureMode;
					}
        break;
				
        case CHAS_DisableMode:
				{
            for(uint8_t j = 0 ; j < 3 ; j++)
            {
                RUD_PID[j][PID_Outer].Reset();
                RUD_PID[j][PID_Inner].Reset();
								DRV_PID[j].Reset();
								DRV_Motor[j].Out = 0;
                Cal_Speed[j] = 0;

                RUD_Motor[j].Out = 0;  
                Cal_Speed[j] = 0;
            }
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
            Last_Mode = CHAS_DisableMode;
				}
				break;
						
				case CHAS_FZSPMode:
				{
					if(xxx_flag)
					{
            POS_PID[Posture_X][PID_Outer].Target = zhongjin_zhu_houtui_x;//Auto.Posture.POS_W();
            POS_PID[Posture_Y][PID_Outer].Target = zhongjin_zhu_houtui_y-1030;//Auto.Posture.POS_W();
            POS_PID[Posture_W][PID_Outer].Target = zhongjin_zhu_houtui_z;

            POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
            POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
            POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();
						
				
							if(abs(Auto.Posture.POS_X() - zhongjin_zhu_houtui_x) < 20 &&\
							abs(Auto.Posture.POS_Y() - (zhongjin_zhu_houtui_y-1030)) < 20 &&\
							abs(Auto.Posture.POS_W() - zhongjin_zhu_houtui_z) < 2 )
						{
							Process(0, 0, POS_PID[Posture_W][PID_Outer].Cal());
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);//默认
						}
						else
						{
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
							Process(POS_PID[Posture_X][PID_Outer].Cal(), POS_PID[Posture_Y][PID_Outer].Cal(), POS_PID[Posture_W][PID_Outer].Cal());
						}
					}
					else
					{
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
						if(EdgeDete[2] == GPIO_PIN_RESET &&\
							EdgeDete[3] == GPIO_PIN_RESET)
						{
							zhongjin_zhu_houtui_x=Auto.Posture.POS_X();
							zhongjin_zhu_houtui_y=Auto.Posture.POS_Y();
							zhongjin_zhu_houtui_z=Auto.Posture.POS_W();
							Process(0,1200,0);
						}
						else
						{
							POS_PID[Posture_W][PID_Outer].Target = zhongjin_zhu_houtui_z;
							POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
							Process(0,1200,POS_PID[Posture_W][PID_Outer].Cal());
						}
						
					}
				}
				break;
				
				case CHAS_TQQHMode:
				{
						if(CTRL_DR16.Get_ExptVx()&&CTRL_DR16.Get_ExptVy()){}
						else{POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();}
						if(CTRL_DR16.Get_ExptVw())
						{
							POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
							POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();
							zhuan_jiao_chi = Auto.Posture.POS_W();
						}
						else
						{
							POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();
							POS_PID[Posture_W][PID_Outer].Target = zhuan_jiao_chi;
						}
						Process(800, CTRL_DR16.Get_ExptVy(), POS_PID[Posture_W][PID_Outer].Cal()+CTRL_DR16.Get_ExptVw());//CTRL_DR16.Get_ExptVw());
					}

				break;
    }
}

//底盘数据处理
void Chassis_classdef::Process(float Vx, float Vy, float Vw)
{		
    //--- 速度斜坡	作用?
//    Drv_Slow(&Ramp_Vx, Vx, 10.0f, ACCCCC_VAL, DECCCCC_VAL);//
//    Drv_Slow(&Ramp_Vy, Vy, 10.0f, ACCCCC_VAL, DECCCCC_VAL);
//    Drv_Slow(&Ramp_Vw, Vw*0.5, 6.0f, ACCCCC_VAL*0.6, DECCCCC_VAL*0.6);
    Ramp_Vx=Vx;Ramp_Vy=Vy;Ramp_Vw=Vw;

    //运动解算
    Rudder_Solve(Ramp_Vx, Ramp_Vy, Ramp_Vw, Cal_Speed);
    
    //PID计算
    for(uint8_t i = 0; i < 3; i++)
    {
			if(i == 1){DRV_Motor[i].Out = Cal_Speed[i] * falsh[i];}
			else{DRV_Motor[i].Out = Cal_Speed[i] * falsh[i] * -1;}
        
        RUD_PIDCalc(i);  
    }
}

void Chassis_classdef::Send_Data()
{
    MotorMsgSend(&hcan1, RUD_Motor);
		CAN_Send();
}

/**
 * @brief      底盘速度斜坡
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_classdef::Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal)
{
    if(abs(*rec) - abs(target) < 0)//加速时
    {
        // if(abs(*rec) > 10)
        // {
            slow_Inc = slow_Inc * Accval;//速度提起来的时候增大到5倍
        // }
    }else if(abs(*rec) - abs(target) > 0)
    {
        slow_Inc = slow_Inc * DecVal;//减速时放大15倍
    }

    if(abs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }
}


//轮舵轮运动解算
void Chassis_classdef::Rudder_Solve(float Vx, float Vy, float Vw, float *cal_speed)
{
    float Param = 1.0f;
    float MaxSpeed = 0.0f;

	//速度限制
    Constrain(&Vx, (float)(-CHASSIS_MAX_SPEED), (float)(CHASSIS_MAX_SPEED));
    Constrain(&Vy, (float)(-CHASSIS_MAX_SPEED), (float)(CHASSIS_MAX_SPEED));
    Constrain(&Vw, (float)(-CHASSIS_MAX_VW), (float)(CHASSIS_MAX_VW));

    RudAngle_Calc(Vx, Vy, Vw);
    
    /* 驱动轮 速度解算 ---------------------------------------------------------------------------------*/
    cal_speed[Front_Rud] = sqrt(pow(Vx + Vw,2) + pow(Vy,2));
    cal_speed[Left_Rud] =  sqrt(pow(Vx - Vw*0.5,2) + pow(Vy + Vw*0.5,2));
    cal_speed[Right_Rud] = sqrt(pow(Vx - Vw*0.5,2) + pow(Vy - Vw*0.5,2));

    // 寻找最大速度
    for (uint8_t i = 0; i < 3; i++)
    {
        //RUD_PID[i][PID_Outer].Current = Vw; 
        if(abs(cal_speed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(cal_speed[i]);
        }
    }

    // 速度分配  
    if (MaxSpeed > CHASSIS_MAX_SPEED)
    {
        Param = (float)CHASSIS_MAX_SPEED / MaxSpeed;
    }

    if(abs(RUD_PID[Front_Rud][PID_Outer].Target) <= 1500 &&
        abs(RUD_PID[Left_Rud][PID_Outer].Target) <= 1500 &&
        abs(RUD_PID[Right_Rud][PID_Outer].Target) <= 1500)
    {
        cal_speed[Front_Rud] *= Param;
        cal_speed[Left_Rud] *= Param;
        cal_speed[Right_Rud] *= Param;
//        cal_speed[RB_Rud] *= Param;
    }
}

/**
 * @brief      底盘转电机位置式式PID计算
 * @param[in]  motor
 * @param[in]  target
 * @param[in]  current
 * @note       None
 * @retval     None
 */
void Chassis_classdef::RUD_PIDCalc(uint8_t motor)
{
	RUD_PID[motor][PID_Inner].Target = RUD_PID[motor][PID_Outer].Cal();
	RUD_PID[motor][PID_Inner].Current = RUD_Motor[motor].getSpeed();
	RUD_Motor[motor].Out = RUD_PID[motor][PID_Inner].Cal();
	
//	DRV_PID[motor].Target = Three_Speed[motor];
//	DRV_PID[motor].Current = DRV_Motor[motor].getRpm();
//	DRV_Motor[motor].Out = DRV_PID[motor].Cal();
}

//三轴速度解算四轮舵轮 舵角度 (直角坐标系)
void Chassis_classdef::RudAngle_Calc(float Vx, float Vy, float Vw)
{

    if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        if(Mode == CHAS_LockMode ||  (Mode == CHAS_AutoMode && NO_PostureMode == 2))//轮子45度X型朝向 暂时不用
        {
            XYZ_Angle[0] = 90;
            XYZ_Angle[1] = 45;
            XYZ_Angle[2] = 135;
        }
        else if(0)//轮子45度◇型朝向
        {
            // XYZ_Angle[0] = 135;
            // XYZ_Angle[1] = 45;
            // XYZ_Angle[2] = 135;
            // XYZ_Angle[3] = 45;
        }
        else
        {
            //--- 目标角度为最后速度指向;
            for(uint8_t i = 0 ; i < 3 ; i++)
            {
                XYZ_Angle[i] = XYZ_PreTar_Angle[i];
            }
        }
    }
    else
    {
        //--- 有目标速度的时候才进行舵轮解算的计算
        XYZ_Angle[Front_Rud] = atan2(Vy, Vx + Vw*(Radius))*(180/PI);
        XYZ_Angle[Left_Rud] = atan2(Vy + Vw*(Radius*0.5), Vx - Vw*(Radius*0.5))*(180/PI);
        XYZ_Angle[Right_Rud] = atan2(Vy - Vw*(Radius*0.5), Vx - Vw*(Radius*0.5))*(180/PI);
//        XYZ_Angle[RB_Rud] = atan2(Vy - Vw*(Radius*arm_sin_f32(Ftheta)), Vx - Vw*(Radius*arm_cos_f32(Ftheta)))*(180/PI);
        //--- 无目标速度的时候不使用上一次角度来保存是因为跟随模式下IMU静止的瞬间会产生轻微的Vw速度

        for(uint8_t i = 0 ; i < 3 ; i++)
        {
            XYZ_Angle[i] = XYZ_Angle[i]<0 ? (360+XYZ_Angle[i]) : abs(XYZ_Angle[i]);
        }
    }

    // XYZ_Angle[0] = 135;
    //     XYZ_Angle[1] = 45;
    //     XYZ_Angle[2] = 135;
    //     XYZ_Angle[3] = 45;

    //--- 更改为保存经过劣弧处理的目标角度(现在暂时不用这个上一帧的角度)
    for(uint8_t i = 0 ; i < 3 ; i++) 
    {
        XYZ_PreTar_Angle[i] = XYZ_Angle[i];
    }
    
    //转成对应编码器的度量数值求取误差
    Angle_Treatment();
}

void Chassis_classdef::Angle_Treatment(void)
{
    float Error;
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
			
        //转舵分辨率
        XYZ_Angle[i] *= DEGREE_TURN_CIRCLE;//角度对应的最小刻度
				
        XYZ_Angle[i] += (FRONT[i]+24576);//-24576front朝向转为直角坐标系的90角位置的转换,+24576front朝向转为直角坐标系的180角位置的转换
        XYZ_Angle[i] = XYZ_Angle[i] - (floor(XYZ_Angle[i]/VALUE_CIRCLE)*VALUE_CIRCLE);//过滤
				
				//目标
				XYZ_Angle[i] = 393216+(XYZ_Angle[i]-49152);
//				//误差
				Error = XYZ_Angle[i]-RUD_Encider[i].getTotolAngle();
				
				falsh[i] = 1;
//				//劣弧
				if(Error > 49152)
				{
					XYZ_Angle[i] -= 49152;
					falsh[i] *= -1;
				}
				else if(Error < -49152)
				{
					XYZ_Angle[i] += 49152;
					falsh[i] *= -1;
				}
				else if(Error > 24576 && XYZ_Angle[i]+(Error-49152)>393216+(12000-49152))
				{
					XYZ_Angle[i] -= 49152;
					falsh[i] *= -1;
				}
				else if(Error < -24576 && XYZ_Angle[i]+(Error+49152)<393216+(87000-49152))
				{
					XYZ_Angle[i] += 49152;
					falsh[i] *= -1;
				}
				
				if(XYZ_Angle[i]>=393216+(87000-49152))
				{
					falsh[i] *= -1;
					XYZ_Angle[i] -= 49152;
				}
				else if(XYZ_Angle[i]<=393216+(12000-49152))
				{
					falsh[i] *= -1;
					XYZ_Angle[i] += 49152;
				}
				Error = XYZ_Angle[i]-RUD_Encider[i].getTotolAngle();

				//差值
//				Error = XYZ_Angle[i] - (RUD_Encider[i].getTotolAngle()%VALUE_CIRCLE);
				
//        //劣弧+反转判断
//        Error = XYZ_Angle[i] - (RUD_Encider[i].getTotolAngle()%VALUE_CIRCLE);//(RUD_Encider[i].getTotolAngle() - floor(RUD_Encider[i].getTotolAngle()/VALUE_CIRCLE) * VALUE_CIRCLE);
//        //
				
//        if (Error > 49152)
//        {
//            Error -= VALUE_CIRCLE;
//        }
//        else if (Error < -49152)
//        {
//            Error += VALUE_CIRCLE;
//        }
//				//
//        if(Error > 24576)
//        {
//            Error -= 49152;
//            falsh[i] = -1;
//        }
//        else if(Error <= -24576)
//        {
//            Error += 49152;
//            falsh[i] = -1;
//        }
//        else
//        {
//            falsh[i] = 1;
//        }
        RUD_PID[i][PID_Outer].Target = (int)Error;
    }
}

/*------------------------------------------------------------ 处理相关 ------------------------------------------------------------*/

void Chassis_classdef::CAN_Send(void)
{
//		DRV_Motor[Front_Rud].Set_current(&hcan2, DRV_Motor[Front_Rud].Out);
//		DRV_Motor[Left_Rud].Set_current(&hcan2, DRV_Motor[Left_Rud].Out);
//		DRV_Motor[Right_Rud].Set_current(&hcan2, DRV_Motor[Right_Rud].Out);
		DRV_Motor[Front_Rud].Set_rpm(&hcan2, DRV_Motor[Front_Rud].Out*Auto.CH_F_X);
		DRV_Motor[Left_Rud].Set_rpm(&hcan2, DRV_Motor[Left_Rud].Out*Auto.CH_HL_X);
		DRV_Motor[Right_Rud].Set_rpm(&hcan2, DRV_Motor[Right_Rud].Out*Auto.CH_HR_X);
}
void Chassis_classdef::CAN_Recvd(uint8_t can_rx_data[])
{
	for(int i=0;i<8;i++)
	{
		RecvCan_Msg.data[i] = can_rx_data[i];
	}
}
void Chassis_classdef::Set_Mode(CHAS_CtrlMode_e mode)
{   
    if(Mode != mode)
    {
        Next_Mode = mode;
        Mode = CHAS_TransiMode;
    }
}
