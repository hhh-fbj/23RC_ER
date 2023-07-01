/**
 ------------------------------------------------------------------------------
 * @file    APP_DR16.cpp
 * @author  Shake
 * @brief   DR16 用户控制
 * @version V0.1
 * @date    2021-10-09
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "APP_DR16.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "System_DataPool.h"

#include "DEV_Timer.h"

/* Private macros ------------------------------------------------------------*/
#define LEVER_PERSONAL 1 // ??己用
#define LEVER_STANDARD 2 // 规范??
#define LEVER_MODE     1

#define DR16DATA_NORMAL   0
#define DR16DATA_ABNORMAL 1

#define DR16_MS /* 500 */1000/14

#define ANLE(key_an) (DR16.IsKeyPress(key_an,CLICK)||DR16.IsKeyPress(key_an,PRESS)||DR16.IsKeyPress(key_an,LONGPRESS))
/* Private function declarations ---------------------------------------------*/



/*------------------------------------------------------------ 初???化 ------------------------------------------------------------*/
/**
 * @brief      Initialize CTRL_DR16 Class
 * @param[in]  None
 * @retval     None
 */
CTRL_DR16_classdef::CTRL_DR16_classdef()
{
    Expt.Target_Vx = 0.0f;
    Expt.Target_Vy = 0.0f;
    Expt.Target_Vw = 0.0f;
    Expt.Target_Yaw = 0.0f;
    Expt.Target_Pit = 0.0f;

    Coe.Yaw_RC = /* 0.07f */0.010f;
    Coe.Pit_RC = -0.013f;
}

/*------------------------------------------------------------ RC遥控器控?? ------------------------------------------------------------*/
//拨杆模式更新
uint16_t Reset_cnt;
uint8_t lllccc;
void CTRL_DR16_classdef::LeverMode_Update(void)
{
    switch((uint8_t)DR16.Get_S1_L())
    {
        case Lever_UP:  // --- 左上 -----------------------------------------------
        {
            Auto.Posture_ResFlag = 1;
            // Chassis.Set_Mode(CHAS_LockMode);
            switch((uint8_t)DR16.Get_S2_R())
            {
                // --- PC??控制
                case Lever_UP:/* 左上-右上 START ------------------------------------------*/ 
                {
//                    Chassis.Set_Mode(CHAS_LockMode);
                    Chassis.Set_Mode(CHAS_FZSPMode);
                }
                break;  /* 左上-右上 END ------------------------------------------*/
                
                case Lever_MID:/* 左上-右中 START ------------------------------------------*/ 
                {
//									Chassis.Set_Mode(CHAS_LockMode);
                    Chassis.Set_Mode(CHAS_FZSPMode);
                }
                break;  /* 左上-右中 END ------------------------------------------*/

                case Lever_DOWN:/* 左上-右下 START ------------------------------------------*/ 
                {
                    
									switch(lllccc)
									{
										case 2:
											if(DR16.Get_DW_Norm() >= 330)
											{
												lllccc = 3;
											}
											Chassis.Set_Mode(CHAS_FZSPMode);
										break;
											
										case 3:
										case 5:
											Chassis.Set_Mode(CHAS_FZSPMode);
										break;
											
										default:
											Chassis.Set_Mode(CHAS_FZSPMode);
										break;
									}
//										if(DR16.Get_LY_Norm()<-550&&DR16.Get_RY_Norm()<-550)
//									{
//										Chassis.xxx_flag=1;
//									}
//									else
//									{
//										Chassis.xxx_flag=0;
//									}
                }
                break;  /* 左上-右下 END ------------------------------------------*/
            }
        }
        break;  // 左上 END ---------------------------------------------------

        case Lever_MID:  // --- 左中 ----------------------------------------------
        {
            Auto.Posture_ResFlag = 1;
            switch((uint8_t)DR16.Get_S2_R())
            {
                case Lever_UP:/* 左中-右上 START ------------------------------------------*/ 
                {	

									Chassis.Set_Mode(CHAS_MoveMode);
									
									
//									Chassis.Set_Mode(CHAS_PostureMode);
									
									
//                    Chassis.Set_Mode(CHAS_AutoMode);
//                    if(DR16.Get_DW_Norm() >= 550 && Auto.overFlag == 0){Auto.startFlag=1;}
//                    else if(DR16.Get_DW_Norm() <= -550 && Auto.SX == 0){Auto.SX=99;}
//                    if(DR16.Get_DW_Norm()==0){Auto.SX=0;}
//									switch(ccc)
//									{
//										case 0:
//											if(DR16.Get_DW_Norm() <= -330)
//											{
//												ccc=1;
//											}
//											Chassis.Set_Mode(CHAS_PostureMode);
//										break;
//											
//										case 1:
//											Chassis.Set_Mode(CHAS_TQQHMode);
//											if(DR16.Get_DW_Norm() >= 330)
//											{
//												ccc=2;
//											}
//										break;
//											
//										case 2:
//											Chassis.Set_Mode(CHAS_PostureMode);
//										break;
//									}

//										
//										Chassis.Set_Mode(CHAS_LockMode);//CHAS_LockMode; CHAS_LaserMode
//                    if(DR16.Get_DW_Norm() >= 550)
//                    {
//                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);//正转
//                    }
//                    if(DR16.Get_DW_Norm() <= -550)
//                    {
//                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
//                    }
                }
                break;/* 左中-右上 END ------------------------------------------*/

                case Lever_MID:/* 左中-右中 START ------------------------------------------*/ 
                {
										Chassis.Set_Mode(CHAS_PostureMode);
                    
                }
                break;/* 左中-右中 END ------------------------------------------*/

                case Lever_DOWN:/* 左中-右下 START ------------------------------------------*/ 
                {
									switch(lllccc)
									{
										case 0:
											Chassis.Set_Mode(CHAS_PostureMode);
											if(DR16.Get_DW_Norm() <= -330)
											{
												lllccc = 1;
											}
										break;
											
										case 1:
											Chassis.Set_Mode(CHAS_TQQHMode);
											if(DR16.Get_DW_Norm() >= 330)
											{
												lllccc = 2;
											}
										break;
											
										case 2:
											Chassis.Set_Mode(CHAS_CSMode);
										break;
											
										case 3:
											Chassis.Set_Mode(CHAS_PostureMode);
										break;
											
										default:
											Chassis.Set_Mode(CHAS_PostureMode);
										break;
									}
                }
                break;/* 左中-右下 END ------------------------------------------*/
            }
        }
        break;  // 左中 END ---------------------------------------------------

        case Lever_DOWN: // --- 左下 ----------------------------------------------
        {
           lllccc=0;
            if(Auto.Posture_ResFlag)
            {
//								if(Auto.Posture_ResFlag==0){Auto.Posture.time_long[0] = Get_SystemTimer();}
                Auto.Posture_ResFlag=0;
                Auto.Posture.Devices_Posture_Reset();
                Auto.startFlag=0;
                Auto.text_step = 0;	
								Auto.Posture.TenMi = 1;
            }
						
//						Auto.Posture.Office_Value[Posture_X] = Auto.Posture.Final_Value[Posture_X];
//						Auto.Posture.Office_Value[Posture_Y] = Auto.Posture.Final_Value[Posture_Y];
//						Auto.Posture.Office_Value[Posture_Z] = Auto.Posture.Final_ANGLE;

            Chassis.Set_Mode(CHAS_DisableMode);
						Auto.Posture.error = false;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
            // Robot Reset
            if(DR16.Get_DW_Norm() <= -550)
            {
                Reset_cnt++;
                if(Reset_cnt == 1500)//--- 拨轮打上3s重启
                {
                    //--- ??片???位
                    __set_FAULTMASK(1);    //关闭所有中??
                    HAL_NVIC_SystemReset();//复位
                } 
            }
            else
            {
                Reset_cnt = 0;
            }
        }
        break;  // 左下 END ---------------------------------------------------

        default:
        break;
    }
    RCCtrl_Update();
}
/*------------------------------------------------------------ 控制?? ------------------------------------------------------------*/
//RC控制模式 - 对??????置??标??
void CTRL_DR16_classdef::RCCtrl_Update(void)
{
    Expt.Target_Vx = DR16.Get_LX_Norm()*abs(DR16.Get_LX_Norm())/660*20;//220
    Expt.Target_Vy = DR16.Get_LY_Norm()*abs(DR16.Get_LY_Norm())/660*20;//660
    Expt.Target_Vw = DR16.Get_RX_Norm()*abs(DR16.Get_RX_Norm())/660 * 4;
}

/*------------------------------------------------------------ 处理 ------------------------------------------------------------*/
//数据监测 ———?? - 0:Normal - 1:Abnormal
int8_t CTRL_DR16_classdef::Data_Monitor(void)
{ 
    if((DR16.Get_S1_L() != Lever_UP && DR16.Get_S1_L() != Lever_MID && DR16.Get_S1_L() != Lever_DOWN && DR16.Get_S1_L() != Lever_NONE) || /*<! 左拨?? */
       (DR16.Get_S2_R() != Lever_UP && DR16.Get_S2_R() != Lever_MID && DR16.Get_S2_R() != Lever_DOWN && DR16.Get_S2_R() != Lever_NONE) || /*<! 右拨?? */
       (DR16.Get_RX_Norm() > 660 || DR16.Get_RX_Norm() < -660) ||                                                                    /*<! CH0 */
       (DR16.Get_RY_Norm() > 660 || DR16.Get_RY_Norm() < -660) ||                                                                    /*<! CH1 */
       (DR16.Get_LX_Norm() > 660 || DR16.Get_LX_Norm() < -660) ||                                                                    /*<! CH2 */
       (DR16.Get_LY_Norm() > 660 || DR16.Get_LY_Norm() < -660) ||                                                                    /*<! CH3 */
       (DR16.Get_DW_Norm() > 660 || DR16.Get_DW_Norm() < -660))                                                                      /*<! CH4 */
    {
        return DR16DATA_ABNORMAL;
    }
    else
    {
        return DR16DATA_NORMAL;
    }
}



/*------------------------------------------------------------ 输出 ------------------------------------------------------------*/
//输出数据重置
void CTRL_DR16_classdef::ExptData_Reset(void)
{
    Expt.Target_Vx = 0;
    Expt.Target_Vy = 0;
    Expt.Target_Vw = 0;
    Expt.Target_Yaw = 0;
    Expt.Target_Pit = 0;
}

//获取对???输出数?? ———?? - export data
float CTRL_DR16_classdef::Get_ExptVx()
{
    return Expt.Target_Vx;
}
float CTRL_DR16_classdef::Get_ExptVy()
{
    return Expt.Target_Vy;
}
float CTRL_DR16_classdef::Get_ExptVw()
{
    return Expt.Target_Vw;
}
float CTRL_DR16_classdef::Get_ExptYaw()
{
    return Expt.Target_Yaw;
}
float CTRL_DR16_classdef::Get_ExptPit()
{
    return Expt.Target_Pit;
}
uint8_t CTRL_DR16_classdef::IsAnyOutput()
{
    if((DR16.Get_RX_Norm()||DR16.Get_RY_Norm()||DR16.Get_LX_Norm()||DR16.Get_LY_Norm()||DR16.Get_DW_Norm())==0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
