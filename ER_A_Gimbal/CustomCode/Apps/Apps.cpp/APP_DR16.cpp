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

/* Private macros ------------------------------------------------------------*/
#define DR16DATA_NORMAL   0
#define DR16DATA_ABNORMAL 1


/* Private function declarations ---------------------------------------------*/



/*------------------------------------------------------------ 初�?�化 ------------------------------------------------------------*/
/**
 * @brief      Initialize CTRL_DR16 Class
 * @param[in]  None
 * @retval     None
 */
CTRL_DR16_classdef::CTRL_DR16_classdef()
{
    Expt.L_X = 0.0f;
    Expt.L_Y = 0.0f;
    Expt.R_X = 0.0f;
    Expt.R_Y = 0.0f;
    Expt.D_W = 0.0f;

}

/*------------------------------------------------------------ RC遥控器控�? ------------------------------------------------------------*/
//拨杆模式更新
uint16_t Reset_cnt;
uint8_t init_pick;
void CTRL_DR16_classdef::LeverMode_Update(void)
{
    if(DR16.Get_S1_L() != DR16.Last_S1_L || DR16.Get_S2_R() != DR16.Last_S2_R)
    {
        LX_Zero=true;
        LY_Zero=true;
        RX_Zero=true;
        RY_Zero=true;
        DW_Zero=true;
    }

    switch((uint8_t)DR16.Get_S1_L())
    {
        case Lever_UP:  // --- 左上 -----------------------------------------------
        {
            switch((uint8_t)DR16.Get_S2_R())
            {
                case Lever_UP:/* 左上-右上 START ------------------------------------------*/ 
                {
									Gimbal.setMode(Gimbal_AutoMode);//Gimbal_HalfAutoMode;
                  Shoot.Shoot_Mode_Set(Shoot_NewAutoMode);
                  Shoot.Pull_Mode_Set(Pull_LockMode);//(Pull_DebugMode);//Pull_LockMode
                  Clamp.setMode(Clamp_AutoMode);

                  if(DR16.Get_DW_Norm() == 0)
                  {
                    DW_Zero = true;
                  }
                  if(DR16.Get_DW_Norm() >= 330 && DW_Zero)
                  {
                      if(Clamp.Init_Flag || Clamp.Pick_Flag ||\
                       Clamp.Place_Point_Flag || Clamp.Place_Flag){}
                       else
                       {
                          if(init_pick)
                          {
                              init_pick = 0;
                              Clamp.Pick_Flag = 1;
                          }
                          else
                          {
                               Clamp.Place_Flag = 1;
//                              Vision.aim = 1;
                          }
                       }
                       DW_Zero = false;
                  }
                  else if(DR16.Get_DW_Norm() <= -330 && DW_Zero)
                  {
                      init_pick = 1;
                      Clamp.Init_Flag = 1;
                      Clamp.Pick_Flag = 0;
                      Clamp.Place_Point_Flag = 0;
                      Clamp.Place_Flag = 0;
											Clamp.step = 0;
                      DW_Zero = false;
                  }
									Mode = RCCtrl_Update_DisableMode;
                }
                break;  /* �?
                �?-右上 END ------------------------------------------*/
                
                case Lever_MID:/* 左上-右中 START ------------------------------------------*/ 
                {
                    // 正常发射
                    Gimbal.setMode(Gimbal_NormalMode);
										Shoot.Shoot_Mode_Set(Shoot_NewAutoMode);
                    Shoot.Pull_Mode_Set(Pull_DebugMode);
										Clamp.setMode(Clamp_AutoMode);

									if(DR16.Get_DW_Norm() == 0)
                  {
                    DW_Zero = true;
                  }
                  if(DR16.Get_DW_Norm() >= 110 && DW_Zero)
                  {
                      if(Clamp.Init_Flag || Clamp.Pick_Flag ||\
                       Clamp.Place_Point_Flag || Clamp.Place_Flag){}
                       else
                       {
													Clamp.Place_Flag = 1;
                       }
                       DW_Zero = false;
                  }
                  else if(DR16.Get_DW_Norm() <= -110 && DW_Zero)
                  {
                      Clamp.Init_Flag = 1;
                      Clamp.Pick_Flag = 0;
                      Clamp.Place_Point_Flag = 0;
                      Clamp.Place_Flag = 0;
											Clamp.step = 0;
                      DW_Zero = false;
                  }
									
									Mode = RCCtrl_Update_DisableMode;
                }
                break;  /* 左上-右中 END ------------------------------------------*/

                case Lever_DOWN:/* 左上-右下 START ------------------------------------------*/ 
                {
                    //发射调整
                    Gimbal.setMode(Gimbal_LockMode);
                    Shoot.Shoot_Mode_Set(Shoot_LockMode);
                    Shoot.Pull_Mode_Set(Pull_LockMode);//(Pull_DebugMode);
                    Clamp.setMode(Clamp_DebugMode);
										Mode = RCCtrl_Update_ClampMode;
                }
                break;  /* 左上-右下 END ------------------------------------------*/
            }
        }
        break;  // 左上 END ---------------------------------------------------

        case Lever_MID:  // --- 左中 ----------------------------------------------
        {
            //底盘
            // Gimbal.Mode = Gimbal_LockMode;
            // Shoot.Shoot_Mode = Shoot_LockMode;
            // Shoot.Pull_Mode_Set(Pull_LockMode);
            // Clamp.setMode(Clamp_LockMode);
            switch((uint8_t)DR16.Get_S2_R())
            {
                case Lever_UP:/* 左中-右上 START ------------------------------------------*/ 
                {
                    // 正常发射
									Gimbal.setMode(Gimbal_AutoMode);//Gimbal_HalfAutoMode;
                  Shoot.Shoot_Mode_Set(Shoot_NewAutoMode);
                  Shoot.Pull_Mode_Set(Pull_LockMode);//(Pull_DebugMode);//Pull_LockMode
                  Clamp.setMode(Clamp_AutoMode);

										Mode = RCCtrl_Update_DisableMode;
                }
                break;/* 左中-右上 END ------------------------------------------*/

                case Lever_MID:/* 左中-右中 START ------------------------------------------*/ 
                {
//                    Gimbal.Mode = Gimbal_LockMode;
//                    Shoot.Shoot_Mode = Shoot_NewAutoMode;
//                    Shoot.Pull_Mode_Set(Pull_DebugMode);//Pull_LockMode
//                    Clamp.setMode(Clamp_AutoMode);


//                    if(DR16.Get_DW_Norm() >= 550)
//                    {
//                        Gimbal.Ding_TEXT_Flag = 1;
//                    }
//                    if(DR16.Get_DW_Norm() <= -550)
//                    {
//                        Gimbal.Ding_TEXT_Flag = 2;
//                    }
											Gimbal.setMode(Gimbal_LockMode);
											Shoot.Shoot_Mode_Set(Shoot_LockMode);
											Shoot.Pull_Mode_Set(Pull_LockMode);
											Clamp.setMode(Clamp_LockMode);
											Mode = RCCtrl_Update_DisableMode;
                    
                }
                break;/* 左中-右中 END ------------------------------------------*/

                case Lever_DOWN:/* 左中-右下 START ------------------------------------------*/ 
                {
//										Gimbal.Mode = Gimbal_PCMode;
//                    Shoot.Shoot_Mode = Shoot_LockMode;
//                    Shoot.Pull_Mode_Set(Pull_LockMode);//Pull_LockMode
//                    Clamp.setMode(Clamp_LockMode);
//										if(DR16.Get_DW_Norm() >= 550)
//										{
//											Vision.aim = 1;
//										}
										
                    // Gimbal.Mode = Gimbal_DisableMode;
                    // Shoot.Shoot_Mode = Shoot_DisableMode;
                    // Shoot.Pull_Mode_Set(Pull_DisableMode);
                    // Clamp.setMode(Clamp_DisableMode);

//										 if(DR16.Get_DW_Norm() >= 550)
//										 {
//												 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//										 }
//										 if(DR16.Get_DW_Norm() <= -550)
//										 {
//												 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//										 }
										
									Gimbal.setMode(Gimbal_NormalMode);
									Shoot.Shoot_Mode_Set(Shoot_DebugMode);
									Shoot.Pull_Mode_Set(Pull_NewDebugMode);//(Pull_DebugMode);
									Clamp.setMode(Clamp_LockMode);
									Mode = RCCtrl_Update_AllShootMode;
                }
                break;/* 左中-右下 END ------------------------------------------*/
            }
        }
        break;  // 左中 END ---------------------------------------------------

        case Lever_DOWN: // --- 左下 ----------------------------------------------
        {
            Gimbal.setMode(Gimbal_DisableMode);
            Shoot.Shoot_Mode_Set(Shoot_DisableMode);
            Shoot.Pull_Mode_Set(Pull_DisableMode);
            Clamp.setMode(Clamp_DisableMode);
						Mode = RCCtrl_Update_DisableMode;
            // Robot Reset
            if(DR16.Get_DW_Norm() == -660)
            {
                Reset_cnt++;
                if(Reset_cnt == 1500)//--- 拨轮打上3s重启
                {
                    //--- �?片�?�位
                    __set_FAULTMASK(1);    //关闭所有中�?
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

//RC控制模式 - 对�?��?�置�?标�?
void CTRL_DR16_classdef::RCCtrl_Update(void)
{
    switch((uint8_t)Mode)
    {
        case RCCtrl_Update_DisableMode:
            Expt.L_X = DR16.Get_LX_Norm() * 0;
            Expt.L_Y = DR16.Get_LY_Norm() * 0;
            Expt.D_W = DR16.Get_DW_Norm() * 0;
            Expt.R_X = DR16.Get_RX_Norm() * 0;
            Expt.R_Y = DR16.Get_RY_Norm() * 0;
        break;

        case RCCtrl_Update_GimbalMode:
            Expt.L_X = DR16.Get_LX_Norm() * 0;
            Expt.L_Y = DR16.Get_LY_Norm() * 0;
            Expt.D_W = DR16.Get_DW_Norm() * 0;
            Expt.R_X = abs(DR16.Get_RX_Norm()) * DR16.Get_RX_Norm() * sq(Yaw_Turn);
            Expt.R_Y = DR16.Get_RY_Norm() * 0;
        break;
				
        case RCCtrl_Update_AllShootMode:
            Expt.L_X = DR16.Get_LX_Norm() * 0;
            Expt.L_Y = DR16.Get_LY_Norm() * 15;
            Expt.D_W = DR16.Get_DW_Norm() * 1;
            Expt.R_X = abs(DR16.Get_RX_Norm()) * DR16.Get_RX_Norm() * sq(Yaw_Turn);
            Expt.R_Y = DR16.Get_RY_Norm() * 15;
        break;
				
				case RCCtrl_Update_ClampMode:
            Expt.L_X = DR16.Get_LX_Norm() * 0.1;
            Expt.L_Y = DR16.Get_LY_Norm() * 1;
            Expt.D_W = DR16.Get_DW_Norm() * 1;
            Expt.R_X = abs(DR16.Get_RX_Norm()) * DR16.Get_RX_Norm() * sq(Yaw_Turn);
            Expt.R_Y = DR16.Get_RY_Norm() * 1;
        break;
				
				case RCCtrl_Update_AutoMode:
            Expt.L_X = DR16.Get_LX_Norm() * 0;
            Expt.L_Y = DR16.Get_LY_Norm() * 0;
            Expt.D_W = DR16.Get_DW_Norm() * 0;
            Expt.R_X = abs(DR16.Get_RX_Norm()) * DR16.Get_RX_Norm() * sq(Yaw_Turn);
            Expt.R_Y = DR16.Get_RY_Norm() * 0;
        break;
    }
}

/*------------------------------------------------------------ 处理 ------------------------------------------------------------*/
//数据监测 ———�? - 0:Normal - 1:Abnormal
int8_t CTRL_DR16_classdef::Data_Monitor(void)
{
    if((DR16.Get_S1_L() != Lever_UP && DR16.Get_S1_L() != Lever_MID && DR16.Get_S1_L() != Lever_DOWN && DR16.Get_S1_L() != Lever_NONE) || /*<! 左拨�? */
       (DR16.Get_S2_R() != Lever_UP && DR16.Get_S2_R() != Lever_MID && DR16.Get_S2_R() != Lever_DOWN && DR16.Get_S2_R() != Lever_NONE) || /*<! 右拨�? */
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
    Expt.L_X = 0;
    Expt.L_Y = 0;
    Expt.R_X = 0;
    Expt.R_Y = 0;
    Expt.D_W = 0;
}

//获取对�?�输出数�? ———�? - export data
float CTRL_DR16_classdef::Get_LX()
{
    return Expt.L_X;
}
float CTRL_DR16_classdef::Get_LY()
{
    return Expt.L_Y;
}
float CTRL_DR16_classdef::Get_RX()
{
    return Expt.R_X;
}
float CTRL_DR16_classdef::Get_RY()
{
    return Expt.R_Y;
}
float CTRL_DR16_classdef::Get_DW()
{
    return Expt.D_W;
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
