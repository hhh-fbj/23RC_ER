/**
 ------------------------------------------------------------------------------
 * @file    APP_Gimbal.cpp
 * @author  Shake
 * @brief   云台控制
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "APP_Gimbal.h"
/* Includes ------------------------------------------------------------------*/

#include "APP_Vision.h"

#include "System_DataPool.h"
#include "Wolf_Infantry.h"


/* 初始化 ------------------------------------------------------------*/
//初始化
//--- 调参模式
#define Gimbal_Debug 0
//--- 速度单位转不转为机械角度
#define turnangle 0
#define turnAngle 1

extKalman_t Kalman_PitSpeed;
/**
 * @brief      初始化
 * @param[in]  None
 * @retval     None
 */   
Gimbal_classdef::Gimbal_classdef()
{
  /*-------------------------------------------------
		6020正面朝左的参数正负：角度负，速度环正
		6020正面朝右的参数正负：角度正，速度环负
	--------------------------------------------------*/
    //--- kp,ki,kd,ki_max,out_max,dt

	/* Yaw 电机角度 */
	UsePID[Yaw][PID_Outer].SetPIDParam(12.0f, 0.0, 0.0f, 8000, 80000, 0.002f);//
	UsePID[Yaw][PID_Inner].SetPIDParam(3.0f, 1.7, 0.01f, 10000, 30000, 0.002f);//
	UsePID[Yaw][PID_Inner].I_SeparThresh = 4000;
	
	/*--- Angle Init -------------------------------------------------------------------------*/
	Set_InitAngle();

	Param.Yaw_Max = 436860;
	Param.Yaw_Centre = 395569;
	Param.Yaw_Min = 354170;
	Param.Yaw_Speed = 200;
	Param.Angle_Big = 22.109448343751673f;
	Param.Angle_small = 67.890551656248330f;//
	Param.Yaw_TurnAngle = 455.11111111111111111111111111111;
}


/**
 * @brief      云台总控函数
 * @param[in]  None
 * @retval     None
 */
//540528
void Gimbal_classdef::Control()
{	
	//传感器检测结果获取
	Last_D12 = D12;
	D12 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	
	//--- 云台更新——陀螺仪数据
	IMU_Update(imu_Export.AIMU_onlyAngle, imu_Export.AIMU_gyro);
//	UseIMU.Gyro[1] = imu_Export.AIMU_gyro[2];
//	if(Imu_Det(imu_Export.AIMU_onlyAngle[0]) &&\
//	Imu_Det(imu_Export.AIMU_onlyAngle[1]) &&\
//	Imu_Det(imu_Export.AIMU_onlyAngle[2]))
//	{
////		DevicesMonitor.Update(Frame_GIMBAL_CIMU);
//	}

	if(init_mode)
	{
		//--- 等待IMU初始化完毕
		wait_imuInit();
		return;
	}
	
	//--- 离线检测
	//问题检测
	if(ProblemDetection()){return;}

	//更新角度
	TargetAngle_Update();

	//限制角度
	AngleLimit();
		
	//--- PID计算
	Motor_PIDCalc();
}

/** 100%
 * @brief      获取IMU角度(角度制)
 * @param[in]  type
 * @retval     Angle
 */
void Gimbal_classdef::IMU_Update(float *angle, float *gyro)
{
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        // //--- 角度
        UseIMU.Angle[i] = angle[i];//*(180/PI) + 180; //--- 弧度转化为度(0 ~ 360°)
        //--- 角速度
        UseIMU.Gyro[i] = gyro[2-i];//*(180/PI);
        //--- 解算后的角速度 Yaw
//        UseIMU.OffsetGyro[i] = INS_OffsetGyro[2-i]*(180/PI);
        //--- 过零处理,累计角度
        if(UseIMU.Angle[i] - UseIMU.Pre_Angle[i] < -180)
        {
            UseIMU.Cnt[i]++;
        }
        else if(UseIMU.Angle[i] - UseIMU.Pre_Angle[i] > 180)
        {
            UseIMU.Cnt[i]--;
        }

        UseIMU.TotalAngle[i] = UseIMU.Angle[i] + UseIMU.Cnt[i] * 360.0f;
        UseIMU.Pre_Angle[i] = UseIMU.Angle[i];
    }
    //检测陀螺仪是否数据异常
	if(Imu_Det(angle[0]) && Imu_Det(angle[1]) && Imu_Det(angle[2]))
	{
//		DevicesMonitor.Update(Frame_GIMBAL_CIMU);
	}
}

void Gimbal_classdef::wait_imuInit(void)
{
	if(UseIMU.Angle[Yaw] != 0 || UseIMU.Angle[Pit] != 0 || UseIMU.Angle[Rol] != 0)
	{
		init_cnt++;
		if(init_cnt > 100) //--- 等待IMU稳定 800+500
		{
			//设置目标初始角度
			Set_InitAngle();
			init_mode = false;
		}
	}
}

uint8_t Gimbal_classdef::ProblemDetection(void)
{
  if(Mode == Gimbal_DisableMode ||\
	DevicesMonitor.Get_State(YAW_MOTOR_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(YAW_ENCODER_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(Frame_GIMBAL_CIMU) == Off_line)
	{
		//set init angle...
		//设置目标初始角度
		Set_InitAngle();
				
		UsePID[Yaw][PID_Outer].Reset();
		UsePID[Yaw][PID_Inner].Reset();
//		UsePID[Pit][PID_Outer].Reset();
//		UsePID[Pit][PID_Inner].Reset();
		
		Yaw_Motor.Out = 0;
		Mode = Gimbal_DisableMode;
		Last_Mode = Gimbal_DisableMode;
		return 1;
	}
	else
	{
		return 0;
	}
}

/** 100%
 * @brief      更新目标角度
 * @param[in]  Yawparam Pitparam
 * @note       这里的数值会换成和电机相同的量纲
 * @retval     None
 */
float cs_yaw;
void Gimbal_classdef::TargetAngle_Update(void)
{
	switch((int)Mode)
	{
		case Gimbal_TransiMode:
		{
			Set_InitAngle();
			UsePID[Yaw][PID_Outer].Reset();
			UsePID[Yaw][PID_Inner].Reset();
			Yaw_Motor.Out = 0;
			Mode = Next_Mode;
			Last_Mode = Gimbal_TransiMode;
		}
		break;
		
		case Gimbal_NormalMode:
		{
			//目标角度更新
			UseTarget[Yaw] -= CTRL_DR16.Get_RX();
			Last_Mode = Gimbal_NormalMode;
		}
		break;
		
		case Gimbal_PCMode:
		{
			if(Vision.Use_Flag)
			{
				UseTarget[Yaw] -= (Vision.Use_Yaw*Param.Yaw_TurnAngle);
				Vision.Use_Flag = 0;
			}
			else if(Vision.aim == 0)
			{
				UseTarget[Yaw] -= CTRL_DR16.Get_RX();
			}
			Last_Mode = Gimbal_PCMode;
		}
		break;
		
		case Gimbal_LockMode:
		{
			UseTarget[Yaw] -= 0;
			if(Midpoint_Flag==1)
			{
				if(UseTarget[Yaw] > Param.Yaw_Centre + Param.Yaw_Speed)
				{
					UseTarget[Yaw] -= Param.Yaw_Speed;
				}
				else if(UseTarget[Yaw] < Param.Yaw_Centre - Param.Yaw_Speed)
				{
					UseTarget[Yaw] += Param.Yaw_Speed;
				}
				else
				{
					UseTarget[Yaw] = Param.Yaw_Centre;
					Midpoint_Flag = 2;
				}
			}
			else
			{
				UseTarget[Yaw] -= 0;
			}
			
			switch(Ding_TEXT_Flag)
			{
				//最左
				case 1:
				{
					if(UseTarget[Yaw] > Param.Yaw_Centre + Param.Angle_Big*Param.Yaw_TurnAngle + Param.Yaw_Speed)
					{
						UseTarget[Yaw] -= Param.Yaw_Speed;
					}
					else if(UseTarget[Yaw] < Param.Yaw_Centre + Param.Angle_Big*Param.Yaw_TurnAngle - Param.Yaw_Speed)
					{
						UseTarget[Yaw] += Param.Yaw_Speed;
					}
					else
					{
						UseTarget[Yaw] = Param.Yaw_Centre+Param.Angle_Big*Param.Yaw_TurnAngle;
					}
				}
				break;

				//左
				case 2:
				{
					if(UseTarget[Yaw] > Param.Yaw_Centre+Param.Angle_small*Param.Yaw_TurnAngle + Param.Yaw_Speed)
					{
						UseTarget[Yaw] -= Param.Yaw_Speed;
					}
					else if(UseTarget[Yaw] < Param.Yaw_Centre+Param.Angle_small*Param.Yaw_TurnAngle - Param.Yaw_Speed)
					{
						UseTarget[Yaw] += Param.Yaw_Speed;
					}
					else
					{
						UseTarget[Yaw] = Param.Yaw_Centre+Param.Angle_small*Param.Yaw_TurnAngle;
					}
				}
				break;

				//右
				case 3:
				{
					if(UseTarget[Yaw] > Param.Yaw_Centre-Param.Angle_Big*Param.Yaw_TurnAngle + Param.Yaw_Speed)
					{
						UseTarget[Yaw] -= Param.Yaw_Speed;
					}
					else if(UseTarget[Yaw] < Param.Yaw_Centre-Param.Angle_Big*Param.Yaw_TurnAngle - Param.Yaw_Speed)
					{
						UseTarget[Yaw] += Param.Yaw_Speed;
					}
					else
					{
						UseTarget[Yaw] = Param.Yaw_Centre-Param.Angle_Big*Param.Yaw_TurnAngle;
					}
				}
				break;

				//最右
				case 4:
				{
					if(UseTarget[Yaw] > Param.Yaw_Centre-Param.Angle_small*Param.Yaw_TurnAngle + Param.Yaw_Speed)
					{
						UseTarget[Yaw] -= Param.Yaw_Speed;
					}
					else if(UseTarget[Yaw] < Param.Yaw_Centre-Param.Angle_small*Param.Yaw_TurnAngle - Param.Yaw_Speed)
					{
						UseTarget[Yaw] += Param.Yaw_Speed;
					}
					else
					{
						UseTarget[Yaw] = Param.Yaw_Centre-Param.Angle_small*Param.Yaw_TurnAngle;
					}
				}
				break;

				//中间
				case 0:
				{
					if(UseTarget[Yaw] > Param.Yaw_Centre + Param.Yaw_Speed)
					{
						UseTarget[Yaw] -= Param.Yaw_Speed;
					}
					else if(UseTarget[Yaw] < Param.Yaw_Centre - Param.Yaw_Speed)
					{
						UseTarget[Yaw] += Param.Yaw_Speed;
					}
					else
					{
						UseTarget[Yaw] = Param.Yaw_Centre;
					}
				}
				break;
			}
			Last_Mode = Gimbal_LockMode;
		}
		break;

		case Gimbal_HalfAutoMode:
		{
			//io口接收到可发射信号
			if(Vision.Use_Flag)
			{
				if(once == 0)
				{
					UseTarget[Yaw] -= (Vision.Use_Yaw*Param.Yaw_TurnAngle);
					once = 1;
				}
				if(abs(Yaw_Encider.getTotolAngle() - UseTarget[Yaw]) < 800)
				{
					Clamp.Place_Flag = 1;
					Vision.Use_Flag = 0;
					once = 0;
				}
			}
			else if(Vision.aim == 0)
			{
				if(Midpoint_Flag==1)
				{
					if(UseTarget[Yaw] > Param.Yaw_Centre + Param.Yaw_Speed)
					{
						UseTarget[Yaw] -= Param.Yaw_Speed;
					}
					else if(UseTarget[Yaw] < Param.Yaw_Centre - Param.Yaw_Speed)
					{
						UseTarget[Yaw] += Param.Yaw_Speed;
					}
					else
					{
						UseTarget[Yaw] = Param.Yaw_Centre;
						Midpoint_Flag = 2;
					}
				}
				
				else
				{
					UseTarget[Yaw] -= CTRL_DR16.Get_RX();
				}
			}
			Last_D12 = D12;
			Last_Mode = Gimbal_HalfAutoMode;
		}
		break;

		case Gimbal_AutoMode:
		{
			if(UseTarget[Yaw] > Param.Yaw_Centre+clamp_angle*Param.Yaw_TurnAngle + Param.Yaw_Speed)
			{
				UseTarget[Yaw] -= Param.Yaw_Speed;
			}
			else if(UseTarget[Yaw] < Param.Yaw_Centre+clamp_angle*Param.Yaw_TurnAngle - Param.Yaw_Speed)
			{
				UseTarget[Yaw] += Param.Yaw_Speed;
			}
			else
			{
				UseTarget[Yaw] = Param.Yaw_Centre+clamp_angle*Param.Yaw_TurnAngle;
			}
			Last_Mode = Gimbal_AutoMode;
		}
		break;
	}
}

void Gimbal_classdef::AngleLimit(void)
{
	if(UseTarget[Yaw] >= Param.Yaw_Max)
	{
		UseTarget[Yaw] = Param.Yaw_Max;
	}
	else if(UseTarget[Yaw] <= Param.Yaw_Min)
	{
		UseTarget[Yaw] = Param.Yaw_Min;
	};
}

/** 100%
 * @brief      云台电机位置式PID计算
 * @param[in]  None
 * @note       这里的数值会换算成和电机相同的量纲
 * @retval     None
 */
void Gimbal_classdef::Motor_PIDCalc()
{
	/*--- Yaw PID Calc --------------------------------------------------------------------------*/
	UsePID[Yaw][PID_Outer].Target = *Get_TargetAngle(Yaw);
	UsePID[Yaw][PID_Outer].Current = Yaw_Encider.getTotolAngle();
		
		
	UsePID[Yaw][PID_Inner].Target = UseIMU.Gyro[0] * Param.Yaw_TurnAngle; //Yaw_Motor.getSpeed()*60;
	UsePID[Yaw][PID_Inner].Current = UsePID[Yaw][PID_Outer].Cal();
    
	Yaw_Motor.Out = UsePID[Yaw][PID_Inner].Cal();

	/*--- Pit PID Calc --------------------------------------------------------------------------*/
}

float Gimbal_classdef::Imu_Det(float eup)
{
  if(0 <= abs(eup) && abs(eup) <= 360)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/** 100%
 * @brief      设置初始目标角度
 * @param[in]  yaw_init
 * @param[in]  pit_init
 * @retval     None
 */
void Gimbal_classdef::Set_InitAngle()
{
    UseTarget[Yaw] = Yaw_Encider.getTotolAngle();
}

/** 100%
 * @brief      获取计算的目标角度
 * @param[in]  type
 * @retval     Target Angle
 */
float *Gimbal_classdef::Get_TargetAngle(Gimbal_type_e type)
{
    static float *return_angle[3];
    return_angle[type] = &UseTarget[type];
    return return_angle[type];
}

bool Gimbal_classdef::TarPos_Move(int angle)
{	
	switch(angle)
	{
		case 0:
			clamp_angle = 0;
		break;
		
		case 2:
			clamp_angle = 0.0f;
		break;
		case 3:
			clamp_angle = 67.11341748046875f;
		break;
		case 4:
			clamp_angle = 21.93983349609375f;
		break;
		case 5:
			clamp_angle = -21.91099658203125f;
		break;
		case 6:
			clamp_angle = -66.982201171875f;
		break;
		case 7:
			clamp_angle = 0.0f;
		break;
		
		case 8:
			clamp_angle = 17.0f;
		break;
		
		case 9:
			clamp_angle = -17.0f;
		break;
	}
	
	if(abs(Yaw_Encider.getTotolAngle()-(Param.Yaw_Centre+clamp_angle*Param.Yaw_TurnAngle)) <= 1000)
	{
		return true;
	}
	return false;
}


void Gimbal_classdef::setMode(Gimbal_CtrlMode_e mode)
{
	if(Mode != mode)
	{
		Mode = mode;
		Next_Mode = mode;
	}
}






