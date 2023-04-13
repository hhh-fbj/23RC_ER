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

#include "INS_task.h"


/* 初始化 ------------------------------------------------------------*/
//初始化
//--- 调参模式
#define Gimbal_Debug 0
//--- 速度单位转不转为机械角度
#define turnangle 0
#define turnAngle 1

#define TURN_GROY 455.11111111111111111111111111111

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
    UsePID[Yaw][PID_Outer].SetPIDParam(15.0f, 0.0, 0.0f, 8000, 80000, 0.002f);//
	UsePID[Yaw][PID_Inner].SetPIDParam(4.0f, 1.2, 0.0f, 10000, 30000, 0.002f);//

	/*--- Angle Init -------------------------------------------------------------------------*/
	Set_InitAngle();

	Param.Yaw_Max = 583025;
	Param.Yaw_Centre = 540889;
	Param.Yaw_Min = 507715;
	Param.Yaw_Speed = 20;
}


/**
 * @brief      云台总控函数
 * @param[in]  None
 * @retval     None
 */
//540528
uint8_t zero_init = false;
void Gimbal_classdef::Control()
{	
	//传感器检测结果获取
	Last_I6 = I6;
	I6 = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);
	
	//--- 云台更新——陀螺仪数据
	IMU_Update(INS_angle, INS_gyro);

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
#if CIMU_Code == Old_CIMU_Code
        // //--- 角度
        UseIMU.Angle[i] = angle[i]*(180/PI) + 180; //--- 弧度转化为度(0 ~ 360°)
        //--- 角速度
        UseIMU.Gyro[i] = gyro[2-i]*(180/PI);
        //--- 解算后的角速度 Yaw
        UseIMU.OffsetGyro[i] = INS_OffsetGyro[2-i]*(180/PI);
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
#endif
    }
	UseIMU.Gyro[1] = gyro[0]*(180/PI);
    //检测陀螺仪是否数据异常
	if(Imu_Det(INS_angle[0]) && Imu_Det(INS_angle[1]) && Imu_Det(INS_angle[2]))
	{
		DevicesMonitor.Update(Frame_GIMBAL_CIMU);
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
		UsePID[Pit][PID_Outer].Reset();
		UsePID[Pit][PID_Inner].Reset();

		Yaw_Motor.Out = 0;
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
float once;
GPIO_PinState last_I6;
void Gimbal_classdef::TargetAngle_Update(void)
{
	switch((int)Mode)
	{
		case Gimbal_NormalMode:
			//目标角度更新
			UseTarget[Yaw] -= CTRL_DR16.Get_RX();
		break;
		
		case Gimbal_PCMode:
			if(Vision.Use_Flag)
			{
				UseTarget[Yaw] -= (Vision.Use_Yaw*TURN_GROY);
				Vision.Use_Flag = 0;
			}
			else if(Vision.aim == 0)
			{
				UseTarget[Yaw] -= CTRL_DR16.Get_RX();
			}
		break;
		
		case Gimbal_LockMode:
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
		break;

		case Gimbal_HalfAutoMode:
			//io口接收到可发射信号
			if(Vision.Use_Flag)
			{
				if(once == 0)
				{
					UseTarget[Yaw] -= (Vision.Use_Yaw*TURN_GROY);
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
			

			last_I6 = I6;
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
		
		
	UsePID[Yaw][PID_Inner].Target = UseIMU.Gyro[0] * TURN_GROY;
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











