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
	UsePID[Yaw][PID_Outer].SetPIDParam(0.65f, 0.0, 0.0f, 700, 80000, 0.002f);//
	UsePID[Yaw][PID_Inner].SetPIDParam(50.0f, 0.0, 1.4f, 12000, 30000, 0.002f);//

//	UsePID[Yaw][PID_Outer].SetPIDParam(15.0f, 0.0, 0.0f, 8000, 80000, 0.002f);//
//	UsePID[Yaw][PID_Inner].SetPIDParam(1.70f, 1.7, 0.008f, 12000, 30000, 0.002f);//
//	UsePID[Yaw][PID_Inner].I_SeparThresh = 2200;
	
	/*--- Angle Init -------------------------------------------------------------------------*/
	Set_InitAngle();
	
	Param.Yaw_Centre = 183685;
	Param.Yaw_Max = Param.Yaw_Centre+40960;
	Param.Yaw_Min = Param.Yaw_Centre-40960;
	Param.Yaw_Speed = 200;
	Param.Angle_Big = 22.109448343751673f;
	Param.Angle_small = 67.890551656248330f;//
	Param.Yaw_TurnAngle = 1;//455.11111111111111111111111111111;
	
	//左为正，右为负
//	Param.Yaw_TarError[0] = 0.0f;
//	Param.Yaw_TarError[1] = 31276+400;
//	Param.Yaw_TarError[2] = 6;
//	Param.Yaw_TarError[3] = -31636;
//	Param.Yaw_TarError[4] = 10096;
//	Param.Yaw_TarError[5] = -10578+225;                                                                                                 ;
//	Param.Yaw_TarError[6] = -358+270;
//	Param.Yaw_TarError[7] = 5596+300-400+405;
//	Param.Yaw_TarError[8] = -6100+200-200+405;
	Param.Yaw_TarError[0] = 0.0f;
	Param.Yaw_TarError[1] = 31656;
	Param.Yaw_TarError[2] = 6;
	Param.Yaw_TarError[3] = -31636;
	Param.Yaw_TarError[4] = 10096;
	Param.Yaw_TarError[5] = -10353;                                                                                                 ;
	Param.Yaw_TarError[6] = -88;
	Param.Yaw_TarError[7] = 5901;
	Param.Yaw_TarError[8] = -5695;

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
	
//	Last_D12 = D12;
//	D12 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	
	//--- 云台更新——陀螺仪数据
	IMU_Update(imu_Export.AIMU_onlyAngle, imu_Export.AIMU_gyro);
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
	
	//
	CS_SJ = Yaw_Encider.getTotolAngle() - Param.Yaw_Centre;
	
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
	if(DevicesMonitor.Get_State(Frame_GIMBAL_CIMU) == Off_line)
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
	else{UseIMU.Gyro[0] = CIMU.Pack.Yaw_Z;init_mode = false;}
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
				UseTarget[Yaw] = Param.Yaw_Centre+(int)(clamp_angle*Param.Yaw_TurnAngle);
			}
			Last_Mode = Gimbal_AutoMode;
		}
		break;
		
		case Gimbal_RevAutoMode:
		{
			if(Clamp.Tar_Ring==Tar_Mid){YawRevise[Clamp.Tar_Ring]=0;}
			UseTarget[Yaw] -= CTRL_DR16.Get_RX();
			Param.Yaw_TarError[Clamp.Tar_Ring] = Yaw_Encider.getTotolAngle() - Param.Yaw_Centre;
			Last_Mode = Gimbal_RevAutoMode;
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
		
		
	UsePID[Yaw][PID_Inner].Target = UseIMU.Gyro[0];// * 455.11111111111111111111111111111; //Yaw_Motor.getSpeed()*60;
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

int gimbal_time;
bool Gimbal_classdef::TarPos_Move(Tar_Select_e angle)
{
	switch(angle)
	{
		case Tar_Mid:
			clamp_angle = Param.Yaw_TarError[0];
		break;
		case Tar_LTen:
			clamp_angle = Param.Yaw_TarError[1]+YawRevise[1];//215218-183685;//31677-455;//67.11341748046875f;
		break;
		case Tar_MTen:
			clamp_angle = Param.Yaw_TarError[2]+YawRevise[2];
		break;
		case Tar_RTen:
			clamp_angle = Param.Yaw_TarError[3]+YawRevise[3];//-31324-455;//-66.982201171875f;
		break;
		case Tar_LThirty:
			clamp_angle = Param.Yaw_TarError[4]+YawRevise[4];//+100;//10393-455;//21.93983349609375f;
		break;
		case Tar_RThirty:
			clamp_angle = Param.Yaw_TarError[5]+YawRevise[5];//-100;//-9904-455;//21.91099658203125f;
		break;
		case Tar_MSeventy:
			clamp_angle = Param.Yaw_TarError[6]+YawRevise[6];
		break;
		case Tar_DLThirty:
			clamp_angle = Param.Yaw_TarError[7]+YawRevise[7];//6043-455;
		break;
		case Tar_DRThirty:
			clamp_angle = Param.Yaw_TarError[8]+YawRevise[8];//-5552-455;
		break;
		case Tar_MTwenty_Five:
			clamp_angle = 0.0f;
		break;
	}
	
	if(abs(Yaw_Encider.getTotolAngle()-(Param.Yaw_Centre+clamp_angle*Param.Yaw_TurnAngle)) <= 100)
	{
		gimbal_time++;
		if(gimbal_time>10)
		{
			gimbal_time=0;
			return true;
		}
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

void Gimbal_classdef::CIMU_Rev(uint8_t data[8])
{
	CIMU.data[0] = data[0];
	CIMU.data[1] = data[1];
	CIMU.data[2] = data[2];
	CIMU.data[3] = data[3];
	CIMU.data[4] = data[4];
	CIMU.data[5] = data[5];
	CIMU.data[6] = data[6];
	CIMU.data[7] = data[7];
	if(CIMU.Pack.error){DevicesMonitor.FrameCounter[Frame_GIMBAL_CIMU]=0;}
	else{DevicesMonitor.Update(Frame_GIMBAL_CIMU);}
}





