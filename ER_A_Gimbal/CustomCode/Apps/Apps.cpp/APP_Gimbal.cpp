/**
 ------------------------------------------------------------------------------
 * @file    APP_Gimbal.cpp
 * @author  Shake
 * @brief   ��̨����
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


/* ��ʼ�� ------------------------------------------------------------*/
//��ʼ��
//--- ����ģʽ
#define Gimbal_Debug 0
//--- �ٶȵ�λת��תΪ��е�Ƕ�
#define turnangle 0
#define turnAngle 1

extKalman_t Kalman_PitSpeed;
/**
 * @brief      ��ʼ��
 * @param[in]  None
 * @retval     None
 */   
Gimbal_classdef::Gimbal_classdef()
{
  /*-------------------------------------------------
		6020���泯��Ĳ����������Ƕȸ����ٶȻ���
		6020���泯�ҵĲ����������Ƕ������ٶȻ���
	--------------------------------------------------*/
    //--- kp,ki,kd,ki_max,out_max,dt

	/* Yaw ����Ƕ� */
	UsePID[Yaw][PID_Outer].SetPIDParam(15.0f, 0.0, 0.0f, 8000, 80000, 0.002f);//
	UsePID[Yaw][PID_Inner].SetPIDParam(1.6f, 1.6, 0.008f, 10000, 30000, 0.002f);//
	UsePID[Yaw][PID_Inner].I_SeparThresh = 4000;
	
	/*--- Angle Init -------------------------------------------------------------------------*/
	Set_InitAngle();
	
	Param.Yaw_Centre = 563678;
	Param.Yaw_Max = Param.Yaw_Centre+40960;
	Param.Yaw_Min = Param.Yaw_Centre-40960;
	Param.Yaw_Speed = 100;
	Param.Angle_Big = 22.109448343751673f;
	Param.Angle_small = 67.890551656248330f;//
	Param.Yaw_TurnAngle = 1;//455.11111111111111111111111111111;
}


/**
 * @brief      ��̨�ܿغ���
 * @param[in]  None
 * @retval     None
 */
//540528
void Gimbal_classdef::Control()
{	
	//�������������ȡ
	Last_D12 = D12;
	D12 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	
	//--- ��̨���¡�������������
	IMU_Update(imu_Export.AIMU_onlyAngle, imu_Export.AIMU_gyro);
	if(init_mode)
	{
		//--- �ȴ�IMU��ʼ�����
		wait_imuInit();
		return;
	}
	
	//--- ���߼��
	//������
	if(ProblemDetection()){return;}

	//���½Ƕ�
	TargetAngle_Update();

	//���ƽǶ�
	AngleLimit();
		
	//--- PID����
	Motor_PIDCalc();
}

/** 100%
 * @brief      ��ȡIMU�Ƕ�(�Ƕ���)
 * @param[in]  type
 * @retval     Angle
 */
void Gimbal_classdef::IMU_Update(float *angle, float *gyro)
{
	if(DevicesMonitor.Get_State(Frame_GIMBAL_CIMU) == Off_line)
	{
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        // //--- �Ƕ�
        UseIMU.Angle[i] = angle[i];//*(180/PI) + 180; //--- ����ת��Ϊ��(0 ~ 360��)
        //--- ���ٶ�
        UseIMU.Gyro[i] = gyro[2-i];//*(180/PI);
        //--- �����Ľ��ٶ� Yaw
//        UseIMU.OffsetGyro[i] = INS_OffsetGyro[2-i]*(180/PI);
        //--- ���㴦��,�ۼƽǶ�
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
    //����������Ƿ������쳣
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
		if(init_cnt > 100) //--- �ȴ�IMU�ȶ� 800+500
		{
			//����Ŀ���ʼ�Ƕ�
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
		//����Ŀ���ʼ�Ƕ�
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
 * @brief      ����Ŀ��Ƕ�
 * @param[in]  Yawparam Pitparam
 * @note       �������ֵ�ỻ�ɺ͵����ͬ������
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
			//Ŀ��Ƕȸ���
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
				//����
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

				//��
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

				//��
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

				//����
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

				//�м�
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
			//io�ڽ��յ��ɷ����ź�
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
 * @brief      ��̨���λ��ʽPID����
 * @param[in]  None
 * @note       �������ֵ�ỻ��ɺ͵����ͬ������
 * @retval     None
 */
void Gimbal_classdef::Motor_PIDCalc()
{
	/*--- Yaw PID Calc --------------------------------------------------------------------------*/
	UsePID[Yaw][PID_Outer].Target = *Get_TargetAngle(Yaw);
	UsePID[Yaw][PID_Outer].Current = Yaw_Encider.getTotolAngle();
		
		
	UsePID[Yaw][PID_Inner].Target = UseIMU.Gyro[0] * 455.11111111111111111111111111111; //Yaw_Motor.getSpeed()*60;
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
 * @brief      ���ó�ʼĿ��Ƕ�
 * @param[in]  yaw_init
 * @param[in]  pit_init
 * @retval     None
 */
void Gimbal_classdef::Set_InitAngle()
{
    UseTarget[Yaw] = Yaw_Encider.getTotolAngle();
}

/** 100%
 * @brief      ��ȡ�����Ŀ��Ƕ�
 * @param[in]  type
 * @retval     Target Angle
 */
float *Gimbal_classdef::Get_TargetAngle(Gimbal_type_e type)
{
    static float *return_angle[3];
    return_angle[type] = &UseTarget[type];
    return return_angle[type];
}

bool Gimbal_classdef::TarPos_Move(Tar_Select_e angle)
{
	switch(angle)
	{
		case Tar_MTen:
		case Tar_MSeventy:
			clamp_angle = 0.0f;
		break;
		
		case Tar_LTen:
			clamp_angle = 10002;//67.11341748046875f;
		break;
		
		case Tar_RTen:
			clamp_angle = -10382;//-66.982201171875f;
		break;
		
		case Tar_LThirty:
			clamp_angle = 31305;//21.93983349609375f;
		break;
		
		case Tar_RThirty:
			clamp_angle = -31323;//21.91099658203125f;
		break;
		
		case Tar_DLThirty:
			clamp_angle = 12.8739f*455.11111111111111111111111111111;;
		break;
		
		case Tar_DRThirty:
			clamp_angle = -12.8739f*455.11111111111111111111111111111;;
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





