
#include "APP_Shoot.h"
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include  <stdint.h>
#include  <string.h>

#include "tim.h"
#include "stm32f4xx_hal.h"

#include "System_DataPool.h"


//-834383
//-1682762
//848380

/**
 * @brief  	   
 * @param[in]  None
 * @retval 	   None
 */
Shoot_classdef::Shoot_classdef()
{
	LeftPull_PID[PID_Outer].SetPIDParam(/* 0.66f */0.35f, 0.0, 0.0f, 8000, 10000, 0.002f);//
	LeftPull_PID[PID_Inner].SetPIDParam(/* 0.66f */8.0f, 0.6, 0.0f, 8000, 15000, 0.002f);//

	RightPull_PID[PID_Outer].SetPIDParam(/* 0.66f */0.35f, 0.0, 0.0f, 8000, 10000, 0.002f);//
	RightPull_PID[PID_Inner].SetPIDParam(/* 0.66f */8.0f, 0.6, 0.0f, 8000, 15000, 0.002f);//
			
	Shoot_PID[PID_Outer].SetPIDParam(0.4f, 0.0, 0.0f, 8000, 15000, 0.002f);//
	Shoot_PID[PID_Inner].SetPIDParam(13.0f, 5.0f, 0.015f, 8000, 15000, 0.002f);//
	Shoot_PID[PID_Inner].I_SeparThresh = 3000;

	// Pre_PID[0].SetPIDParam(/* 0.66f */500.0f, 0.0, 0.0f, 5000, 18000, 0.002f);
	// Pre_PID[1].SetPIDParam(/* 0.66f */500.0f, 0.0, 0.0f, 5000, 18000, 0.002f);

	Shoot_TarAngle = Shoot_Motor.get_totalencoder();
	LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
	RightPull_TarAngle = RightPull_Motor.get_totalencoder();

	// Pre_Tar[0] = -80;
	// Pre_Tar[1] = -100;

	Param.Shoot_Hold = 263138;//560000;//520000
	Param.Shoot_Speed = 1320;//1320;//1320;
	Param.Shoot_Circle = 680000;//848380;683194
	Param.Shoot_ErrorPos = 1000;
	Param.Pull_Max = 13100000;

	Param.Shoot_StopTime = 5;
	Param.Shoot_LowSpeed = -1500;
	Param.Shoot_HightSpeed = -8000;
	Param.Pull_InitSpeed = -8000;
	Param.Pull_LeftFirst = 741510;
	Param.Pull_RightFirst = 789561;
	Param.Pull_LeftSecond = 11082351;
	Param.Pull_RightSecond = 11165807;
	Param.Pull_LeftThird = 12685275;
	Param.Pull_RightThird = 12770837;
	
	//判断参数
	Param.Servo_CtrlTime = 10;
	Param.Servo_TorqueCtrl = 64;
	Param.Servo_PosCtrl = 116;
	Param.Servo_InitPos = 3376;
	Param.Servo_OverPos = 1335;
	Param.Servo_ErrorPos = 25;
	
	Param.Servo_Mid_InitPos = 1080;
	Param.Servo_Mid_OverPos = 3045;

	//0
	Param.Pull_TarError[0][0]=0;//3559067+120000+130000;
	Param.Pull_TarError[0][1]=0;//3565900+120000+120000;
	//1
	Param.Pull_TarError[1][0]=3300000;//20000;
	Param.Pull_TarError[1][1]=3300000;//20000;
	//2
	Param.Pull_TarError[2][0]=3120000;//1720136;
	Param.Pull_TarError[2][1]=3120000;//1720741;
	//3
	Param.Pull_TarError[3][0]=3230000;
	Param.Pull_TarError[3][1]=3230000;
	//4
	Param.Pull_TarError[4][0]=3560000;//242561;
	Param.Pull_TarError[4][1]=3560000;//244171;
	//5
	Param.Pull_TarError[5][0]=3480000;//209019;
	Param.Pull_TarError[5][1]=3480000;//208879;
	//6
	Param.Pull_TarError[6][0]=3410000;//2581724;
	Param.Pull_TarError[6][1]=3410000;//2584193;
	//7
	Param.Pull_TarError[7][0]=4040000;//3145004;
	Param.Pull_TarError[7][1]=4040000;//3140182;
	//8
	Param.Pull_TarError[8][0]=3910000;//3107017;
	Param.Pull_TarError[8][1]=3910000;//3103687;		
	Shoot_Servo_Mid.Tyg=1;
}


/**
 * @brief  	   发射控制
 * @param[in]  None
 * @retval 	   None
 */

bool One_Mid;
void Shoot_classdef::Control()
{
	//�?动开关与光电传感�?
	//发射的光电传感器
	A2 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
	A3 = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10);
	I5 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);

	//舵机读值——与舵机控制交错进行
	servo_time++;
	if(servo_time >= Param.Servo_CtrlTime)
	{
		if(One_Mid){Shoot_Servo.ReadPosition();One_Mid=false;}
		else{Shoot_Servo_Mid.ReadPosition();One_Mid=true;}
		servo_time = 0;
	}
	
	//�?题�?��?
	if(ProblemDetection()){return;}

    //改拉�?+发射 �?
    PullTar_Update();
		ShootSpe_Update();
		Left_CS_SJ = LeftPull_Motor.get_totalencoder()-Top_LeftPull;
		Right_CS_SJ = RightPull_Motor.get_totalencoder()-Top_RightPull;
		
		Servo_Control();
	
		AngleLimit();

    //PID计算
    PullMotor_PIDCalc();
    ShootMotor_PIDCalc();
}


int shoot_ready_time;
void Shoot_classdef::Shoot_Sensor(GPIO_PinState io_pin)
{
	if(first == 0)//初始化的打一次情况—先到了再考虑其他情况
	{
		if(io_pin == GPIO_PIN_SET)
		{
			stop_time++;
		}
		else
		{
			stop_time = 0;
		}
		
		if(stop_time>=Param.Shoot_StopTime)
		{
			first = 1;
			stop_time = 0;
			AddAngle = 0;
			stop_shoot = Shoot_TarAngle = Shoot_Motor.get_totalencoder();
			Shoot_TarAngle += 106292;
		}
		else
		{
			AddAngle = 660;
		}
	}
}



bool One_Mid_Ctrl;
void Shoot_classdef::Servo_Control(void)
{

		if(servo_time == Param.Servo_CtrlTime/2)
		{
			if(One_Mid_Ctrl)
			{
				if(Ctrl_Flag == Param.Servo_TorqueCtrl)
				{
					Shoot_Servo.Torque(true);
				}
				else if(Ctrl_Flag == Param.Servo_PosCtrl &&\
				 Shoot_Servo.Torque_Flag == 1)
				{
					Shoot_Servo.Position_Control(ServoPos_Target);
				}
				One_Mid_Ctrl = false;
			}
			else
			{
				if(Ctrl_Mid_Flag == Param.Servo_TorqueCtrl)
				{
					Shoot_Servo_Mid.Torque(true);
				}
				else if(Ctrl_Mid_Flag == Param.Servo_PosCtrl &&\
				 Shoot_Servo_Mid.Torque_Flag == 1)
				{
					Shoot_Servo_Mid.Position_Control(ServoPos_Mid_Target);
				}
				One_Mid_Ctrl = true;
			}
		}	

}
void Shoot_classdef::PullTar_Update(void)
{
	Pull_Lock_Flag = 1;
	switch((int)Pull_Mode)
	{
		case Pull_TransiMode://转换区
			LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
			RightPull_TarAngle = RightPull_Motor.get_totalencoder();
			
			Pull_Mode = Pull_Next_Mode;
		break;

		case Pull_NewDebugMode://新测试模式，同步，会初始化需要传感器，一代的自动
		{
//			if(Top_LeftPull_Flag){;}
//			else{LeftPull_TarAngle += Param.Pull_InitSpeed;}
//			if(Top_RightPull_Flag){;}
//			else{RightPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_LeftPull_Flag && Top_RightPull_Flag)
			{
				if(CTRL_DR16.Get_LY() != 0)
				{
						LeftPull_TarAngle += (-CTRL_DR16.Get_LY());
						RightPull_TarAngle += (-CTRL_DR16.Get_LY());
				}
				else
				{
						LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
						RightPull_TarAngle = RightPull_Motor.get_totalencoder();
				}
			}
		}
		break;

		case Pull_GearSetMode://旧旧模式
		{
			if(Top_LeftPull_Flag){;}
			else{LeftPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_RightPull_Flag){;}
			else{RightPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_LeftPull_Flag && Top_RightPull_Flag)
			{
				if(Turn_Pull_Flag == 1)
				{
					Turn_Pull_Flag = 6;
					if(LeftPull_TarAngle == Param.Pull_LeftFirst + Top_LeftPull && RightPull_TarAngle == Param.Pull_RightFirst + Top_RightPull)
					{
						LeftPull_TarAngle = Top_LeftPull;
						RightPull_TarAngle = Top_RightPull;
					}
					else if(LeftPull_TarAngle == Param.Pull_LeftSecond + Top_LeftPull && RightPull_TarAngle == Param.Pull_RightSecond + Top_RightPull)
					{
						LeftPull_TarAngle = Param.Pull_LeftFirst + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightFirst + Top_RightPull;
					}
					else if(LeftPull_TarAngle == Param.Pull_LeftThird + Top_LeftPull && RightPull_TarAngle == Param.Pull_RightThird + Top_RightPull)
					{
						LeftPull_TarAngle = Param.Pull_LeftSecond + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightSecond + Top_RightPull;
					}
					else
					{
						LeftPull_TarAngle = Param.Pull_LeftFirst + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightFirst + Top_RightPull;
					}
				}
				else if(Turn_Pull_Flag == 2)
				{				
					Turn_Pull_Flag = 6;
					if(LeftPull_TarAngle == Param.Pull_LeftSecond + Top_LeftPull && RightPull_TarAngle == Param.Pull_RightSecond + Top_RightPull)
					{
						LeftPull_TarAngle = Param.Pull_LeftThird + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightThird + Top_RightPull;
					}
					else if(LeftPull_TarAngle == Param.Pull_LeftFirst + Top_LeftPull && RightPull_TarAngle == Param.Pull_RightFirst + Top_RightPull)
					{
						LeftPull_TarAngle = Param.Pull_LeftSecond + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightSecond + Top_RightPull;
					}
					else if(LeftPull_TarAngle == Top_LeftPull && RightPull_TarAngle == Top_RightPull)
					{
						LeftPull_TarAngle = Param.Pull_LeftFirst + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightFirst + Top_RightPull;
					}
					else
					{
						LeftPull_TarAngle = Param.Pull_LeftFirst + Top_LeftPull;
						RightPull_TarAngle = Param.Pull_RightFirst + Top_RightPull;
					}
				}
			}
		}
		break;
			
		case Pull_DebugMode://旧测试模式，无初始化，手动控制
		{
			if(CTRL_DR16.Get_LY() != 0)
			{
					LeftPull_TarAngle += (-CTRL_DR16.Get_LY());
			}
			else
			{
					LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
			}
			
			if(CTRL_DR16.Get_RY() != 0)
			{
					RightPull_TarAngle += (-CTRL_DR16.Get_RY());
			}
			else
			{
					RightPull_TarAngle = RightPull_Motor.get_totalencoder();
			}
		}
		break;
		
		case Pull_AutoMode:
		{
			if(Top_LeftPull_Flag){;}
			else{LeftPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_RightPull_Flag){;}
			else{RightPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_LeftPull_Flag && Top_RightPull_Flag)
			{
				
				if(clamp_pos_L == 0 && clamp_pos_R == 0)
				{
					LeftPull_TarAngle += Param.Pull_InitSpeed;
					RightPull_TarAngle += Param.Pull_InitSpeed;
				}
				else
				{
					if(LeftPull_TarAngle > Top_LeftPull + clamp_pos_L + (-Param.Pull_InitSpeed))
					{
						LeftPull_TarAngle -= (-Param.Pull_InitSpeed);
					}
					else if(LeftPull_TarAngle < Top_LeftPull + clamp_pos_L - (-Param.Pull_InitSpeed))
					{
						LeftPull_TarAngle += (-Param.Pull_InitSpeed);
					}
					else
					{
						LeftPull_TarAngle = Top_LeftPull + clamp_pos_L;
					}

					if(RightPull_TarAngle > Top_RightPull + clamp_pos_R + (-Param.Pull_InitSpeed))
					{
						RightPull_TarAngle -= (-Param.Pull_InitSpeed);
					}
					else if(RightPull_TarAngle < Top_RightPull + clamp_pos_R - (-Param.Pull_InitSpeed))
					{
						RightPull_TarAngle += (-Param.Pull_InitSpeed);
					}
					else
					{
						RightPull_TarAngle = Top_RightPull + clamp_pos_R;
					}
				}
			}
		}
		break;
		
		case Pull_RevAutoMode:
		{
			if(Top_LeftPull_Flag){;}
			else{LeftPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_RightPull_Flag){;}
			else{RightPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_LeftPull_Flag && Top_RightPull_Flag)
			{
				if(Clamp.Tar_Ring==Tar_Mid)
				{
					PullRevise[Clamp.Tar_Ring][0]=0;
					PullRevise[Clamp.Tar_Ring][1]=0;
				}
				
				if(clamp_pos_L+PullRevise[Clamp.Tar_Ring][0] == 0 &&\
					clamp_pos_R+PullRevise[Clamp.Tar_Ring][1] == 0)
				{
					LeftPull_TarAngle += Param.Pull_InitSpeed;
					RightPull_TarAngle += Param.Pull_InitSpeed;
				}
				else
				{
					if(LeftPull_TarAngle > Top_LeftPull + clamp_pos_L + (-Param.Pull_InitSpeed) + PullRevise[Clamp.Tar_Ring][0])
					{
						LeftPull_TarAngle -= (-Param.Pull_InitSpeed);
					}
					else if(LeftPull_TarAngle < Top_LeftPull + clamp_pos_L - (-Param.Pull_InitSpeed) + PullRevise[Clamp.Tar_Ring][0])
					{
						LeftPull_TarAngle += (-Param.Pull_InitSpeed);
					}
					else
					{
						LeftPull_TarAngle = Top_LeftPull + clamp_pos_L + PullRevise[Clamp.Tar_Ring][0];
					}

					if(RightPull_TarAngle > Top_RightPull + clamp_pos_R + (-Param.Pull_InitSpeed) + PullRevise[Clamp.Tar_Ring][1])
					{
						RightPull_TarAngle -= (-Param.Pull_InitSpeed);
					}
					else if(RightPull_TarAngle < Top_RightPull + clamp_pos_R - (-Param.Pull_InitSpeed) + PullRevise[Clamp.Tar_Ring][1])
					{
						RightPull_TarAngle += (-Param.Pull_InitSpeed);
					}
					else
					{
						RightPull_TarAngle = Top_RightPull + clamp_pos_R + PullRevise[Clamp.Tar_Ring][1];
					}
				}
			}
		}
		case Pull_LockMode:
			Pull_Lock_Flag = 0;
		break;
		
		case Pull_NewRevDebugMode:
		{
//			if(Top_LeftPull_Flag){;}
//			else{LeftPull_TarAngle += Param.Pull_InitSpeed;}
//			if(Top_RightPull_Flag){;}
//			else{RightPull_TarAngle += Param.Pull_InitSpeed;}
			if(Top_LeftPull_Flag && Top_RightPull_Flag)
			{
				if(CTRL_DR16.Get_LY() != 0)
				{
						LeftPull_TarAngle += (-CTRL_DR16.Get_LY());
						RightPull_TarAngle += (-CTRL_DR16.Get_LY());
				}
				else
				{
						LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
						RightPull_TarAngle = RightPull_Motor.get_totalencoder();
				}
			}
			Param.Pull_TarError[Clamp.Tar_Ring][0]=LeftPull_Motor.get_totalencoder()-Top_LeftPull;
			Param.Pull_TarError[Clamp.Tar_Ring][1]=RightPull_Motor.get_totalencoder()-Top_RightPull;
		}
		break;
	}
}


void Shoot_classdef::AngleLimit(void)
{
	// ������λ
	if(A3 == GPIO_PIN_RESET)
	{
		if(last_A3 != A3)
		{
			Top_LeftPull = LeftPull_Motor.get_totalencoder();
			Top_LeftPull_Flag = 1;
		}
		else if(Top_LeftPull_Flag == 1 && LeftPull_TarAngle <= Top_LeftPull)
		{ 
			LeftPull_TarAngle = Top_LeftPull;
			Pull_AddAngle[0]=0;
		}
	}
	else if(Top_LeftPull_Flag == 1)
	{
		if(LeftPull_TarAngle>=Top_LeftPull+Param.Pull_Max)
		{
			LeftPull_TarAngle = Top_LeftPull+Param.Pull_Max;
			Pull_AddAngle[0]=0;
		}
	}
	last_A3 = A3;

	// ������λ
	if(I5 == GPIO_PIN_RESET)
	{
		if(last_I5 != I5)
		{
			Top_RightPull = RightPull_Motor.get_totalencoder();
			Top_RightPull_Flag = 1;
		}
		else if(Top_RightPull_Flag == 1 && RightPull_TarAngle <= Top_RightPull)
		{ 
			RightPull_TarAngle = Top_RightPull;
			Pull_AddAngle[1]=0;
		}
	}
	else if(Top_RightPull_Flag == 1)
	{
		if(RightPull_TarAngle>=Top_RightPull+Param.Pull_Max)
		{
			RightPull_TarAngle = Top_RightPull+Param.Pull_Max;
			Pull_AddAngle[0]=0;
			
		}
	}
	last_I5 = I5;
}
		
bool g_mid,qie_huan;
void Shoot_classdef::ShootSpe_Update(void)
{
	switch((int)Shoot_Mode)
	{
		case Shoot_TransiMode:
			Shoot_TarAngle = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = 0;
			//特殊处理
			if(Shoot_Next_Mode==Shoot_NewAutoMode&&first)
			{
				Shoot_TarAngle=stop_shoot+108292;
			}
			Shoot_Mode = Shoot_Next_Mode;
			AddAngle = 0;
		break;
			
		case Shoot_NewAutoMode:
			Shoot_Sensor(A2);
			Shoot_TarAngle += AddAngle;
			Shoot_PID[PID_Outer].Target = Shoot_TarAngle;
			Shoot_PID[PID_Outer].Current = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = Shoot_PID[PID_Outer].Cal();
		break;

		case Shoot_ManualMode:
			Shoot_PID[PID_Inner].Target = (Manual_Flag == 0 ? 0:\
			(Manual_Flag == 1 ? Param.Shoot_LowSpeed : Param.Shoot_HightSpeed));
		break;

		case Shoot_NewManualMode:
			Shoot_TarAngle += (-CTRL_DR16.Get_DW());
			Shoot_PID[PID_Outer].Target = Shoot_TarAngle;
			Shoot_PID[PID_Outer].Current = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = Shoot_PID[PID_Outer].Cal();
		break;
		
		case Shoot_DebugMode:
//			Shoot_PID[PID_Inner].Target = CTRL_DR16.Get_DW();
			if(CTRL_DR16.Get_DW() > 110){sj=1;}
			if(sj)
			{
				if(Shoot.Set_Shoot(true)){sj = 0;}
			}
			
			Shoot_Sensor(A2);
			Shoot_TarAngle += AddAngle;
			Shoot_PID[PID_Outer].Target = Shoot_TarAngle;
			Shoot_PID[PID_Outer].Current = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = Shoot_PID[PID_Outer].Cal();
			if(CTRL_DR16.Get_RY()==0){g_mid=true;}
			if(CTRL_DR16.Get_RY() > 0 && g_mid)
			{
				if(qie_huan)
				{
					Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_OverPos);
					qie_huan = false;
				}
				else{
					Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_InitPos);
					qie_huan = true;
				}
				g_mid=false;
			}
			if(CTRL_DR16.Get_RY() < 0 && g_mid)
			{
				if(qie_huan)
				{
					Set_Mid_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_Mid_OverPos);
					qie_huan = false;
				}
				else{
					Set_Mid_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_Mid_InitPos);
					qie_huan = true;
				}
				g_mid=false;
			}
			
		break;
		
		case Shoot_LockMode:
			Shoot_TarAngle += 0;
			Shoot_PID[PID_Outer].Target = Shoot_TarAngle;
			Shoot_PID[PID_Outer].Current = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = Shoot_PID[PID_Outer].Cal();
		break;
	}
}


uint8_t Shoot_classdef::ProblemDetection(void)
{
	if(Shoot.Shoot_Mode == Shoot_DisableMode ||\
	Shoot.Pull_Mode == Pull_DisableMode ||\
	CTRL_DR16.start == 0 ||\
	DevicesMonitor.Get_State(DR16_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(LEFTPULL_MOTOR_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(RIGHTPULL_MOTOR_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(SHOOT_MOTOR_MONITOR) == Off_line)
//    if(CTRL_DR16.start == 0 ||\
//		DevicesMonitor.Get_State(SHOOT_MOTOR_MONITOR) == Off_line)
		{
		LeftPull_PID[PID_Outer].Reset();
		LeftPull_PID[PID_Inner].Reset();

		RightPull_PID[PID_Outer].Reset();
		RightPull_PID[PID_Inner].Reset();
		
		Shoot_PID[PID_Outer].Reset();
		Shoot_PID[PID_Inner].Reset();

//		Pre_PID[0].Reset();
//		Pre_PID[1].Reset();
					
		Shoot_Motor.Out = 0;
		LeftPull_Motor.Out = 0;
		RightPull_Motor.Out = 0;

		Shoot_TarAngle = Shoot_Motor.get_totalencoder();
		LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
		RightPull_TarAngle = RightPull_Motor.get_totalencoder();

		Motor[0].Out = 0;
		Motor[1].Out = 0;
		Motor[2].Out = 0;
		Motor[3].Out = 0;
		
		
		Set_TurnPlacel(0,Shoot_Servo.Posotion);
		Shoot_Servo.Torque_Flag = 0;
		Set_Mid_TurnPlacel(0,Shoot_Servo_Mid.Posotion);
		Shoot_Servo_Mid.Torque_Flag = 0;
		if(servo_time == 5)
		{
			if(One_Mid_Ctrl)
			{
				Shoot_Servo.Torque(false);
				One_Mid_Ctrl = false;
			}
			else
			{
				Shoot_Servo_Mid.Torque(false);
				One_Mid_Ctrl = true;
			}
		}
//		first = 0;
		Shoot.Shoot_Mode = Shoot_DisableMode;
		Shoot.Pull_Mode = Pull_DisableMode;
		Shoot.Shoot_Next_Mode = Shoot_DisableMode;
		Shoot.Pull_Next_Mode = Pull_DisableMode;
		return 1;
	}
	else
	{
		return 0;
	}
}

void Shoot_classdef::PullMotor_PIDCalc(void)
{
	if(Pull_Mode == Pull_DisableMode)
	{
			LeftPull_PID[PID_Outer].Reset();
			LeftPull_PID[PID_Inner].Reset();

			RightPull_PID[PID_Outer].Reset();
			RightPull_PID[PID_Inner].Reset();

//			Pre_PID[0].Reset();
//			Pre_PID[1].Reset();

			LeftPull_Motor.Out = 0;
			RightPull_Motor.Out = 0;
			
			LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
			RightPull_TarAngle = RightPull_Motor.get_totalencoder();

			Motor[1].Out = 0;
			Motor[2].Out = 0;
			Motor[3].Out = 0;
			return;
	}

	LeftPull_PID[PID_Outer].Target = LeftPull_TarAngle;
	LeftPull_PID[PID_Outer].Current = LeftPull_Motor.get_totalencoder();
	LeftPull_PID[PID_Inner].Target = LeftPull_PID[PID_Outer].Cal()*Pull_Lock_Flag;
	LeftPull_PID[PID_Inner].Current = LeftPull_Motor.getSpeed();
	LeftPull_Motor.Out = LeftPull_PID[PID_Inner].Cal();

	RightPull_PID[PID_Outer].Target = RightPull_TarAngle;
	RightPull_PID[PID_Outer].Current = RightPull_Motor.get_totalencoder();
	RightPull_PID[PID_Inner].Target = RightPull_PID[PID_Outer].Cal()*Pull_Lock_Flag;
	RightPull_PID[PID_Inner].Current = RightPull_Motor.getSpeed();
	RightPull_Motor.Out = RightPull_PID[PID_Inner].Cal();

	Motor[1].Out = LeftPull_Motor.Out;
	Motor[2].Out = RightPull_Motor.Out;
}

void Shoot_classdef::ShootMotor_PIDCalc(void)
{
	if(Shoot_Mode == Shoot_DisableMode)
	{
		Shoot_TarAngle = Shoot_Motor.get_totalencoder();
		Shoot_PID[PID_Outer].Reset();
		Shoot_PID[PID_Inner].Reset();
					
		Shoot_Motor.Out = 0;

		Motor[0].Out = 0;
		Motor[3].Out = 0;
		return;
	}
	Shoot_PID[PID_Inner].Current = Shoot_Motor.getSpeed();
	Shoot_Motor.Out = Shoot_PID[PID_Inner].Cal();
	Motor[0].Out = Shoot_Motor.Out;
}

void Shoot_classdef::Shoot_Mode_Set(Shoot_CtrlMode_e mode)
{
	if(Shoot_Mode != mode)
	{
		Shoot_Mode = Shoot_TransiMode;
		Shoot_Next_Mode = mode;
	}
}
void Shoot_classdef::Pull_Mode_Set(Pull_CtrlMode_e mode)
{
	if(Pull_Mode != mode)
	{
		Pull_Mode = Pull_TransiMode;
		Pull_Next_Mode = mode;
	}
}

bool Shoot_classdef::Pull_Move(Tar_Select_e pos)
{
		switch(pos)
		{
			case Tar_Mid:
				clamp_pos_L = Param.Pull_TarError[0][0];//+70000;
				clamp_pos_R = Param.Pull_TarError[0][1];//+70000;
			break;
			case Tar_LTen:
				clamp_pos_L = Param.Pull_TarError[1][0]+PullRevise[1][0];//+50000;
				clamp_pos_R = Param.Pull_TarError[1][1]+PullRevise[1][1];//+50000;
			break;
			case Tar_MTen:
				clamp_pos_L = Param.Pull_TarError[2][0]+PullRevise[2][0];//+50000;//一1720136;//1680136;
				clamp_pos_R = Param.Pull_TarError[2][1]+PullRevise[2][1];//+50000;//一1720741;//1680741;
			break;
			case Tar_RTen:
				clamp_pos_L = Param.Pull_TarError[3][0]+PullRevise[3][0];;//+50000;
				clamp_pos_R = Param.Pull_TarError[3][1]+PullRevise[3][1];//+50000;
			break;
			case Tar_LThirty:
				clamp_pos_L = Param.Pull_TarError[4][0]+PullRevise[4][0];;//+50000;
				clamp_pos_R = Param.Pull_TarError[4][1]+PullRevise[4][1];//+50000;
			break;
			case Tar_RThirty:
				clamp_pos_L = Param.Pull_TarError[5][0]+PullRevise[5][0];;//+50000;
				clamp_pos_R = Param.Pull_TarError[5][1]+PullRevise[5][1];//+50000;
			break;
			case Tar_MSeventy:
				clamp_pos_L = Param.Pull_TarError[6][0]+PullRevise[6][0];;//+50000;//一2581724;
				clamp_pos_R = Param.Pull_TarError[6][1]+PullRevise[6][1];//+50000;//一2584193;
			break;
			case Tar_DLThirty:
				clamp_pos_L = Param.Pull_TarError[7][0]+PullRevise[7][0];;//+50000;//3105004;//3145322;
				clamp_pos_R = Param.Pull_TarError[7][1]+PullRevise[7][1];//+50000;//3100182;//3128846;
			break;
			case Tar_DRThirty:
				clamp_pos_L = Param.Pull_TarError[8][0]+PullRevise[8][0];;//+130000;//3067017;
				clamp_pos_R = Param.Pull_TarError[8][1]+PullRevise[8][1];//+130000;//3073687;
			break;
			
			case Tar_MTwenty_Five:
				clamp_pos_L = Param.Pull_TarError[8][0];//+130000;//3067017;
				clamp_pos_R = Param.Pull_TarError[8][1];//+130000;//3073687;
			break;
		}
		if(abs(LeftPull_Motor.get_totalencoder()-(Top_LeftPull+clamp_pos_L)) <= 800 && \
			abs(RightPull_Motor.get_totalencoder()-(Top_RightPull+clamp_pos_R)) <= 800)
		{
			return true;
		}
		return false;
}

//当发射时不可与其他一起并用使发射完成后需要等待，会影响stop_time，
//以上bug暂未处理
bool shoot_qianjin,shoot_tin;
int tin_time;
bool Shoot_classdef::Set_Shoot(bool shoot)
{
	if(first)
	{
		if(shoot)//发射
		{
			AddAngle = Param.Shoot_Speed+1320;
			
			//消抖
			//一圈+500容差
			if(A2 == GPIO_PIN_SET  &&\
			(Shoot_Motor.get_totalencoder() > stop_shoot+Param.Shoot_Circle/2))
			{
				stop_time++;
			}
			else if(shoot_tin==0 && shoot_qianjin==0)
			{
				stop_time = 0;
				return false;
			}
			
			if(stop_time>Param.Shoot_StopTime && shoot_tin==0)
			{
				stop_shoot = Shoot_TarAngle = Shoot_Motor.get_totalencoder();
				Shoot_TarAngle = stop_shoot+108292;
				shoot_tin=1;
				AddAngle = 0;
				
						tin_time=0;
						shoot_tin=0;
						stop_time = 0;
						shoot_qianjin=0;
						return true;
//				return false;
			}
			
//			if(shoot_tin)
//			{
//				AddAngle = 0;
//				tin_time++;
//				if(tin_time>=80)
//				{
//					shoot_qianjin=1;
//				}
//			}
			
//			if(shoot_qianjin)
//			{
//				if(Shoot_TarAngle >= stop_shoot+106292+Param.Shoot_Speed)
//				{
//					AddAngle = -Param.Shoot_Speed;
//					return false;
//				}
//				else if(Shoot_TarAngle <= stop_shoot+106292-Param.Shoot_Speed)
//				{
//					AddAngle = Param.Shoot_Speed;
//					return false;
//				}
//				else
//				{
//					AddAngle = 0;
//					Shoot_TarAngle = stop_shoot+106292;

//					if(abs(Shoot_Motor.get_totalencoder() - (stop_shoot+106292)) < Param.Shoot_ErrorPos)
//					{
//						tin_time=0;
//						shoot_tin=0;
//						stop_time = 0;
//						shoot_qianjin=0;
//						return true;
//					}
//					return false;
//				}
//			}
			return false;
		}
		
//		if(shoot)//发射
//		{
//			AddAngle = Param.Shoot_Speed;
//			//消抖
//			//一圈+500容差
//			if((A2 == GPIO_PIN_SET  &&\
//			(Shoot_Motor.get_totalencoder() > stop_shoot+Param.Shoot_Circle/2))||\
//			Shoot_Motor.get_totalencoder()>(stop_shoot+Param.Shoot_Circle+800))
//			{
//				stop_time++;
//			}
//			else if(shoot_tin==0 && shoot_qianjin==0)
//			{
//				stop_time = 0;
//				return false;
//			}
//			
//			if(stop_time>Param.Shoot_StopTime && shoot_tin==0)
//			{
//				stop_shoot = Shoot_TarAngle = Shoot_Motor.get_totalencoder();
//				shoot_tin=1;
//				AddAngle = 0;
//				return false;
//			}
//			
//			if(shoot_tin)
//			{
//				AddAngle = 0;
//				tin_time++;
//				if(tin_time>=150)
//				{
//					shoot_qianjin=1;
//				}
//			}
//			
//			if(shoot_qianjin)
//			{
//				if(Shoot_TarAngle >= stop_shoot+108292+Param.Shoot_Speed)
//				{
//					AddAngle = -Param.Shoot_Speed;
//					return false;
//				}
//				else if(Shoot_TarAngle <= stop_shoot+108292-Param.Shoot_Speed)
//				{
//					AddAngle = Param.Shoot_Speed;
//					return false;
//				}
//				else
//				{
//					AddAngle = 0;
//					Shoot_TarAngle = stop_shoot+108292;

//					if(abs(Shoot_Motor.get_totalencoder() - (stop_shoot+108292)) < Param.Shoot_ErrorPos)
//					{
//						tin_time=0;
//						shoot_tin=0;
//						stop_time = 0;
//						shoot_qianjin=0;
//						return true;
//					}
//					return false;
//				}
//			}
//			return false;
//		}
		else//停到某个位置——二代可以不用但需要修改发射因为检测冲突
		{
			if(Shoot_TarAngle >= stop_shoot+Param.Shoot_Hold+Param.Shoot_Speed)
			{
				AddAngle = -Param.Shoot_Speed;
				return false;
			}
			else if(Shoot_TarAngle <= stop_shoot+Param.Shoot_Hold-Param.Shoot_Speed)
			{
				AddAngle = Param.Shoot_Speed;
				return false;
			}
			else
			{
				AddAngle = 0;
				Shoot_TarAngle = stop_shoot+Param.Shoot_Hold;

				if(abs(Shoot_Motor.get_totalencoder() - (stop_shoot+Param.Shoot_Hold)) < Param.Shoot_ErrorPos)
				{
					return true;
				}
				return false;
			}
		}
	}
	return false;
}
bool Shoot_classdef::Set_TurnPlacel(int mode,int position)//位置设置
{
	if(mode == Param.Servo_PosCtrl && Shoot_Servo.Torque_Flag)
	{
		Ctrl_Flag = mode;
		ServoPos_Target = position;
		if(abs((int)Shoot_Servo.Posotion - position) <= Param.Servo_ErrorPos)
		{
			return true;
		}
		return false;
	}
	else
	{
		Ctrl_Flag = Param.Servo_TorqueCtrl;
		ServoPos_Target = Param.Servo_InitPos;
		return false;
	}
}
bool Shoot_classdef::Set_Mid_TurnPlacel(int mode,int position)//位置设置
{
	if(mode == Param.Servo_PosCtrl && Shoot_Servo_Mid.Torque_Flag)
	{
		Ctrl_Mid_Flag = mode;
		ServoPos_Mid_Target = position;
		if(abs((int)Shoot_Servo_Mid.Posotion - position) <= Param.Servo_ErrorPos)
		{
			return true;
		}
		return false;
	}
	else
	{
		Ctrl_Mid_Flag = Param.Servo_TorqueCtrl;
		switch(Clamp.Tar_Ring)
		{
		case Tar_MTen:
		ServoPos_Mid_Target = Param.Servo_Mid_InitPos;
		break;
			
		case Tar_LTen:
		case Tar_RTen:
		case Tar_LThirty:
		case Tar_RThirty:
		ServoPos_Mid_Target = Param.Servo_Mid_OverPos;
			break;
			
		case Tar_MSeventy:
		case Tar_DLThirty:
		case Tar_DRThirty:
		case Tar_Mid:
		ServoPos_Mid_Target = Param.Servo_Mid_InitPos;
			break;
		}
		return false;
	}
}

bool Shoot_classdef::Set_ShootServo(Tar_Select_e sta)
{
	switch(sta)
	{
		case Tar_MTen:
			if(Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_OverPos)&&\
		Set_Mid_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_Mid_InitPos)){return true;}
			else{return false;}
		break;
			
		case Tar_LTen:
		case Tar_RTen:
		case Tar_LThirty:
		case Tar_RThirty:
			if(Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_InitPos)&&\
			Set_Mid_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_Mid_OverPos)){return true;}
		break;
			
		case Tar_MSeventy:
		case Tar_DLThirty:
		case Tar_DRThirty:
		case Tar_Mid:
			if(Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_InitPos)&&\
			Set_Mid_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_Mid_InitPos)){return true;}
			else{return false;}
		break;
	}
	if(Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_InitPos)&&\
			Set_Mid_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_Mid_OverPos)){return true;}
	else{return false;}
}


