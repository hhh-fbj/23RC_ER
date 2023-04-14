
#include "APP_Shoot.h"
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include  <stdint.h>
#include  <string.h>

#include "tim.h"
#include "stm32f4xx_hal.h"

#include "System_DataPool.h"



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

	Param.Shoot_Hold = 560000;
	Param.Shoot_Speed = 880;
}


/**
 * @brief  	   发射控制
 * @param[in]  None
 * @retval 	   None
 */

GPIO_PinState C6;
GPIO_PinState E13;
GPIO_PinState E11;
void Shoot_classdef::Control()
{
	//�?动开关与光电传感�?
	//发射的光电传感器
	C6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	E13 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	E11 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);

	//�?题�?��?
	if(ProblemDetection()){return;}
		
    //改拉�?+发射 �?
    PullTar_Update();
	ShootSpe_Update();

	AngleLimit();

    //PID计算
    PullMotor_PIDCalc();
    ShootMotor_PIDCalc();
}


uint8_t first;
float shoot_time;
float stop_time;
float stop_shoot;
void Shoot_classdef::Shoot_NewSensor(GPIO_PinState IO_PIN)
{
	if(first == 0)//�?一次情况—先到了再�?�其他情�?
	{

		if(IO_PIN == GPIO_PIN_SET)
		{
			stop_time++;
		}
		else
		{
			stop_time = 0;
		}
	
		if(stop_time>=15)
		{
			first = 1;
			stop_time = 0;
			AddAngle = 0;
			Shoot_Continue = 0;
			Launch_Switch = 0;
			stop_shoot = Shoot_TarAngle = Shoot_Motor.get_totalencoder();
		}
		else
		{
			AddAngle = -Param.Shoot_Speed;
		}
	}
	else
	{
		if(Shoot_Place_Flag == 1 && Clamp.E14 == GPIO_PIN_RESET)
		{
			AddAngle = -Param.Shoot_Speed;
			if(Shoot_TarAngle <= stop_shoot-Param.Shoot_Hold)
			{
				Shoot_TarAngle = stop_shoot-Param.Shoot_Hold;
				AddAngle = 0;
				Shoot_Place_Flag = 2;
			}
		}

		//多�?�情�?
		if(Launch_Switch)
		{
			shoot_time++;
			//消抖
			if(IO_PIN == GPIO_PIN_SET)
			{
				stop_time++;
			}
			else
			{
				stop_time = 0;
			}
			AddAngle = -Param.Shoot_Speed;
			
			if(stop_time>25)
			{
				stop_shoot = Shoot_TarAngle = Shoot_Motor.get_totalencoder();
				shoot_time = 0;
				stop_time = 0;
				AddAngle = 0;
				Launch_Switch = 0;
				Shoot_Place_Flag = 0;
				if(Clamp.Place_Flag == 1)
				{
					Clamp.Homeing_Flag = 1;
				}
			}
		}
	}
}

void Shoot_classdef::Shoot_Sensor(GPIO_PinState IO_PIN)
{
	if(IO_PIN == GPIO_PIN_SET && Launch_Switch)
	{
		Shoot_Flag = 1;
		Shoot_Continue = 1;
	}
	else if(IO_PIN == GPIO_PIN_RESET && Shoot_Continue)
	{
		Shoot_Flag = 1;
		Launch_Switch = 0;
	}
	else if(IO_PIN == GPIO_PIN_SET && Shoot_Continue)
	{
		Shoot_Flag = 0;
		Shoot_Continue = 0;
		Launch_Switch = 0;
	}
	else if(IO_PIN == GPIO_PIN_RESET && Launch_Switch)
	{
		Shoot_Flag = 1;
		Shoot_Continue = 1;
		Launch_Switch = 0;
	}
}

GPIO_PinState last_E13 = GPIO_PIN_SET;
float Top_LeftPull = 0;
float Top_LeftPull_Flag = 0;
GPIO_PinState last_E11 = GPIO_PIN_SET;
float Top_RightPull = 0;
float Top_RightPull_Flag = 0;
void Shoot_classdef::PullTar_Update(void)
{
	Pull_Lock_Flag = 1;
	switch(Pull_Mode)
	{
		case Pull_DisableMode:
		break;
			
		case Pull_TransiMode:
			LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();
			RightPull_TarAngle = RightPull_Motor.get_totalencoder();
			Pull_Mode = Pull_Next_Mode;
		break;

		case Pull_FixedMode:
			if(Top_LeftPull_Flag){Pull_AddAngle[0] = 0;}
			else{Pull_AddAngle[0] = -8000;}
			if(Top_RightPull_Flag){Pull_AddAngle[1] = 0;}
			else{Pull_AddAngle[1] = -8000;}
			if(Top_LeftPull_Flag && Top_RightPull_Flag)
			{
				if(CTRL_DR16.Get_LY() == 0)
				{LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();}\
				else{LeftPull_TarAngle += (-CTRL_DR16.Get_LY());}
				if(CTRL_DR16.Get_LY() == 0)
				{RightPull_TarAngle = RightPull_Motor.get_totalencoder();}\
				else{RightPull_TarAngle += (-CTRL_DR16.Get_LY());}
			}
		break;

		case Pull_NewDebugMode:
			if(CTRL_DR16.Get_LY() == 0)
			{LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();}\
			else{LeftPull_TarAngle += (-CTRL_DR16.Get_LY());}
			
			if(CTRL_DR16.Get_LX() == 0)
			{RightPull_TarAngle = RightPull_Motor.get_totalencoder();}\
			else{RightPull_TarAngle += (-CTRL_DR16.Get_LX());}				
		break;
		
		case Pull_DebugMode:
			if(CTRL_DR16.Get_LY() <= 0)
			{
				if(Top_LeftPull_Flag)
				{
					if(CTRL_DR16.Get_LY() == 0)
					{LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();}\
					else{LeftPull_TarAngle += (-CTRL_DR16.Get_LY());}
				}
				else
				{
					LeftPull_TarAngle += 0;
				}
			}
			else
			{
				if(CTRL_DR16.Get_LY() == 0)
				{LeftPull_TarAngle = LeftPull_Motor.get_totalencoder();}\
				else{LeftPull_TarAngle += (-CTRL_DR16.Get_LY());}
			}
			
			if(CTRL_DR16.Get_LY() <= 0)
			{
				if(Top_RightPull_Flag)
				{
					if(CTRL_DR16.Get_LY() == 0)
					{RightPull_TarAngle = RightPull_Motor.get_totalencoder();}\
					else{RightPull_TarAngle += (-CTRL_DR16.Get_LY());}
				}
				else
				{
					RightPull_TarAngle += 0;
				}
			}
			else
			{
				if(CTRL_DR16.Get_LY() == 0)
				{RightPull_TarAngle = RightPull_Motor.get_totalencoder();}\
				else{RightPull_TarAngle += (-CTRL_DR16.Get_LY());}
			}			
		break;
		
		case Pull_LockMode:
			Pull_Lock_Flag = 0;
		break;
	}
}


void Shoot_classdef::AngleLimit(void)
{
	//������λ
	if(E13 == GPIO_PIN_RESET)
	{
		if(last_E13 != E13)
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
		if(LeftPull_TarAngle>=Top_LeftPull+13100000)
		{
			LeftPull_TarAngle = Top_LeftPull+13100000;
			Pull_AddAngle[0]=0;
		}
	}
	last_E13 = E13;

	//������λ
	if(E11 == GPIO_PIN_RESET)
	{
		if(last_E11 != E11)
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
		if(RightPull_TarAngle>=Top_RightPull+13100000)
		{
			RightPull_TarAngle = Top_RightPull+13100000;
			Pull_AddAngle[0]=0;
			
		}
	}
	last_E11 = E11;
}
		
void Shoot_classdef::ShootSpe_Update(void)
{
	switch(Shoot_Mode)
	{
		case Shoot_DisableMode:
			first = 0;
		break;
		
		case Shoot_TransiMode:
			Shoot_TarAngle = Shoot_Motor.get_totalencoder();
			Pull_Mode = Pull_Next_Mode;
		break;
			
		case Shoot_AutoMode:
			Shoot_Sensor(C6);
			Shoot_PID[PID_Inner].Target = \
			-1500 * Shoot_Flag * Shoot_Speed_BL;
		break;
			
		case Shoot_NewAutoMode:
			Shoot_NewSensor(C6);

			Shoot_TarAngle += AddAngle;
			Shoot_PID[PID_Outer].Target = Shoot_TarAngle;
			Shoot_PID[PID_Outer].Current = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = Shoot_PID[PID_Outer].Cal();

		break;

		case Shoot_ManualMode:
			Shoot_PID[PID_Inner].Target = (Manual_Flag == 0 ? 0:\
			(Manual_Flag == 1 ? -1500:-8000));
		break;

		case Shoot_NewManualMode:
			Shoot_TarAngle += (-CTRL_DR16.Get_DW());
			Shoot_PID[PID_Outer].Target = Shoot_TarAngle;
			Shoot_PID[PID_Outer].Current = Shoot_Motor.get_totalencoder();
			Shoot_PID[PID_Inner].Target = Shoot_PID[PID_Outer].Cal();
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
    if(CTRL_DR16.start == 0 ||\
	DevicesMonitor.Get_State(LEFTPULL_MOTOR_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(RIGHTPULL_MOTOR_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(SHOOT_MOTOR_MONITOR) == Off_line)
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
		for(int i = 0;i < 4;i++)
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
		}
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
