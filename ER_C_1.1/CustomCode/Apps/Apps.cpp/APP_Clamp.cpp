
#include "APP_Clamp.h"
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include  <stdint.h>
#include  <string.h>

#include "tim.h"
#include "stm32f4xx_hal.h"

#include "APP_DR16.h"

#include "System_DataPool.h"

//小心在取环机构初始化时，取环夹子松了与发送盘发送接触

Clamp_classdef::Clamp_classdef()
{
	Stretch_PID[PID_Outer].SetPIDParam(1.5f, 0.0, 0.0f, 8000, 10000, 0.002f);//
	Stretch_PID[PID_Inner].SetPIDParam(12.0f, 0.0, 0.0f, 4000, 16000, 0.002f);//

	Lift_PID[PID_Outer].SetPIDParam(0.3f, 0.0, 0.0f, 8000, 10000, 0.002f);//
	Lift_PID[PID_Inner].SetPIDParam(12.0f, 6.0, 0.0f, 8000, 16000, 0.002f);//

	PickPlace_PID[PID_Outer].SetPIDParam(/* 0.66f */0.35f, 0.0, 0.0f, 8000, 10000, 0.002f);//
	PickPlace_PID[PID_Inner].SetPIDParam(/* 0.66f */8.0f, 0.6, 0.0f, 8000, 16000, 0.002f);//

	UseTarget[0] = Stretch_Encider.getTotolAngle();
	UseTarget[1] = Lift_Motor.get_totalencoder();
	UseTarget[2] = PickPlace_Motor.get_totalencoder();
	
	//测量参数
	Param.Stretch_Hold = 1360000;
	Param.Stretch_Speed = 400;
	Param.Lift_Max = 770000;
	Param.Lift_Speed = 660;
	Param.PickPlace_Max = 5480000;
	Param.PickPlace_Release = 250000;
	Param.PickPlace_Loop = 485000;
	Param.PickPlace_Speed = 4000;

	//推算参数
	Param.Stretch_Max = Param.Stretch_Hold - 8848;
	Param.Stretch_Take = Param.Stretch_Hold - 9048;
	Param.Stretch_Shoot = Param.Stretch_Hold - 109981;//109981
	Param.Stretch_Min = Param.Stretch_Hold - 128052;//138052
	Param.PickPlace_Ready = Param.PickPlace_Max - 10 * Param.PickPlace_Loop;

	//判断参数
	Param.Servo_CtrlTime = 10;
	Param.Servo_TorqueCtrl = 64;
	Param.Servo_PosCtrl = 116;
	Param.Servo_InitPos = 2100;
	Param.Servo_OverPos = 1550;//1579
	Param.Servo_ErrorPos = 20;
	Param.Stretch_ErrorPos = 400;
	Param.PickPlace_ErrorPos = 200;
	Param.Lift_ErrorPos = 500;
}


void Clamp_classdef::Control()
{
	//传感器检测结果获取
	I7 = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7);
	E14 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);

	//舵机读值——与舵机控制交错进行
	servo_time++;
	if(servo_time >= Param.Servo_CtrlTime)
	{
		TurnPlace_Servo.ReadPosition();
		servo_time = 0;
	}

	//问题检测
	if(ProblemDetection()){return;}

	//目标更新
	Tar_Update();

	//舵机控制
	Servo_Control();

	//限位
	AngleLimit();

	//PID计算
	Motor_PIDCalc();
}

uint8_t Clamp_classdef::ProblemDetection(void)
{
	if(Clamp_Mode == Clamp_DisableMode ||\
	DevicesMonitor.Get_State(DR16_MONITOR) == Off_line ||\
	DevicesMonitor.Get_State(STRETCH_MOTOR_MONITOR)  == Off_line ||\
	DevicesMonitor.Get_State(STRETCH_ENCODER_MONITOR)  == Off_line ||\
	DevicesMonitor.Get_State(LIFT_MOTOR_MONITOR)  == Off_line ||\
	DevicesMonitor.Get_State(PICKPLACE_MOTOR_MONITOR)  == Off_line ||\
	DevicesMonitor.Get_State(TURNPLACE_SERVO_MONITOR)  == Off_line)
	{
		//set init angle...
		//数值清零
		UseTarget[0] = Stretch_Encider.getTotolAngle();
		UseTarget[1] = Lift_Motor.get_totalencoder();
		UseTarget[2] = PickPlace_Motor.get_totalencoder();
				
		Stretch_PID[PID_Outer].Reset();
		Stretch_PID[PID_Inner].Reset();
		Lift_PID[PID_Outer].Reset();
		Lift_PID[PID_Inner].Reset();
		PickPlace_PID[PID_Outer].Reset();
		PickPlace_PID[PID_Inner].Reset();

		Stretch_Motor.Out = 0;
		Lift_Motor.Out = 0;
		PickPlace_Motor.Out = 0;

		Motor[0].Out = 0;
		Motor[1].Out = 0;
		Motor[2].Out = 0;
		Motor[3].Out = 0;

		Set_TurnPlacel(0,Param.Servo_InitPos);
		TurnPlace_Servo.Torque_Flag = 0;
		if(servo_time == 5)
		{
			TurnPlace_Servo.Torque(false);
		}

		Should_Init_Flag = 1;
		Should_Place_Flag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}


void Clamp_classdef::Servo_Control(void)
{
	if(servo_time == Param.Servo_CtrlTime/2)
	{
		if(Ctrl_Flag == Param.Servo_TorqueCtrl)
		{
			TurnPlace_Servo.Torque(true);
		}
		else if(Ctrl_Flag ==  Param.Servo_PosCtrl &&\
		 TurnPlace_Servo.Torque_Flag == 1)
		{
			TurnPlace_Servo.Position_Control(ServoPos_Target);
		}
	}	
}

void Clamp_classdef::Set_TurnPlacel(uint8_t mode,float position)
{
	Ctrl_Flag = mode;
	ServoPos_Target = position;
}


void Clamp_classdef::Tar_Update(void)
{
	switch((int)Clamp_Mode)
	{
		case Clamp_TransiMode:
			UseTarget[0] = Stretch_Encider.getTotolAngle();
			UseTarget[1] = Lift_Motor.get_totalencoder();
			UseTarget[2] = PickPlace_Motor.get_totalencoder();
			Clamp_Mode = Clamp_Next_Mode;
		break;
		
		case Clamp_DebugMode:
			
			//在伸出允许范围内才可以下调抬伸——下
			if(CTRL_DR16.Get_LY()<0)
			{
				if(UseTarget[0] <= Param.Stretch_Max &&\
				 UseTarget[0] >= Param.Stretch_Take &&\
				 Top_Lift_Flag == 1)
				{
					UseTarget[1] += (-CTRL_DR16.Get_LY());
					if(TurnPlace_Servo.Torque_Flag == 0)
					{
						Set_TurnPlacel(Param.Servo_TorqueCtrl, Param.Servo_InitPos);
					}
					else
					{
						Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos);
					}
				}
				else
				{
					UseTarget[1] += 0;
				}
			}
			else
			{
				UseTarget[1] += (-CTRL_DR16.Get_LY());
			}

			//内
			if(CTRL_DR16.Get_LX() > 0)
			{
				if(E14 == GPIO_PIN_RESET)
				{
					UseTarget[0] += (-CTRL_DR16.Get_LX());
				}
				else
				{
					UseTarget[0] += 0;
				}
			}
			else
			{
				UseTarget[0] += (-CTRL_DR16.Get_LX());
			}
			
			if(CTRL_DR16.Get_DW()>0)
			{
				if(Top_PickPlace_Flag == 1)
				{
					UseTarget[2] += (-CTRL_DR16.Get_DW());
				}
				else
				{
					UseTarget[0] += 0;
				}
			}
			else
			{
				UseTarget[2] += (-CTRL_DR16.Get_DW());
			}
		break;
		
		case Clamp_AutoMode:
			if(TurnPlace_Servo.Torque_Flag == 0)
			{
				Set_TurnPlacel(Param.Servo_TorqueCtrl, Param.Servo_InitPos);
			}
			if (Gimbal.I6 == GPIO_PIN_SET &&\
			 Gimbal.Last_I6 == GPIO_PIN_RESET)
			{
				if(Should_Init_Flag==1)
				{
					Should_Place_Flag = 0;
					Init_Flag = 1;
				}
				else if(Auto_Aim_Flag)
				{
					if(Should_Place_Flag && Vision.aim==0)
					{
						Vision.aim=1;
					}
				}
				else if(Should_Place_Flag)
				{
					Place_Flag = 1;
				}
			}
			Init();
			Pick();
			Place_Point();
			Place();
			if(abs((int)TurnPlace_Servo.Posotion - Param.Servo_InitPos) <= Param.Servo_ErrorPos && AddVar[2] > 0)
			{
				AddVar[1] = 0;
			}
			UseTarget[0] += AddVar[0];
			UseTarget[1] += AddVar[1];
			UseTarget[2] += AddVar[2];
			//E14——Lift_Motor
			//I7——PickPlace_Motor
		break;
		
		case Clamp_LockMode:
			UseTarget[0] += 0;
			UseTarget[1] += 0;
			UseTarget[2] += 0;
		break;
	}

//	Var_SetAndLimit();


}

void Clamp_classdef::AngleLimit(void)
{
	//定点限位
	if(E14 == GPIO_PIN_RESET)
	{
		if(last_E14 != E14)
		{
			Top_Lift = Lift_Motor.get_totalencoder();
			Top_Lift_Flag = 1;
		}
		else if(Top_Lift_Flag == 1 && UseTarget[1]<=Top_Lift)
		{ 
			UseTarget[1] = Top_Lift;
		}
	}
	else if(Top_Lift_Flag == 1)
	{
		if(UseTarget[1]>=(Param.Lift_Max+Top_Lift))
		{
			UseTarget[1] = Param.Lift_Max+Top_Lift;
		}
	}
	last_E14 = E14;
	
	//取放限位
	if(I7 == GPIO_PIN_RESET)
	{
		if(last_I7 !=  I7)
		{
			Top_PickPlace = PickPlace_Motor.get_totalencoder();
			Top_PickPlace_Flag = 1;
		}
		else if(Top_PickPlace_Flag == 1 && UseTarget[2]>=Top_PickPlace)
		{ 
			UseTarget[2] = Top_PickPlace;
		}
	}
	else if(Top_PickPlace_Flag == 1)
	{
		if(UseTarget[2]<=(Top_PickPlace-Param.PickPlace_Max))
		{
			UseTarget[2] = Top_PickPlace-Param.PickPlace_Max;
		}
	}

	last_I7 = I7;

	//1351152
	//1180019_1146801
	//伸出限位
	if(UseTarget[0] >= Param.Stretch_Max)
	{
		UseTarget[0] = Param.Stretch_Max;
	}
	if(UseTarget[0] <= Param.Stretch_Min)
	{
		UseTarget[0] = Param.Stretch_Min;
	}

	//限死伸出
	// if(E14 != GPIO_PIN_RESET)
	// {
	// 	UseTarget[0] = Stretch_Encider.getTotolAngle();
	// }
}


void Clamp_classdef::Motor_PIDCalc(void)
{
	Stretch_PID[PID_Outer].Target = Stretch_Encider.getTotolAngle();
	Stretch_PID[PID_Outer].Current = UseTarget[0];
	Stretch_PID[PID_Inner].Target = Stretch_PID[PID_Outer].Cal();
	Stretch_PID[PID_Inner].Current = Stretch_Motor.getSpeed();
		
	Lift_PID[PID_Outer].Target = UseTarget[1];
  	Lift_PID[PID_Outer].Current = Lift_Motor.get_totalencoder();
	Lift_PID[PID_Inner].Target = Lift_PID[PID_Outer].Cal();
//	Lift_PID[PID_Inner].Target = UseTarget[1];
	Lift_PID[PID_Inner].Current = Lift_Motor.getSpeed();
    
	PickPlace_PID[PID_Outer].Target = UseTarget[2];
	PickPlace_PID[PID_Outer].Current = PickPlace_Motor.get_totalencoder();
	PickPlace_PID[PID_Inner].Target = PickPlace_PID[PID_Outer].Cal();
	PickPlace_PID[PID_Inner].Current = PickPlace_Motor.getSpeed();
	
	Stretch_Motor.Out = Stretch_PID[PID_Inner].Cal();
	Lift_Motor.Out = Lift_PID[PID_Inner].Cal();
	PickPlace_Motor.Out = PickPlace_PID[PID_Inner].Cal();

	Motor[0].Out = Stretch_Motor.Out;
	Motor[1].Out = Lift_Motor.Out;
	Motor[2].Out = PickPlace_Motor.Out;
}

void Clamp_classdef::setMode(Clamp_CtrlMode_e mode)
{
	if(Clamp_Mode != mode)
	{
		Clamp_Mode = Clamp_TransiMode;
		Clamp_Next_Mode = mode;
	}
}

void Clamp_classdef::Init(void)
{
	if(Init_Flag)
	{
		//上
		if(E14 != GPIO_PIN_RESET &&\
		Stretch_Encider.getTotolAngle() >=Param.Stretch_Take)
		{
			AddVar[1] = -Param.Lift_Speed;
		}
		else
		{
			AddVar[1] = 0;
		}



		//出
		AddVar[0] = Param.Stretch_Speed;
		AddVar[2] = Param.PickPlace_Speed;

		if(E14 == GPIO_PIN_RESET)
		{
			if(Gimbal.Midpoint_Flag == 0)
			{
				Gimbal.Midpoint_Flag = 1;
			}

			Set_TurnPlacel(Param.Servo_PosCtrl,Param.Servo_InitPos);
			if(UseTarget[0] == Param.Stretch_Max &&\
			abs(Stretch_Encider.getTotolAngle() - Param.Stretch_Max) <= Param.Lift_ErrorPos &&\
			I7 == GPIO_PIN_RESET &&\
			abs((int)TurnPlace_Servo.Posotion - Param.Servo_InitPos) <= Param.Servo_ErrorPos &&\
			Gimbal.Midpoint_Flag == 2)
			{
				Gimbal.Midpoint_Flag = 0;
				Init_Flag = 0;

				if(Should_Init_Flag==1)
				{
					Pick_Flag = 1;
					Should_Place_Flag = 1;
				}
				
				if(Should_Init_Flag==2)
				{
					Should_Init_Flag = 1;
				}
				else
				{
					Should_Init_Flag = 0;
				}
			}
		}
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else if(Should_Init_Flag == 1)
	{
		Should_Place_Flag = 0;
	}
}


//三个点
//取环
//放环位置
//放环
//
void Clamp_classdef::Pick(void)
{
	if(Pick_Flag)
	{
		if(UseTarget[0] >=Param.Stretch_Take  &&\
		UseTarget[0] <= Param.Stretch_Max &&\
		Stretch_Encider.getTotolAngle()>Param.Stretch_Take &&\
		TakeOut_Flag == 0)
		{
			AddVar[0] = 0;
			AddVar[1] = Param.Lift_Speed;
			AddVar[2] = -Param.PickPlace_Speed;
		}

		if(UseTarget[1] >= Param.Lift_Max+Top_Lift && TakeOut_Flag==0)
		{
			UseTarget[1] = Param.Lift_Max+Top_Lift;
			AddVar[0] = 0;
			if(abs(Lift_Motor.get_totalencoder() - (Param.Lift_Max+Top_Lift)) < Param.Lift_ErrorPos)
			{
				AddVar[1] = -Param.Lift_Speed;
				TakeOut_Flag = 1;
			}
		}
		if(UseTarget[2] <= Top_PickPlace-Param.PickPlace_Release)
		{	
			UseTarget[2] = Top_PickPlace-Param.PickPlace_Release;
			AddVar[0] = 0;
			AddVar[2] = 0;
		}

		if(E14 == GPIO_PIN_RESET && TakeOut_Flag == 1)
		{
			UseTarget[1] = Lift_Motor.get_totalencoder();
			AddVar[0] = 0;
			AddVar[1] = 0;
			AddVar[2] = 0;
			Pick_Flag =0;
			TakeOut_Flag = 0;
			Place_Point_Flag = 1;
			Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		}
	}
	else
	{
		TakeOut_Flag = 0;
	}
}

void Clamp_classdef::Place_Point(void)
{
	if(Place_Point_Flag)
	{
		Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos);
		if(E14 == GPIO_PIN_RESET)
		{
			AddVar[0] = -Param.Stretch_Speed;
			AddVar[1] = 0;
			AddVar[2] = -Param.PickPlace_Speed;
		}

		if(UseTarget[2] <= Top_PickPlace-Param.PickPlace_Ready)
		{
			UseTarget[2] = Top_PickPlace-Param.PickPlace_Ready;
			AddVar[2] = 0;
		}
		
		if(UseTarget[0] <= Param.Stretch_Min &&\
		abs((int)TurnPlace_Servo.Posotion - Param.Servo_OverPos) <= Param.Servo_ErrorPos)
		{
			UseTarget[0] = Param.Stretch_Min;
			AddVar[0] = 0;
			AddVar[1] = 0;
			AddVar[2] = 0;
			if(abs(Stretch_Encider.getTotolAngle() - Param.Stretch_Min)< Param.Stretch_ErrorPos)
			{
				Place_Point_Flag = 0;
			}
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		}
	}
}

void Clamp_classdef::Place(void)
{
	if(Place_Flag)
	{
		Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos);
		if(wait_flag==0)
		{
			if(Homeing_Flag==0 && UseTarget[0] <= Param.Stretch_Min && wait_flag==0)
			{
				UseTarget[0] = Param.Stretch_Min;
				if(Shoot.Shoot_Place_Flag==2 &&\
				 abs(Stretch_Encider.getTotolAngle()- Param.Stretch_Min) < Param.Stretch_ErrorPos)
				{
					AddVar[0] = 0;
					AddVar[1] = 0;
					//放环+判断放环是否结束
					AddVar[2] = -Param.PickPlace_Speed;
					if(UseTarget[2] <= now_place-Param.PickPlace_Loop)
					{
						UseTarget[2] = now_place-Param.PickPlace_Loop;
						AddVar[1] = 0;
						AddVar[2] = 0;
						if(abs(PickPlace_Motor.get_totalencoder()-(now_place-Param.PickPlace_Loop)<Param.PickPlace_ErrorPos))
						{	
							// AddVar[0] = Param.Stretch_Speed;
							// wait_flag = 1;
							AddVar[0] = 0;
							Shoot.Set_Shoot(true);
							wait_flag = 2;
						}
					}
					else if(UseTarget[2] <= Top_PickPlace-Param.PickPlace_Max)
					{
						UseTarget[2] = Top_PickPlace-Param.PickPlace_Max;
						AddVar[1] = 0;
						AddVar[2] = 0;
						if(abs(PickPlace_Motor.get_totalencoder()-(Top_PickPlace-Param.PickPlace_Max)<Param.PickPlace_ErrorPos))
						{	
							// UseTarget[2] = Top_PickPlace-Param.PickPlace_Max;
							// AddVar[0] = Param.Stretch_Speed;
							// wait_flag = 1;
							
							AddVar[0] = 0;
							Shoot.Set_Shoot(true);
							wait_flag = 2;
						}
					}
				}
			}
			else if(UseTarget[0] <= Param.Stretch_Min && Homeing_Flag==1)
			{
				UseTarget[0] = Param.Stretch_Min;
				AddVar[0] = 0;
				AddVar[1] = 0;
				AddVar[2] = 0;
				wait_flag = 0;
				Homeing_Flag = 0;
				Place_Flag = 0;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
				return;
			}
		}
		// //后退再发射
		// else if(UseTarget[0] >= Param.Stretch_Shoot && wait_flag == 1)
		// {
		// 	UseTarget[0] = Param.Stretch_Shoot;
		// 	AddVar[0] = 0;
		// 	AddVar[1] = 0;
		// 	AddVar[2] = 0;
		// 	if(abs(Stretch_Encider.getTotolAngle()-Param.Stretch_Shoot<Param.Stretch_ErrorPos))
		// 	{
		// 		Shoot.Set_Shoot(true);
		// 		//因为后退所以与最末端有一定距离
		// 		//故不会在wait_flag、Homeing_Flag为 0 时诱发放环条件
		// 		//但若改掉后退则需要注意
		// 		wait_flag = 0;
		// 	}
		// }

		if(Homeing_Flag == 1)
		{
			AddVar[0] = -Param.Stretch_Speed;
			wait_flag = 0;
		}
		else
		{
			if(Shoot.Shoot_Place_Flag == 0)
			{
				Shoot.Shoot_Place_Flag = 1;
			}
		}
	}
	else
	{
		wait_flag = 0;
		Homeing_Flag = 0;
		Place_Flag = 0;
		now_place = UseTarget[2];
	}
}