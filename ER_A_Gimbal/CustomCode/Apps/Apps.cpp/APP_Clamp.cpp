
#include "APP_Clamp.h"
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include  <stdint.h>
#include  <string.h>

#include "tim.h"
#include "stm32f4xx_hal.h"

#include "APP_DR16.h"

#include "System_DataPool.h"
#include "DEV_Timer.h"

#define LIFT_INIT_SPEED 440

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
	Param.Stretch_Min = 1333506;//1343000;//1341900//1349007//
	Param.Stretch_Max = 1435506;//1445000;//1487910;//1440000;//1333506;//
	Param.Stretch_Speed = 400;
	Param.Stretch_Speed_Ramp = 10;
	Param.Lift_Max = 585000;//770000;
	Param.Lift_Hold = 338000;//770000;336,776
	Param.Lift_Max_Speed = 3400;
	Param.Lift_Min_Speed = 1200;
	Param.Lift_addSpeed_Ramp = 10;
	Param.Lift_subSpeed_Ramp = 18;
	Param.Lift_Sub_SPortion = 300000;
	Param.Lift_Sub_XPortion = 80000;
	Param.Lift_PickWaitTime = 0;
	Param.PickPlace_Max = 6300000;//5480000;
	Param.PickPlace_Release = 500000;//250000;
	Param.PickPlace_Loop = 470000;//485000;
	Param.PickPlace_Speed = 8000;
	Param.Shoot_WaitTime = 40;
	
	//推算参数
	Param.PickPlace_D = -8200;
	Param.PickPlace_A1 = 480000-4*Param.PickPlace_D;
//	Param.PickPlace_Ready = Param.PickPlace_Max - 9 * Param.PickPlace_Loop - 580000;
	Param.PickPlace_Ready = Param.PickPlace_Max - (480000-5*Param.PickPlace_D+4320000);

	//判断参数
	Param.Servo_CtrlTime = 10;
	Param.Servo_TorqueCtrl = 64;
	Param.Servo_PosCtrl = 116;
	Param.Servo_InitPos = 510;
	Param.Servo_OverPos = 1080;
	Param.Servo_ErrorPos = 25;
	Param.Stretch_ErrorPos = 400;
	Param.PickPlace_ErrorPos = 500;
	Param.Lift_ErrorPos = 4000;
}


void Clamp_classdef::Control()
{
	//传感器检测结果获取
	A0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);//抬伸
	A1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);//取环
	
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


int D12_SetTime,D12_FwdNum;
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
		
		D12_SetTime = 0;
		D12_FwdNum = 0;
		step = 0;
		
		Init_Flag=0;
		Pick_Flag=0;
		Place_Point_Flag=0;
		Place_Flag=0;

		Set_TurnPlacel(0,TurnPlace_Servo.Posotion);
		TurnPlace_Servo.Torque_Flag = 0;
		if(servo_time == 5)
		{
			TurnPlace_Servo.Torque(false);
		}
		Clamp_Mode = Clamp_DisableMode;
		Clamp_Last_Mode = Clamp_DisableMode;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//默认
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
		else if(Ctrl_Flag == Param.Servo_PosCtrl &&\
		 TurnPlace_Servo.Torque_Flag == 1)
		{
			TurnPlace_Servo.Position_Control(ServoPos_Target);
		}
	}	
}


void Clamp_classdef::Tar_Update(void)
{
	switch((int)Clamp_Mode)
	{
		case Clamp_DisableMode:
		{
			
		}
		break;
		
		case Clamp_TransiMode:
		{
			UseTarget[0] = Stretch_Encider.getTotolAngle();
			UseTarget[1] = Lift_Motor.get_totalencoder();
			UseTarget[2] = PickPlace_Motor.get_totalencoder();
			D12_SetTime = 0;
			D12_FwdNum = 0;
			step = 0;
			
			Init_Flag=0;
			Pick_Flag=0;
			Place_Point_Flag=0;
			Place_Flag=0;
			Clamp_Mode = Clamp_Next_Mode;
			Clamp_Last_Mode = Clamp_TransiMode;
		}
		break;
		
		case Clamp_DebugMode:
		{
			//在伸出允许范围内才可以下调抬伸——下
			if(CTRL_DR16.Get_LY()<0)
			{
//				if(UseTarget[0] <= Param.Stretch_Max &&\
//				 UseTarget[0] >= Param.Stretch_Take &&\
//				 Top_Lift_Flag == 1)
				if(Top_Lift_Flag == 1)
				{
					if(TurnPlace_Servo.Torque_Flag == 0)//舵机使能
					{
						Set_TurnPlacel(Param.Servo_TorqueCtrl, TurnPlace_Servo.Posotion);
					}
					UseTarget[1] += (CTRL_DR16.Get_LY());
				}
				else
				{
					UseTarget[1] += 0;
				}
			}
			else
			{
				UseTarget[1] += (CTRL_DR16.Get_LY());
			}

			//伸出
			if(CTRL_DR16.Get_LX() > 0)
			{
				if(A0 == GPIO_PIN_RESET)
				{
					UseTarget[0] += (CTRL_DR16.Get_LX());
				}
				else
				{
					UseTarget[0] += 0;
				}
			}
			else
			{
				UseTarget[0] += (CTRL_DR16.Get_LX());
			}
			
			//取环
			if(CTRL_DR16.Get_DW()>0)
			{
				if(Top_PickPlace_Flag == 1)
				{
					UseTarget[2] += (-CTRL_DR16.Get_DW());
				}
				else
				{
					UseTarget[2] += 0;
				}
			}
			else
			{
				UseTarget[2] += (-CTRL_DR16.Get_DW());
			}
			
			//舵机
			if(CTRL_DR16.Get_RY()<0)
			{
				Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos);
			}
			else if(CTRL_DR16.Get_RY()>0)
			{
				Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos);
			}
			Clamp_Last_Mode = Clamp_DebugMode;
		}
		break;
		
		case Clamp_NDebugMode:
		{
			//在伸出允许范围内才可以下调抬伸——下
			if(CTRL_DR16.Get_LY()<0)
			{
//				if(UseTarget[0] <= Param.Stretch_Max &&\
//				 UseTarget[0] >= Param.Stretch_Take &&\
//				 Top_Lift_Flag == 1)
				if(Top_Lift_Flag == 1)
				{
					if(TurnPlace_Servo.Torque_Flag == 0)//舵机使能
					{
						Set_TurnPlacel(Param.Servo_TorqueCtrl, TurnPlace_Servo.Posotion);
					}
					UseTarget[1] += (CTRL_DR16.Get_LY());
				}
				else
				{
					UseTarget[1] += 0;
				}
			}
			else
			{
				UseTarget[1] += (CTRL_DR16.Get_LY());
			}
			

			//伸出
			if(Top_Lift_Flag && cs_yd &&\
				Lift_Motor.get_totalencoder() < Top_Lift-62000 &&\
			Lift_Motor.get_totalencoder() > Top_Lift-273261)
			{
				UseTarget[0] = LiftStretch_K * Lift_Motor.get_totalencoder() + LiftStretch_B;
			}
			if(CTRL_DR16.Get_LX() > 0)
			{
				if(A0 == GPIO_PIN_RESET)
				{
					UseTarget[0] += (CTRL_DR16.Get_LX());
				}
				else
				{
					UseTarget[0] += 0;
				}
			}
			else
			{
				UseTarget[0] += (CTRL_DR16.Get_LX());
			}
			
			//取环
			if(CTRL_DR16.Get_DW()>0)
			{
				if(Top_PickPlace_Flag == 1)
				{
					UseTarget[2] += (-CTRL_DR16.Get_DW());
				}
				else
				{
					UseTarget[2] += 0;
				}
			}
			else
			{
				UseTarget[2] += (-CTRL_DR16.Get_DW());
			}
			
			//舵机
			if(CTRL_DR16.Get_RY()<0)
			{
				Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos);
			}
			else if(CTRL_DR16.Get_RY()>0)
			{
				Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos);
			}
			Clamp_Last_Mode = Clamp_DebugMode;
		}
		break;
	
		case Clamp_AutoMode:
		{
			
			//与底盘联系进行自瞄发送与跑点
			if (Gimbal.D12 == GPIO_PIN_SET &&\
			Gimbal.Last_D12 == GPIO_PIN_RESET &&\
			(D12_SetTime == 0))
			{
				D12_SetTime++;
			}
			if(Gimbal.D12 == GPIO_PIN_SET && D12_SetTime)
			{
				D12_SetTime++;
			}
			else
			{
				D12_SetTime = 0;
			}
			
			//计数进行步骤
			if(D12_SetTime > 5)
			{
				D12_FwdNum++;
				switch(D12_FwdNum-1)
				{
					case 0:
						Pick_Flag = 1;
					break;
					
					case 1:Tar_Ring = Tar_MSeventy;Place_Flag = 1;break;
					case 2:Tar_Ring = Tar_MTen;Place_Flag = 1;break;
					case 3:Tar_Ring = Tar_LTen;Place_Flag = 1;break;
					case 4:Tar_Ring = Tar_LThirty;Place_Flag = 1;break;
					case 5:Tar_Ring = Tar_RThirty;Place_Flag = 1;break;
					case 6:Tar_Ring = Tar_RTen;Place_Flag = 1;break;
					case 7:Tar_Ring = Tar_DLThirty;Place_Flag = 1;break;
					case 8:Tar_Ring = Tar_DRThirty;Place_Flag = 1;break;
				}
				D12_SetTime=0;
			}
			
			Init();
			
			//叠加限制
			//舵机摆正测才能往下走
			if(abs((int)TurnPlace_Servo.Posotion - Param.Servo_InitPos) > Param.Servo_ErrorPos && AddVar[1] <= 0)
			{
				AddVar[1] = 0;
				UseTarget[1] = Lift_Motor.get_totalencoder();
			}
			
			AutoShoot();
			Place_NoShoot();
			
			if(Top_Lift_Flag){}else{if(AddVar[1]<0){(AddVar[1]=0);}}
			if(Top_PickPlace_Flag){}else{if(AddVar[2]<0){(AddVar[2]=0);}}
			
			UseTarget[0] += AddVar[0];
			UseTarget[1] += AddVar[1];
			UseTarget[2] += AddVar[2];
			//A0——Lift_Motor
			//A1——PickPlace_Motor
			Clamp_Last_Mode = Clamp_AutoMode;
		}
		break;
		
		case Clamp_LockMode:
		{
			UseTarget[0] += 0;
			UseTarget[1] += 0;
			UseTarget[2] += 0;
			Clamp_Last_Mode = Clamp_LockMode;
		}
		break;
	}

//	Var_SetAndLimit();


}


//目标限制
void Clamp_classdef::AngleLimit(void)
{
	//抬伸定点限位
	if(A0 == GPIO_PIN_RESET)
	{
		if(last_A0 != A0)
		{
			AddVar[1]=0;
			Top_Lift = Lift_Motor.get_totalencoder();
			Top_Lift_Flag = 1;
			
			LiftStretch_K=(Param.Stretch_Max-Param.Stretch_Min)/(273261-62000);
			LiftStretch_B=Param.Stretch_Min - LiftStretch_K * (Top_Lift-273261);
		}
		else if(Top_Lift_Flag == 1 && UseTarget[1]>=Top_Lift)
		{ 
			UseTarget[1] = Top_Lift;
		}
	}
	else if(Top_Lift_Flag == 1)//下限位
	{
		if(UseTarget[1]<=(Top_Lift-Param.Lift_Max))
		{
			UseTarget[1] = Top_Lift-Param.Lift_Max;
		}
	}
	last_A0 = A0;
	
	//取放限位
	if(A1 == GPIO_PIN_RESET)
	{
		if(last_A1 != A1)
		{
			AddVar[2]=0;
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

	last_A1 = A1;

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
	// if(A0 != GPIO_PIN_RESET)
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
		switch (step)
		{
			case 0:
				Gimbal.TarPos_Move(Tar_Ring);
				Shoot.Set_ShootServo(Tar_Ring);
				Shoot.Pull_Move(Tar_Ring);
				step=2;
			break;
				
			case 1:
				
				step=2;
			break;
			
			case 2:
				if(Stretch(Param.Stretch_Min,true)){step=3;}
			break;

			case 3:
				if(TurnPlace_Servo.Torque_Flag == 0)//舵机使能
				{
					Set_TurnPlacel(Param.Servo_TorqueCtrl, TurnPlace_Servo.Posotion);
				}
				if(Shoot.Shoot_Servo.Torque_Flag == 0)//舵机使能
				{
					Set_TurnPlacel(Param.Servo_TorqueCtrl, Shoot.Shoot_Servo.Posotion);
				}
				step=4;
			break;
			
			case 4:
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos)){step=5;}
			break;
			
			case 5:
				if(Lift(Lift_Motor.get_totalencoder(),true)){step=6;}
			break;
			
			case 6:
				if(PickPlace(PickPlace_Motor.get_totalencoder(),true)){step=7;}
			break;
			
			case 7:
				if(Gimbal.TarPos_Move(Tar_Ring) && Stretch(Param.Stretch_Min,true)){step=8;}
			break;
			
			case 8:
				if(Lift(Top_Lift-Param.Lift_Hold) && PickPlace(Top_PickPlace-Param.PickPlace_Release)\
					&& Shoot.Set_ShootServo(Tar_Ring))
				{
					Gimbal.Midpoint_Flag = 0;
					Init_Flag = 0;
					step=0;
					D12_SetTime = 0;
					D12_FwdNum = 0;
				}
			break;
		}
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//默认
	}
	else
	{
		Gimbal.Midpoint_Flag = 0;
	}
}


//三个点
//取环
//放环位置
//放环
//
int pick_wait_time;
void Clamp_classdef::Pick(void)
{
	if(Pick_Flag)
	{
		switch (step)
		{
			case 0:
					//(Lift(Lift_Motor.get_totalencoder(),true) |
					if(Gimbal.TarPos_Move(Tar_Ring) || 1){step = 1;}
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//默认
			break;
					
			case 1:
				if(Stretch(Param.Stretch_Min,true)){step = 2;}
			break;
			
			case 2:
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos)){step = 4;}
			break;
			
			case 3:
//				if(PickPlace(PickPlace_Motor.get_totalencoder(),true)){step = 4;}
			break;
			
			case 4:
				if(Gimbal.TarPos_Move(Tar_Ring)){step = 5;}
			break;

			case 5:
				if(PickPlace(Top_PickPlace-Param.PickPlace_Release)){step = 6;}
			break;
				
			case 6:
				if(Lift(Top_Lift-Param.Lift_Max))
				{
					pick_wait_time++;
					if(pick_wait_time>Param.Lift_PickWaitTime){pick_wait_time=0;step = 7;}
				}
			break;

			case 7:
				if(Lift(Top_Lift-62000,false) & PickPlace(Top_PickPlace-Param.PickPlace_Release))
				{
					step = 0;
					Pick_Flag =0;
					Place_Point_Flag = 1;
				}
				if(Lift_Motor.get_totalencoder() > Top_Lift-800000)
				{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);// 第一次转正，底盘去中间
				}
			break;
		
			default:
			break;
		}
	}
	else
	{
	}
}

void Clamp_classdef::Place_Point(void)
{
	if(Place_Point_Flag)
	{
		switch (step)
		{
			case 0:
				if(Gimbal.TarPos_Move(Tar_Ring) || 1){step = 1;}
			break;
				
			case 1:
//				if(Lift(Top_Lift,true)){step = 2;}
			if(Lift(Top_Lift-62000,false)){step = 2;}
			break;
				
			case 2:
				if(Stretch(Param.Stretch_Min,true)){step = 3;}
			break;
			
			case 3:
				if(PickPlace(Top_PickPlace-Param.PickPlace_Release)){step = 4;}
			break;
			
			case 4:
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos)){step = 5;}
			break;
			
			case 5:
				if(Gimbal.TarPos_Move(Tar_Ring)){step = 8;}
			break;

			case 6:
				if(PickPlace(Top_PickPlace-Param.PickPlace_Ready)){step = 8;}
			break;
			
			case 7:
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos)){step = 8;}
			break;
			
			case 8:
//				if(Stretch(Param.Stretch_Min))
				if(PickPlace(Top_PickPlace-Param.PickPlace_Ready) & Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos) & Stretch(Param.Stretch_Max))
				{
					step = 0;
					Place_Point_Flag = 0;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//恢复准备下一次转正
					PickPlace_Num = 0;
				}
			break;

			default:
			break;
		}
	}
}

int shoot_wait_time;
void Clamp_classdef::Place_NoShoot(void)
{
	if(Place_Flag)
	{
		switch (step)
		{
			case 0:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
				Shoot.Pull_Move(Tar_Ring);
				Shoot.Set_ShootServo(Tar_Ring);
				if(Lift(Top_Lift-62000,false)){step = 1;PickPlace_Num++;}
			break;
			
			case 1:
				if(PickPlace(now_place)){step = 2;}
			break;
			
			case 2:
				Shoot.Set_ShootServo(Tar_Ring);
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos)){step = 3;}
			break;
			
			case 3:
				if(Gimbal.TarPos_Move(Tar_Ring) & Shoot.Pull_Move(Tar_Ring) & Shoot.Set_ShootServo(Tar_Ring)){step = 4;}
			break;
			
			case 4:
				if(PickPlace_Num == 1)
				{
					if(PickPlace(now_place-(Param.PickPlace_A1+Param.PickPlace_D)))
					{
						shoot_wait_time++;
						if(shoot_wait_time>Param.Shoot_WaitTime)
						{
							shoot_wait_time=0;
							step = 5;
						}
					}
				}
				else
				{
					if(PickPlace(now_place-(Param.PickPlace_A1+Param.PickPlace_D*(PickPlace_Num-2))))
					{
						shoot_wait_time++;
						if(shoot_wait_time>Param.Shoot_WaitTime)
						{
							shoot_wait_time=0;
							step = 5;
						}
					}
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			break;

			case 5:
				if(Shoot.Set_Shoot(true))
				{
					step = 0;
					Place_Flag = 0;
					now_place = UseTarget[2];
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
				}
			break;

			default:
			break;
		}
	}
	else
	{
		shoot_wait_time=0;
		Homeing_Flag = 0;
		Place_Flag = 0;
//		now_place = PickPlace_Motor.get_totalencoder();
		now_place = UseTarget[2];
	}
}




uint32_t chazhi[9],dqz[9];
void Clamp_classdef::AutoShoot(void)
{
	if(Pick_Flag)
	{
		switch (step)
		{
			case 0:
					Gimbal.TarPos_Move(Tar_Ring);
					step = 1;dqz[0]=Get_SystemTimer();
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//默认
			break;
					
			case 1:
				if(Stretch(Param.Stretch_Min,true)){step = 2;dqz[1]=Get_SystemTimer();chazhi[0]=dqz[1]-dqz[0];}
			break;
			
			case 2:
				
				
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos)){step = 4;dqz[2]=Get_SystemTimer();chazhi[1]=dqz[2]-dqz[1];}
			break;
			
			case 3:
				
//				if(PickPlace(PickPlace_Motor.get_totalencoder(),true)){step = 4;}
			break;
			
			case 4:
				
				
				if(Gimbal.TarPos_Move(Tar_Ring)){step = 5;dqz[3]=Get_SystemTimer();chazhi[2]=dqz[3]-dqz[2];}
			break;

			case 5:
				if(PickPlace(Top_PickPlace-Param.PickPlace_Release)){step = 6;dqz[4]=Get_SystemTimer();chazhi[3]=dqz[4]-dqz[3];}//应该可优
			break;
				
			case 6:
				if(Lift(Top_Lift-Param.Lift_Max))
				{
					pick_wait_time++;
					if(pick_wait_time>Param.Lift_PickWaitTime){pick_wait_time=0;step = 7;dqz[5]=Get_SystemTimer();chazhi[4]=dqz[5]-dqz[4];}
				}
			break;

			case 7:
				if(Lift(Top_Lift-62000,false) &\
					PickPlace(Top_PickPlace-Param.PickPlace_Ready))
				{
					Stretch(Param.Stretch_Max);
					step = 8;
					dqz[6]=Get_SystemTimer();
					chazhi[5]=dqz[6]-dqz[5];
				}
				if(Top_Lift_Flag &&\
				Lift_Motor.get_totalencoder() < Top_Lift &&\
				Lift_Motor.get_totalencoder() > Top_Lift-273261)
				{
					if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos))
					{
						Stretch(LiftStretch_K * Lift_Motor.get_totalencoder() + LiftStretch_B);
					}
				}
				
				if(Lift_Motor.get_totalencoder() > Top_Lift-800000)
				{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);// 第一次转正，底盘去中间
				}
			break;
				
			case 8:
				if(Stretch(Param.Stretch_Max)){step = 9;dqz[7]=Get_SystemTimer();chazhi[6]=dqz[7]-dqz[6];}
			break;
				
			case 9:
				
				if(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos))
				{
						Pick_Flag=0;
						step = 0;
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//恢复准备下一次转正
						PickPlace_Num = 0;
						dqz[8]=Get_SystemTimer();
						chazhi[7]=dqz[8]-dqz[7];
				}
			break;
		
			default:
			break;
		}
	}
}

bool Clamp_classdef::Stretch(float pos,bool datum,float*rp)
{
	if(datum)//初始化——去取环点
	{
		AddVar[0] -= Param.Stretch_Speed_Ramp;
		if(AddVar[0] < -Param.Stretch_Speed){AddVar[0]=-Param.Stretch_Speed;}
		
		if(UseTarget[0] <= Param.Stretch_Min && \
		abs(Stretch_Encider.getTotolAngle() - Param.Stretch_Min) <=\
		Param.Lift_ErrorPos)
		{
			AddVar[0] = 0;
			UseTarget[0] = Param.Stretch_Min;
			return true;
		}
		return false;
	}
	else//去范围内任意位置
	{
		if(UseTarget[0] <= pos-Param.Stretch_Speed)//往内
		{
			AddVar[0] += Param.Stretch_Speed_Ramp;
			if(AddVar[0] >= Param.Stretch_Speed){AddVar[0]=Param.Stretch_Speed;}
			return false;
		}
		else if(UseTarget[0] >= pos+Param.Stretch_Speed)//往外
		{
			AddVar[0] -= Param.Stretch_Speed_Ramp;
			if(AddVar[0] <= -Param.Stretch_Speed){AddVar[0]=-Param.Stretch_Speed;}
			return false;
		}
		else
		{
			AddVar[0] = 0;
			UseTarget[0] = pos;
			if(rp != NULL)
			{
				*rp = pos;
			}

			if(abs(Stretch_Encider.getTotolAngle() - pos) <= Param.Stretch_ErrorPos)
			{
				return true;
			}
			return false;
		}
	}
}

bool Clamp_classdef::Lift(float pos,bool datum,float*rp)
{
	if(datum)
	{
		if(A0 != GPIO_PIN_RESET)
		{
			//上升
			if(Top_Lift_Flag)
			{
				//上升快到时进行减速——减速段
				if(UseTarget[1]+Param.Lift_Max_Speed > Top_Lift-Param.Lift_Sub_SPortion)
				{
					AddVar[1] -= Param.Lift_subSpeed_Ramp;
					if(AddVar[1] <= LIFT_INIT_SPEED){AddVar[1] = LIFT_INIT_SPEED;}
				}
				else//起步累叠加
				{
					AddVar[1] += Param.Lift_addSpeed_Ramp;
					if(AddVar[1] >= Param.Lift_Max_Speed){AddVar[1] = Param.Lift_Max_Speed;}
				}
			}
			else
			{
				//慢速进行第一次初始化
				AddVar[1] = LIFT_INIT_SPEED;
			}
				
			return false;
		}
		else 
		{
			UseTarget[1] = Lift_Motor.get_totalencoder();
			AddVar[1] = 0;
			return true;
		}
	}
	else
	{
		if(UseTarget[1] >= pos+Param.Lift_Min_Speed)//下降
		{
			//下降快到时进行减速
			if(UseTarget[1]-Param.Lift_Min_Speed < pos+Param.Lift_Sub_XPortion)
			{
				AddVar[1] += Param.Lift_subSpeed_Ramp;
				if(AddVar[1] >= -LIFT_INIT_SPEED){AddVar[1] = -LIFT_INIT_SPEED;}
			}
			else//起步累叠加
			{
				AddVar[1] -= Param.Lift_addSpeed_Ramp;
				if(AddVar[1] <= -Param.Lift_Min_Speed){AddVar[1] = -Param.Lift_Min_Speed;}
			}
			return false;
		}
		else if(UseTarget[1] <= pos-Param.Lift_Max_Speed)//上升
		{
			//没写是因为其实目前没有一个步骤需要用到这个上升到非初始化位置的
			//只有起步用的累叠加
			//现在用到了！！！但不用斜坡——斜坡效果不好
//			AddVar[1] += LIFT_INIT_SPEED;
			
			//上升快到时进行减速——减速段
			if(UseTarget[1]+Param.Lift_Max_Speed > pos-Param.Lift_Sub_SPortion)
			{
				AddVar[1] -= Param.Lift_subSpeed_Ramp;
				if(AddVar[1] <= LIFT_INIT_SPEED){AddVar[1] = LIFT_INIT_SPEED;}
			}
			else//起步累叠加
			{
				AddVar[1] += Param.Lift_addSpeed_Ramp;
				if(AddVar[1] >= Param.Lift_Max_Speed){AddVar[1] = Param.Lift_Max_Speed;}
			}
			return false;
		}
		else
		{
			AddVar[1] = 0;
			UseTarget[1] = pos;
			if(rp != NULL)
			{
				*rp = pos;
			}

			if(abs(Lift_Motor.get_totalencoder() - pos) < Param.Lift_ErrorPos)
			{
				return true;
			}
			return false;
		}
	}
}

bool Clamp_classdef::PickPlace(float pos,bool datum,float*rp)
{
	if(datum)
	{
		if(A1 != GPIO_PIN_RESET)
		{
			AddVar[2] = Param.PickPlace_Speed;
			return false;
		}
		else 
		{
			UseTarget[2] = PickPlace_Motor.get_totalencoder();
			AddVar[2] = 0;
			return true;
		}
	}
	else
	{
		if(UseTarget[2] <= pos-Param.PickPlace_Speed)
		{
			AddVar[2] = Param.PickPlace_Speed;
		}
		else if(UseTarget[2] >= pos+Param.PickPlace_Speed)
		{
			AddVar[2] = -Param.PickPlace_Speed;
		}
		else
		{
			AddVar[2] = 0;
			UseTarget[2] = pos;
			if(rp != NULL)
			{
				*rp = pos;
			}

			if(abs(PickPlace_Motor.get_totalencoder() - pos) < Param.PickPlace_ErrorPos)
			{
				return true;
			}
		}
		
		if(UseTarget[2]+AddVar[2] < Top_PickPlace-Param.PickPlace_Max)
		{
			AddVar[2] = 0;
			UseTarget[2] = Top_PickPlace-Param.PickPlace_Max;
			return true;
		}
		
		return false;
	}
}

bool Clamp_classdef::Set_TurnPlacel(int mode,int position)//位置设置
{
	if(mode == Param.Servo_PosCtrl && TurnPlace_Servo.Torque_Flag)
	{
		Ctrl_Flag = mode;
		ServoPos_Target = position;
		if(abs((int)TurnPlace_Servo.Posotion - position) <= Param.Servo_ErrorPos)
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

