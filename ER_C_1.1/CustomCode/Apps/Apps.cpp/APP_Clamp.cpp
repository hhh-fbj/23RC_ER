
#include "APP_Clamp.h"
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include  <stdint.h>
#include  <string.h>

#include "tim.h"
#include "stm32f4xx_hal.h"

#include "APP_DR16.h"

#include "System_DataPool.h"

//С����ȡ��������ʼ��ʱ��ȡ�����������뷢���̷��ͽӴ�

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
	
	//��������
	Param.Stretch_Hold = 1360000;
	Param.Stretch_Speed = 400;
	Param.Lift_Max = 812000;//770000;
	Param.Lift_Hold = 500000;//770000;
	Param.Lift_Speed = 660;
	Param.Lift_PickWaitTime = 50;
	Param.PickPlace_Max = 5300000;//5480000;
	Param.PickPlace_Release = 200000;//250000;
	Param.PickPlace_Loop = 455000;//485000;
	Param.PickPlace_Speed = 4000;
	Param.Shoot_WaitTime = 50;

	//�������
	Param.Stretch_Max = Param.Stretch_Hold - 8848;
	Param.Stretch_Take = Param.Stretch_Hold - 9048;
	Param.Stretch_Shoot = Param.Stretch_Hold - 109981;//109981
	Param.Stretch_Min = Param.Stretch_Hold - 128052;//138052
	Param.PickPlace_Ready = Param.PickPlace_Max - 9 * Param.PickPlace_Loop;

	//�жϲ���
	Param.Servo_CtrlTime = 10;
	Param.Servo_TorqueCtrl = 64;
	Param.Servo_PosCtrl = 116;
	Param.Servo_InitPos = 1898;
	Param.Servo_OverPos = 2444;//1579
	Param.Servo_ErrorPos = 20;
	Param.Stretch_ErrorPos = 400;
	Param.PickPlace_ErrorPos = 200;
	Param.Lift_ErrorPos = 500;
}


void Clamp_classdef::Control()
{
	//�������������ȡ
	I7 = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7);
	E14 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);

	//�����ֵ�����������ƽ������
	servo_time++;
	if(servo_time >= Param.Servo_CtrlTime)
	{
		TurnPlace_Servo.ReadPosition();
		servo_time = 0;
	}

	//������
	if(ProblemDetection()){return;}

	//Ŀ�����
	Tar_Update();

	//�������
	Servo_Control();

	//��λ
	AngleLimit();

	//PID����
	Motor_PIDCalc();
}


int I6_SetTime,I6_FwdNum;
uint8_t step;
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
		//��ֵ����
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
		
		I6_SetTime = 0;
		I6_FwdNum = 0;
		step = 0;
		
		Init_Flag=0;
		Pick_Flag=0;
		Place_Point_Flag=0;
		Place_Flag=0;

		Set_TurnPlacel(0,Param.Servo_InitPos);
		TurnPlace_Servo.Torque_Flag = 0;
		if(servo_time == 5)
		{
			TurnPlace_Servo.Torque(false);
		}

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


void Clamp_classdef::Tar_Update(void)
{
	switch((int)Clamp_Mode)
	{
		case Clamp_TransiMode:
			UseTarget[0] = Stretch_Encider.getTotolAngle();
			UseTarget[1] = Lift_Motor.get_totalencoder();
			UseTarget[2] = PickPlace_Motor.get_totalencoder();
			I6_SetTime = 0;
			I6_FwdNum = 0;
			step = 0;
			
			Init_Flag=0;
			Pick_Flag=0;
			Place_Point_Flag=0;
			Place_Flag=0;
			Clamp_Mode = Clamp_Next_Mode;
		break;
		
		case Clamp_DebugMode:
			
			//���������Χ�ڲſ����µ�̧�졪����
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

			//��
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
					UseTarget[2] += 0;
				}
			}
			else
			{
				UseTarget[2] += (-CTRL_DR16.Get_DW());
			}
		break;
		
		case Clamp_AutoMode:
			//���ʹ��
			if(TurnPlace_Servo.Torque_Flag == 0)
			{
				Set_TurnPlacel(Param.Servo_TorqueCtrl, Param.Servo_InitPos);
			}
			
			//�������ϵ�������鷢�����ܵ�
			
			if (Gimbal.I6 == GPIO_PIN_SET &&\
			Gimbal.Last_I6 == GPIO_PIN_RESET &&\
			(I6_SetTime == 0))
			{
				I6_SetTime++;
			}
			if(Gimbal.I6 == GPIO_PIN_SET && I6_SetTime)
			{
				I6_SetTime++;
			}
			else
			{
				I6_SetTime = 0;
			}
			
			
			if(I6_SetTime > 5)
			{
				I6_FwdNum++;
				switch(I6_FwdNum)
				{
					case 1:
						Pick_Flag = 1;
					break;
					
					case 2:
					case 3:
					case 4:
					case 5:
						if(Auto_Aim_Flag)
						{
							if(Vision.aim==0)
							{
								Vision.aim=1;
							}
						}
						else
						{
							Place_Flag = 1;
						}
					break;
				}
				I6_SetTime=0;
			}
			
			
				
			
			Init();
			Pick();
			Place_Point();
			Place();

			//��������
			//������������������
			if(abs((int)TurnPlace_Servo.Posotion - Param.Servo_InitPos) > Param.Servo_ErrorPos && AddVar[1] >= 0)
			{
				AddVar[1] = 0;
				UseTarget[1] = Lift_Motor.get_totalencoder();
			}
			if(Top_Lift_Flag){}else{if(AddVar[1]>0){(AddVar[1]=0);}}
			if(Top_PickPlace_Flag){}else{if(AddVar[2]<0){(AddVar[2]=0);}}
			
			UseTarget[0] += AddVar[0];
			UseTarget[1] += AddVar[1];
			UseTarget[2] += AddVar[2];
			//E14����Lift_Motor
			//I7����PickPlace_Motor
		break;
		
		case Clamp_LockMode:
			UseTarget[0] += 0;
			UseTarget[1] += 0;
			UseTarget[2] += 0;
		break;
	}

//	Var_SetAndLimit();


}


//Ŀ������
void Clamp_classdef::AngleLimit(void)
{
	//������λ
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
	
	//ȡ����λ
	if(I7 == GPIO_PIN_SET)
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
	//�����λ
	if(UseTarget[0] >= Param.Stretch_Max)
	{
		UseTarget[0] = Param.Stretch_Max;
	}
	if(UseTarget[0] <= Param.Stretch_Min)
	{
		UseTarget[0] = Param.Stretch_Min;
	}

	//�������
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
		switch (step)
		{
			case 0:
			if((Lift(Lift_Motor.get_totalencoder(),true) & Gimbal.TarPos_Move(0)) &&\
			Stretch(Param.Stretch_Max,true) &&\
			Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos) &&\
		 (PickPlace(PickPlace_Motor.get_totalencoder(),true)&&\
			Gimbal.TarPos_Move(0)))
			{
//				Gimbal.Midpoint_Flag = 0;
//				Init_Flag = 0;
				step=1;
			}
			break;
			
			case 1:
				if(Lift(Top_Lift+Param.Lift_Hold))
				{
					Gimbal.Midpoint_Flag = 0;
					Init_Flag = 0;
					step=0;
				}
			break;
		}
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//Ĭ��
	}
	else
	{
		Gimbal.Midpoint_Flag = 0;
	}
}


//������
//ȡ��
//�Ż�λ��
//�Ż�
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
					if(Gimbal.TarPos_Move(0) &&\
					Stretch(Param.Stretch_Max,true) &&\
					Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos) &&\
					(PickPlace(PickPlace_Motor.get_totalencoder(),true) &\
					Gimbal.TarPos_Move(0)))
					{
						step = 1;
					}
			break;

			case 1:
				if(PickPlace(Top_PickPlace-Param.PickPlace_Release) && \
				Lift(Param.Lift_Max+Top_Lift))
				{
					pick_wait_time++;
					if(pick_wait_time>Param.Lift_PickWaitTime){pick_wait_time=0;step = 2;}
				}
			break;

			case 2:
				if(Lift(Top_Lift, true) & PickPlace(Top_PickPlace-Param.PickPlace_Release))
				{
					step = 0;
					Pick_Flag =0;
					Place_Point_Flag = 1;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);// ��һ��ת��������ȥ�м�
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
					if((Lift(Lift_Motor.get_totalencoder(),true) | Gimbal.TarPos_Move(0)) &&\
					(Stretch(Param.Stretch_Max,true) &\
					PickPlace(Top_PickPlace-Param.PickPlace_Release)) &&\
					(Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_InitPos) &\
					Gimbal.TarPos_Move(0)))
					{
						step = 1;
					}
			break;

			case 1:
				if(PickPlace(Top_PickPlace-Param.PickPlace_Ready) && \
				Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos)&& \
				Stretch(Param.Stretch_Min))
				{
					step = 0;
					Place_Point_Flag = 0;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//�ָ�׼����һ��ת��
				}
			break;

			default:
			break;
		}
	}
}

int shoot_wait_time;
void Clamp_classdef::Place(void)
{
	if(Place_Flag)
	{
		switch (step)
		{
			case 0:
					if(Lift(Lift_Motor.get_totalencoder(),true) &&\
					(Stretch(Param.Stretch_Min) &\
					PickPlace(now_place)) &&\
					Set_TurnPlacel(Param.Servo_PosCtrl, Param.Servo_OverPos))
					{
						now_place = UseTarget[2];
						step = 1;
						if(Shoot.Shoot_Place_Flag!=0){Shoot.Shoot_Place_Flag=0;}
					}
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
			break;

			case 1:
				if(Shoot.Shoot_Place_Flag == 0)
				{
					Shoot.Shoot_Place_Flag = 1;
				}
				if((((Shoot.Shoot_Place_Flag==2) & \
				Gimbal.TarPos_Move(I6_FwdNum))   & \
				Shoot.Pull_Move(I6_FwdNum)) && \
				PickPlace(now_place-Param.PickPlace_Loop))
				
//				if(((Shoot.Shoot_Place_Flag==2) & \
//				Gimbal.TarPos_Move(I6_FwdNum)) && \
//				PickPlace(now_place-Param.PickPlace_Loop))
				
//				if((Shoot.Shoot_Place_Flag==2) && \
//				PickPlace(now_place-Param.PickPlace_Loop))
				{
					shoot_wait_time++;
					if(shoot_wait_time>Param.Shoot_WaitTime && Shoot.Pull_Move(I6_FwdNum))
					{
						shoot_wait_time=0;
						Shoot.Set_Shoot(true);
						step = 2;
					}
				}
				else if(UseTarget[2]<=Top_PickPlace-Param.PickPlace_Max)
				{
					if((Shoot.Shoot_Place_Flag==2) && \
					PickPlace(Top_PickPlace-Param.PickPlace_Max))
					{
						shoot_wait_time++;
						if(shoot_wait_time>Param.Shoot_WaitTime)
						{
							shoot_wait_time=0;
							Shoot.Set_Shoot(true);
							step = 2;
						}
					}
				}
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
			break;

			case 2:
				if(Homeing_Flag)
				{
					step = 0;
					Homeing_Flag = 0;
					Place_Flag = 0;
					now_place = UseTarget[2];
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
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
		now_place = UseTarget[2];
	}
}



bool Clamp_classdef::Stretch(float pos,bool datum,float*rp)
{
	if(datum)
	{
		AddVar[0] = Param.Stretch_Speed;
		if(UseTarget[0] >= Param.Stretch_Max && \
		abs(Stretch_Encider.getTotolAngle() - Param.Stretch_Max) <= \
		Param.Lift_ErrorPos )
		{
			AddVar[0] = 0;
			UseTarget[0] = Param.Stretch_Max;
			return true;
		}
		return false;
	}
	else
	{
		if(UseTarget[0] <= pos-Param.Stretch_Speed)
		{
			AddVar[0] = Param.Stretch_Speed;
			return false;
		}
		else if(UseTarget[0] >= pos+Param.Stretch_Speed)
		{
			AddVar[0] = -Param.Stretch_Speed;
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
		if(E14 != GPIO_PIN_RESET)
		{
			AddVar[1] = -Param.Lift_Speed;
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
		if(UseTarget[1] <= pos-Param.Lift_Speed)
		{
			AddVar[1] = Param.Lift_Speed;
			return false;
		}
		else if(UseTarget[1] >= pos+Param.Lift_Speed)
		{
			AddVar[1] = -Param.Lift_Speed;
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
		if(I7 != GPIO_PIN_SET)
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
			return false;
		}
		else if(UseTarget[2] >= pos+Param.PickPlace_Speed)
		{
			AddVar[2] = -Param.PickPlace_Speed;
			return false;
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
			return false;
		}
	}
}

bool Clamp_classdef::Set_TurnPlacel(int mode,int position)
{
	if(mode && TurnPlace_Servo.Torque_Flag)
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


