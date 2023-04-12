#include "APP_Auto.h"

#include "System_DataPool.h"


void Auto_classdef::Process(void)
{
	if(DevicesMonitor.Get_State(CHAS_POSTURE_MONITOR) == Off_line &&\
	DevicesMonitor.Get_State(CHAS_ANALOG_MONITOR) == Off_line)
	{
		Chassis.NO_PostureMode = 99;
		return;
	}
	
    switch (Mode)
    {
        case AUTO_DisablelMode:

        break;

        case AUTO_TestMode:
			
			Text_Step();
        break;

        case AUTO_SemiAutolMode:
        break;

        case AUTO_AutoGameMode:
            Game_Step();
        break;
    
        default:
        break;
    }
}

void Auto_classdef::Text_Step(void)
{
	switch(text_step)
	{
		case 0:
			if(startFlag == 1 && overFlag == 0)
			{
				Chassis.NO_PostureMode = 66;
				Chassis.POS_PID[Posture_X][PID_Outer].Target = Spots[0][0];
				Chassis.POS_PID[Posture_Y][PID_Outer].Target = Spots[0][1];
				Chassis.POS_PID[Posture_W][PID_Outer].Target = Spots[0][2];
				if(Detection_Point_text(Spots[0])){overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;text_step = 1;}
			}
			else if(startFlag == 0 &&overFlag == 1)
			{
				overFlag = 0;
			}
			else if(startFlag == 0 && overFlag == 0)
			{
				Chassis.NO_PostureMode = 0;
				Chassis.POS_PID[Posture_X][PID_Outer].Target = Posture.POS_X();
				Chassis.POS_PID[Posture_Y][PID_Outer].Target = Posture.POS_Y();
				Chassis.POS_PID[Posture_W][PID_Outer].Target = Posture.POS_W();
			}
		break;

		case 1:
			Left_PickIdea();
		break;
			
		case 2:
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
		break;

		case 6:

		break;
		
		case 100:
			
		break;
	}
}

int time_AUTO;
void Auto_classdef::Game_Step(void)
{
	if(SX == 99	)
	{
		switch(step)
		{
			case AUTO_TakeRing_PointTwo:step = AUTO_TakeRing; startFlag=0; break;
			case AUTO_TakeRing:step = AUTO_ShootRing_PointOne; startFlag=0; break;
			case AUTO_ShootRing_PointOne:step = AUTO_Aim_One; startFlag=0; break;
			case AUTO_Aim_One:step = AUTO_Revise_One; startFlag=0; break;
			case AUTO_Revise_One:step = AUTO_Shoot_One; startFlag=0; break;
			case AUTO_Shoot_One:step = AUTO_ShootRing_PointTwo; startFlag=0; break;

			case AUTO_Aim_Two:step = AUTO_Revise_Two; startFlag=0; break;
			case AUTO_Revise_Two:step = AUTO_Shoot_Two; startFlag=0; break;
			case AUTO_Shoot_Two:step = AUTO_ShootRing_PointThree; startFlag=0; break;

			case AUTO_Aim_Three:step = AUTO_Revise_Three; startFlag=0; break;
			case AUTO_Revise_Three:step = AUTO_Shoot_Three; startFlag=0; break;
			case AUTO_Shoot_Three:step = AUTO_ShootRing_PointFour; startFlag=0; break;

			case AUTO_Aim_Four:step = AUTO_Revise_Four; startFlag=0; break;
			case AUTO_Revise_Four:step = AUTO_Shoot_Four; startFlag=0; break;
			case AUTO_Shoot_Four:step = AUTO_ShootRing_PointFive; startFlag=0; break;

			case AUTO_Aim_Five:step = AUTO_Revise_Five; startFlag=0; break;
			case AUTO_Revise_Five:step = AUTO_Shoot_Five; startFlag=0; break;
			case AUTO_Shoot_Five:step = AUTO_ShootRing_PointSix; startFlag=0; break;
			
			case AUTO_Aim_Six:step = AUTO_Revise_Six; startFlag=0; break;
			case AUTO_Revise_Six:step = AUTO_Shoot_Six; startFlag=0; break;
			case AUTO_Shoot_Six:step = AUTO_Other; startFlag=0; break;

		}
		startFlag = 0;Vx=Vy=Vw=0;WallFlag=0;
		Chassis.NO_PostureMode = 99;
		SX = 1;
	}

    switch(step)
    {
        case AUTO_Origin:
			if(Chassis.over_init){step = AUTO_TakeRing_PointOne;}
		break;

        case AUTO_TakeRing_PointOne:
			Point_Run(Loop_Point_One, AUTO_TakeRing_PointTwo);
		break;//��Ҫ������
					
        case AUTO_TakeRing_PointTwo:
			MicroWall(-1, AUTO_TakeRing);
		break;

        case AUTO_TakeRing:
			//ȡ��
		break;

        case AUTO_ShootRing_PointOne:
			Point_Run(Batting_Point_One, AUTO_Aim_One);
		break;//��Ҫ������

        case AUTO_Aim_One:
			MicroWall(0, AUTO_Shoot_One);
		break;
        case AUTO_Revise_One:
			//������λ�û�������
		break;
        case AUTO_Shoot_One:
			//��׼+����
			Chassis.NO_PostureMode = 2;
		break;
        case AUTO_ShootRing_PointTwo:
			Point_Run(Batting_Point_Two, AUTO_Aim_Two);
		break;//��Ҫ������
        case AUTO_Aim_Two:
			MicroWall(0, AUTO_Shoot_Two);
		break;
        case AUTO_Revise_Two:
			//������λ�û�������
		break;

        case AUTO_Shoot_Two:
			//����
			Chassis.NO_PostureMode = 2;
		break;

        case AUTO_ShootRing_PointThree:
			Point_Run(Batting_Point_Three, AUTO_Aim_Three);
		break;//��Ҫ������

        case AUTO_Aim_Three:
			MicroWall(0, AUTO_Shoot_Three);
		break;
        case AUTO_Revise_Three:
		break;
        case AUTO_Shoot_Three:
			//����
			Chassis.NO_PostureMode = 2;
		break;
		
        case AUTO_ShootRing_PointFour:
			Point_Run(Batting_Point_Four, AUTO_Aim_Four);
		break;//��Ҫ������

        case AUTO_Aim_Four:
			MicroWall(0, AUTO_Shoot_Three);
		break;
        case AUTO_Revise_Four:
		break;
        case AUTO_Shoot_Four:
			//����
			Chassis.NO_PostureMode = 2;
		break;

        case AUTO_ShootRing_PointFive:
			Point_Run(Batting_Point_Five, AUTO_Aim_Five);
		break;//��Ҫ������

        case AUTO_Aim_Five:
			MicroWall(0, AUTO_Shoot_Five);
		break;

        case AUTO_Revise_Five:
		break;

        case AUTO_Shoot_Five:
			//����
			Chassis.NO_PostureMode = 2;
        break;
				
		case AUTO_ShootRing_PointSix:
			Point_Run(Batting_Point_Six, AUTO_Aim_Six);
		break;

		case AUTO_Aim_Six:
			MicroWall(0, AUTO_Shoot_Six);
		break;

		case AUTO_Revise_Six:break;

        case AUTO_Shoot_Six:
			Chassis.NO_PostureMode = 2;	
		break;

        case AUTO_Other:
        break;

        case AUTO_Test:
        break;
    }
}

void Auto_classdef::Point_Run(int point[3],AUTO_Step_e AUTO_step)
{
	if(startFlag == 1 && overFlag == 0)
	{
		Chassis.NO_PostureMode = 0;
		Chassis.POS_PID[Posture_X][PID_Outer].Target = point[0];
		Chassis.POS_PID[Posture_Y][PID_Outer].Target = point[1];
		Chassis.POS_PID[Posture_W][PID_Outer].Target = point[2];
		if(Detection_Point(Loop_Point_One)){overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;step = AUTO_step;}
	}
	else if(startFlag == 0 &&overFlag == 1)
	{
		overFlag = 0;
	}
	else if(startFlag == 0 && overFlag == 0)
	{
		Chassis.NO_PostureMode = 0;
		Chassis.POS_PID[Posture_X][PID_Outer].Target = Posture.POS_X();
		Chassis.POS_PID[Posture_Y][PID_Outer].Target = Posture.POS_Y();
		Chassis.POS_PID[Posture_W][PID_Outer].Target = Posture.POS_W();
	}
}

void Auto_classdef::MicroWall(int dir,AUTO_Step_e AUTO_step)
{
	if(startFlag == 1 && overFlag == 0)
	{
		Chassis.NO_PostureMode = 3;
		if(Chassis.EdgeDete[dir*(-2)+2] && Chassis.EdgeDete[dir*(-2)+3])
		{
			Vx = dir*550;Vw = 0;
			if(dir == 0){Vy=550;}else{Vy=0;}
		}
		else
		{
			if(Chassis.EdgeDete[dir*(-2)+2] == GPIO_PIN_RESET &&\
			Chassis.EdgeDete[dir*(-2)+3] == GPIO_PIN_RESET && WallFlag == 0)
			{
				if(dir == 0)
				{
					Vx = 0;Vw = 0;
					if(Vy>10){Vy -= 1;}
					else{WallFlag = 1;}
				}
				else
				{
					Vy = 0;Vw = 0;
					if(Vx<-10){Vx += 1;}
					else if(Vx>10){Vx -= 1;}
					else{WallFlag = 1;}
				}
			}
			else if(Chassis.EdgeDete[dir*(-2)+2] == GPIO_PIN_SET)//����
			{
				Vx = dir*440;Vy = 0;Vw = (-1)^dir*220;
			}
			else if(Chassis.EdgeDete[dir*(-2)+3] == GPIO_PIN_SET)//����
			{
				Vx = dir*440;Vy = 0;Vw = 1^dir*220;
			}
			else if(WallFlag)
			{
				Vx = dir*165;Vy = ((-1)^dir)*440;Vw=0;
				if(dir == 0)
				{
					WallTime++;
					if(WallTime > 150)
					{overFlag = 1;startFlag = 0;Vx=Vy=Vw=0;\
					step = AUTO_step;WallFlag=WallTime=0;}

				}
				else
				{
					if(0)
					{overFlag = 1;startFlag = 0;Vx=Vy=Vw=0;\
					step = AUTO_step;WallFlag=WallTime=0;}
				}
			}
		}
	}
	else if(startFlag == 0 && overFlag == 1)
	{
		overFlag = 0;
	}
}

uint8_t Auto_classdef::Detection_Point_text(int *Spots)
{
	if(Posture.POS_X() < Spots[0])
    {
		return 1;
    }
	else 
	{
		return 0;
	}
}

uint8_t Auto_classdef::Detection_Point(int *Spots)
{
	if(abs(Posture.POS_X() - Spots[0]) < 100 &&\
    abs(Posture.POS_Y() - Spots[1]) < 100 &&\
    abs(Posture.POS_W() - Spots[2]) < 5 )
    {
		if(abs(Posture.POS_X() - Spots[0]) < 10 &&\
		abs(Posture.POS_Y() - Spots[1]) < 10 &&\
		abs(Posture.POS_W() - Spots[2]) < 1 )
		{
			point_time++;
			if(point_time > 150)
			{
				point_time = 0;
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if(abs(Posture.POS_X() - Posture.Last_POS[0]) < 1 &&\
		abs(Posture.POS_Y() - Posture.Last_POS[1]) < 1 &&\
		abs(Posture.POS_W() - Posture.Last_POS[2]) < 1 )
		{
			point_time++;
			if(point_time > 150)
			{
				point_time = 0;
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else 
		{
			point_time=0;
			return 0;
		}
    }
	else 
	{
		point_time=0;
		return 0;
	}
}

float L_L,R_L;
float LR_time = 0;
void Auto_classdef::Left_PickIdea(void)
{
	//�㡪��ǽ����
	//��ǽ
	//���ˡ���������
	//ͣ����ȡ��
	
	//����
	if(startFlag == 1 && overFlag == 0)
	{
		Chassis.NO_PostureMode = 3;
		if(Chassis.EdgeDete[4] && Chassis.EdgeDete[5])
		{
			Vx = -770;Vw = 0;Vy=0;
		}
		else
		{
			if(Chassis.EdgeDete[4] == GPIO_PIN_RESET &&\
			Chassis.EdgeDete[5] == GPIO_PIN_RESET && WallFlag == 0)
			{
				if(Vx<-10){Vx += 7;}
				else if(Vx>10){Vx -= 7;}
				else{WallFlag = 1;}
			}
			else if(Chassis.EdgeDete[4] == GPIO_PIN_SET)//����
			{
				Vx = -330;Vy = 0;Vw = -0;
			}
			else if(Chassis.EdgeDete[5] == GPIO_PIN_SET)//����
			{
				Vx = -330;Vy = 0;Vw = 0;
			}
			else if(WallFlag)
			{
				
				if(Analog.LaserRanging[9]<35000 && Analog.LaserRanging[9]>30000 &&\
				Analog.LaserRanging[8]<35000 && Analog.LaserRanging[8]>30000)
				{
					Chassis.Laser_PID[0].Target = 33455+33405;
           			Chassis.Laser_PID[0].Current = Analog.LaserRanging[9]+Analog.LaserRanging[8];
					Vy = Chassis.Laser_PID[0].Cal();
					Vx = -66;
					Chassis.try_bl = 1.5;
					// Vy = 0;
					// L_L = Analog.LaserRanging[9];
					// R_L = Analog.LaserRanging[8];
				}
				else
				{
					Vx = -66;Vy = -880;Vw=0;
					Chassis.try_bl = 1.5;
				}
				
				if(abs(33505+33505-(Analog.LaserRanging[9]+Analog.LaserRanging[8]))<120)
				{
					LR_time++;
				}
				else
				{
					LR_time=0;
				}
				// if(Vy == 0)
				if(LR_time > 15)
				{Chassis.try_bl=0;overFlag = 1;startFlag = 0;Vx=Vy=Vw=0;WallFlag=WallTime=0;HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);}
			}
		}
	}
	else if(startFlag == 0 && overFlag == 1)
	{
		overFlag = 0;
	}
}
