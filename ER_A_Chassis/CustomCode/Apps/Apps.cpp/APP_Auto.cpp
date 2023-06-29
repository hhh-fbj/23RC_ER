#include "APP_Auto.h"

#include "System_DataPool.h"


void Auto_classdef::Process(void)
{
//	if(DevicesMonitor.Get_State(CHAS_POSTURE_MONITOR) == Off_line ||\
//	DevicesMonitor.Get_State(CHAS_ANALOG_MONITOR) == Off_line ||\
//	Posture.error)
	if(DevicesMonitor.Get_State(CHAS_POSTURE_MONITOR) == Off_line || Posture.error)
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
        break;
    
        default:
        break;
    }
}
int ttttt[2];
void Auto_classdef::Game_Step(void)
{
	switch(text_step)
	{
		//ȡ��ǰ��
		case 0:
			if(startFlag == 1 && overFlag == 0)
			{
				Chassis.NO_PostureMode = 55;
				Chassis.POS_PID[Posture_X][PID_Outer].Target = Spots[0][0];
				Chassis.POS_PID[Posture_Y][PID_Outer].Target = Spots[0][1];
				Chassis.POS_PID[Posture_W][PID_Outer].Target = Spots[0][2];
//				if(Detection_Point_text(Spots[0]) || \
//				(Chassis.EdgeDete[4] == GPIO_PIN_RESET &&\
//				Chassis.EdgeDete[5] == GPIO_PIN_RESET))
				if(Detection_Point(Spots[0]))
				{overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;text_step = 5;}
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
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Ĭ��
		break;

		//����ȡ��
		case 1:
			Left_PickIdea();
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Ĭ��
		break;
		
		//取环
		case 2:
		if(ttttt[0]==0)
		{
			ttttt[0] = Get_SystemTimer();
		}
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
//			if(IO_RTX(20,10))
//			{
//				startFlag = 1;
//				text_step = 3;
//			}
			startFlag = 1;
			text_step = 3;
		break;
		
		//����ǰ
		case 3:
			if(ttttt[1]==0)
			{
				ttttt[1] = Get_SystemTimer();
			}
			if(startFlag == 1 && overFlag == 0)
			{
//				Chassis.NO_PostureMode = 0;
				Chassis.NO_PostureMode = 55;
				Chassis.POS_PID[Posture_X][PID_Outer].Target = Spots[1][0];
				Chassis.POS_PID[Posture_Y][PID_Outer].Target = Spots[1][1];
				Chassis.POS_PID[Posture_W][PID_Outer].Target = Spots[1][2];
				if(Detection_Point(Spots[1]))
				{overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;text_step = 4;}
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
		
		//钱顶
		case 4:
			Front_PlaceIdea();
		break;
		
		//发射
		case 5://第1发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
//			if(IO_RTX(20,30))
//			{
//				startFlag = 1;
//				text_step = 6;
//			}
		break;

		case 6://第2发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 7;
			}
		break;
			
		case 7://第3发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 8;
			}
		break;
			
		case 8://第4发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 9;
			}
		break;

		case 9://第5发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 10;
			}
		break;

		case 10://第6发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 11;
			}
		break;
			
		case 11://第7发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 12;
			}
		break;
			
		case 12://第8发
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 13;
			}
		break;
			
		
	}
}

bool zhiling=true;
int ssssp;
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
				//				if(Detection_Point(Spots[0]))
//				if(Detection_Point_text(Spots[0]) || \
//				(Chassis.EdgeDete[4] == GPIO_PIN_RESET &&\
//				Chassis.EdgeDete[5] == GPIO_PIN_RESET))
//				{overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;text_step = 1;}
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
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Ĭ��
		break;

		//����ȡ��
		case 1:
			Chassis.NO_PostureMode = 2;
//			if(startFlag == 1 && overFlag == 0)
//			{
//				Chassis.NO_PostureMode = 55;
//				Chassis.POS_PID[Posture_X][PID_Outer].Target = Spots[1][0];
//				Chassis.POS_PID[Posture_Y][PID_Outer].Target = Spots[1][1];
//				Chassis.POS_PID[Posture_W][PID_Outer].Target = Spots[1][2];
//				if(Detection_Point(Spots[1]))
//				{overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;text_step = 2;}
//			}
//			else if(startFlag == 0 &&overFlag == 1)
//			{
//				overFlag = 0;
//			}
//			else if(startFlag == 0 && overFlag == 0)
//			{
//				Chassis.NO_PostureMode = 0;
//				Chassis.POS_PID[Posture_X][PID_Outer].Target = Posture.POS_X();
//				Chassis.POS_PID[Posture_Y][PID_Outer].Target = Posture.POS_Y();
//				Chassis.POS_PID[Posture_W][PID_Outer].Target = Posture.POS_W();
//			}
//			Left_PickIdea();
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Ĭ��
		break;
		
		case 2:
			startFlag = 0;
			Chassis.NO_PostureMode = 2;
//			if(IO_RTX(20,10))
//			{
//				startFlag = 1;
//				text_step = 3;
//			}
		break;
		
		case 3:
			if(startFlag == 1 && overFlag == 0)
			{
				Chassis.NO_PostureMode = 55;
				Chassis.POS_PID[Posture_X][PID_Outer].Target = 370;
				Chassis.POS_PID[Posture_Y][PID_Outer].Target = 980;
				Chassis.POS_PID[Posture_W][PID_Outer].Target = 0;
				if(abs(Posture.POS_Y() - 980) < 200 &&\
					abs(Posture.POS_X() - 370) < 500)
				{overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;text_step = 4;}
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
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Ĭ��
		break;

		case 4:
			if(startFlag == 1 && overFlag == 0)
			{
				//
				Chassis.NO_PostureMode = 3;
				
				if(Posture.POS_X() - 370 > 20)
				{
					Vx = -1200;Vw = 0;Vy=0;
				}
				else if(Posture.POS_X() - 370 < -20)
				{
					Vx = 1200;Vw = 0;Vy=0;
				}
				else
				{
					Vx = 0;Vw = 0;Vy= 800;
				}
				if(Chassis.EdgeDete[2] == GPIO_PIN_RESET &&\
					Chassis.EdgeDete[3] == GPIO_PIN_RESET)
				{
					Vy = 1600;Vx = 0;Vw = 0;
				}
				
			}
			else if(startFlag == 0 && overFlag == 1)
			{
				overFlag = 0;
			}
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Ĭ��
		break;
		
		//
		case 5:
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 6;
			}
		break;
		
		case 6://第2发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 7;
			}
		break;
			
		case 7://第3发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 8;
			}
		break;
			
		case 8://第4发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 9;
			}
		break;

		case 9://第5发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 10;
			}
		break;

		case 10://第6发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 11;
			}
		break;
			
		case 11://第7发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 12;
			}
		break;
			
		case 12://第8发
			startFlag = 0;
			Chassis.NO_PostureMode = 3;
			Vy = 1600;Vx = 0;Vw = 0;
			if(IO_RTX(20,30))
			{
				startFlag = 1;
				text_step = 13;
			}
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


uint8_t Auto_classdef::Detection_Point(int *spots)
{
	if(abs(Posture.POS_X() - spots[0]) < 20 &&\
	abs(Posture.POS_Y() - spots[1]) < 20 &&\
	abs(Posture.POS_W() - spots[2]) < 2 )
	{
			if(abs(Posture.POS_X() - spots[0]) < 20 &&\
			abs(Posture.POS_Y() - spots[1]) < 20 &&\
			abs(Posture.POS_W() - spots[2]) < 2 )
			{
				point_time++;
				if(point_time > 300)
				{
					point_time = 0;
					return 1;
				}
				else
				{
					return 0;
				}
			}
			else if(abs(Posture.POS_X() - Posture.Last_POS[0]) < 10 &&\
			abs(Posture.POS_Y() - Posture.Last_POS[1]) < 10 &&\
			abs(Posture.POS_W() - Posture.Last_POS[2]) < 1 )
			{
				point_time++;
				if(point_time > 80)
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




void Auto_classdef::Left_PickIdea(void)
{
	//点——墙附件
	//贴墙
	//后退——激光测距->微动
	//停——取环
	
	//左走
	if(startFlag == 1 && overFlag == 0)
	{
		CH_F_X = 1;CH_HL_X = 1;CH_HR_X = 1;
		Chassis.NO_PostureMode = 3;
		if(Chassis.EdgeDete[4] && Chassis.EdgeDete[5])
		{
			Vx = -1200;Vw = 0;Vy=0;
		}
		else
		{
			if(WallFlag)
			{
				if(Chassis.EdgeDete[7] == GPIO_PIN_RESET)// && Chassis.EdgeDete[7] == GPIO_PIN_RESET)
				{
						Vx = -100;
						Vw = 0;
						Vy = -1100;
						LR_out_time++;
				}
				else
				{
					Vx = -100;Vy = -1100;Vw=0;
				}
				
				// if(Vy == 0)
//				if(LR_time > 5|| (LR_out_time > 800 && abs(33505+33505-(Analog.LaserRanging[9]+Analog.LaserRanging[8]))<500))
				if(LR_out_time > 5)
				{LR_out_time=0;overFlag = 0;startFlag = 1;\
				Vx=Vy=Vw=0;WallFlag=WallTime=0;\
				text_step = 2;}
			}
			else if(Chassis.EdgeDete[4] == GPIO_PIN_RESET &&\
			Chassis.EdgeDete[5] == GPIO_PIN_RESET && WallFlag == 0)
			{
					CH_F_X = 1;CH_HL_X = 1;CH_HR_X = 1;
				if(Vx<-100){Vx += 10;}
				else if(Vx>100){Vx -= 10;}
				else{LR_out_time=0;overFlag = 0;startFlag = 1;\
				Vx=Vy=Vw=0;WallFlag=0;\
				text_step = 2;}//WallFlag = 1;
			}
			else if(Chassis.EdgeDete[4] == GPIO_PIN_SET)//����
			{
				Vx = -600;Vy = 0;Vw = 0;
				CH_F_X = 2;CH_HL_X = 1;CH_HR_X = 1;
			}
			else if(Chassis.EdgeDete[5] == GPIO_PIN_SET)//����
			{
				Vx = -600;Vy = 0;Vw = 0;
				CH_F_X = 1;CH_HL_X = 2;CH_HR_X = 2;
			}

		}
	}
	else if(startFlag == 0 && overFlag == 1)
	{
		overFlag = 0;
	}
}

void Auto_classdef::Front_PlaceIdea(void)
{
	//点——墙附件
	//贴墙
	//交给云台
	
	//前走
	if(startFlag == 1 && overFlag == 0)
	{
		Chassis.NO_PostureMode = 3;
		if(Chassis.EdgeDete[2] && Chassis.EdgeDete[3])
		{
			Vx = 0;Vw = 0;Vy=550;
		}
		else
		{
			if(Chassis.EdgeDete[2] == GPIO_PIN_RESET &&\
			Chassis.EdgeDete[3] == GPIO_PIN_RESET && WallFlag == 0)
			{
				if(Vy<-10){Vy += 1;}
				else if(Vy>10){Vy -= 1;}
				else{WallFlag = 1;}
			}
			else if(Chassis.EdgeDete[2] == GPIO_PIN_SET)//����
			{
				Vx = 0;Vy = 0;Vw = -220;
			}
			else if(Chassis.EdgeDete[3] == GPIO_PIN_SET)//����
			{
				Vx = 0;Vy = 0;Vw = 220;
			}
			else if(WallFlag)
			{
				
				Vx = 0;Vy = 330;Vw=0;

				WallTime++;
				if(WallTime > 150)
				{overFlag = 0;startFlag = 1;Vx=Vy=Vw=0;\
				text_step = 5;WallFlag=WallTime=0;}
			}
		}
	}
	else if(startFlag == 0 && overFlag == 1)
	{
		overFlag = 0;
	}
}

int IO_S_time;
int IO_R_time;
bool Auto_classdef::IO_RTX(int io_s,int is_t)
{
	
	if(IO_S_time >= io_s)
	{
		IO_R_time++;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//��ת
		if(IO_R_time >= is_t)
		{
			IO_R_time = 0;
			IO_S_time = 0;
			return 1;
		}
	}
	else
	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//��ת
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);//��ת
	}
	
	if(Chassis.Ready_Flag == GPIO_PIN_SET && Chassis.Last_Ready_Flag == GPIO_PIN_RESET && IO_S_time==0)
	{
		IO_S_time++;
	}
	if(Chassis.Ready_Flag == GPIO_PIN_SET && IO_S_time>0)
	{
		IO_S_time++;
	}
	else
	{
		IO_S_time = 0;
	}
	
	
	
	return 0;
	
}


