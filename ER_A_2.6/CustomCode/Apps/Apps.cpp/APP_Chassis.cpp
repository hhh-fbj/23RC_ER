
/**
 ------------------------------------------------------------------------------
 * @file    APP_Chassis.cpp
 * @author  Shake
 * @brief   底盘控制
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "APP_Chassis.h"
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "arm_math.h"

#include "ALGO_Kalman.h"

#include "BSP_CAN.h"

#include "DEV_AIMU.h"

#include "System_DataPool.h"
#include "Wolf_Infantry.h"

#include "APP_Auto.h"

/* Private define ------------------------------------------------------------*/
#define USE_RUDDER_RESET 1
#define REDUCTION_RATIOE 3//1:3
#define VALUE_CIRCLE 98304
#define DEGREE_TURN_CIRCLE 273.06666666666666666666666666667

#define CHASSIS_MAX_SPEED 4000  // 底盘驱动轮最大速度
#define CHASSIS_MAX_VW    0.8*CHASSIS_MAX_SPEED  // 底盘旋转最大速度

float Radius = 1.0f;  // 圆心距
extKalman_t Kalman_CHASFollow_Speed;

/*------------------------------------------------------------ 初始化 ------------------------------------------------------------*/
Chassis_classdef::Chassis_classdef()
{
	//转向轮 2006
	/*--- 外环 位置 PID -------------------------------------------------------------------------*/
    RUD_PID[RF_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
    RUD_PID[LF_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
    RUD_PID[LB_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
    RUD_PID[RB_Rud][PID_Outer].SetPIDParam(2.0, 0.0, 0.0, 5000, 16000, 0.002f);
//    /*--- 内环 速度 PID -------------------------------------------------------------------------*/
    RUD_PID[RF_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[RF_Rud][PID_Inner].I_SeparThresh = 800;
    RUD_PID[LF_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[RF_Rud][PID_Inner].I_SeparThresh = 800;
    RUD_PID[LB_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[RF_Rud][PID_Inner].I_SeparThresh = 800;
    RUD_PID[RB_Rud][PID_Inner].SetPIDParam(5.0f, 3.0, 0.0f, 5000, 16000, 0.002f);
    RUD_PID[RF_Rud][PID_Inner].I_SeparThresh = 800;
	
	//全场定位
	/*--- 外环 位置 PID -------------------------------------------------------------------------*/
    POS_PID[Posture_X][PID_Outer].SetPIDParam(2.5f, 0.0f, 0.0f, 4000, 15000, 0.002f);POS_PID[Posture_X][PID_Outer].DeadZone = 1;//4 11 0.0003
    POS_PID[Posture_X][PID_Outer].a_p = 4;POS_PID[Posture_X][PID_Outer].b_p = 11;POS_PID[Posture_X][PID_Outer].c_p = 0.0002;
    POS_PID[Posture_Y][PID_Outer].SetPIDParam(2.5f, 0.0f, 0.0f, 4000, 15000, 0.002f);POS_PID[Posture_Y][PID_Outer].DeadZone = 1;//3.5		6 0.003	
    POS_PID[Posture_Y][PID_Outer].a_p = 3.5;POS_PID[Posture_Y][PID_Outer].b_p = 6;POS_PID[Posture_Y][PID_Outer].c_p = 0.0003;
    POS_PID[Posture_Z][PID_Outer].SetPIDParam(90.0f, 0.0f, 0.0f, 2000, 10000, 0.002f);POS_PID[Posture_Z][PID_Outer].DeadZone = 0.5;
    /*--- 内环 速度 PID 没用 -------------------------------------------------------------------------*/
    POS_PID[Posture_X][PID_Inner].SetPIDParam(0.0f, 0.0f, 0.0f, 1000, 10000, 0.002f);
    POS_PID[Posture_Y][PID_Inner].SetPIDParam(0.0f, 0.0f, 0.0f, 1000, 10000, 0.002f);
    POS_PID[Posture_Z][PID_Inner].SetPIDParam(0.0f, 0.0f, 0.0f, 1000, 10000, 0.002f);

    //修复环
    /*--- 外环 位置 PID -------------------------------------------------------------------------*/
    Repair_PID[PID_Outer].SetPIDParam(360.0f, 0.0f, 0.0f, 4000, 10000, 0.002f);Repair_PID[PID_Outer].DeadZone = 1;
    /*--- 内环 速度 PID 没用 -------------------------------------------------------------------------*/
    Repair_PID[PID_Inner].SetPIDParam(0.0, 0.0f, 0.0f, 1000, 10000, 0.002f);Repair_PID[PID_Inner].DeadZone = 1;

    //激光双环——前进环/旋转环
    Laser_PID[0].SetPIDParam(0.4f, 0.01f, 0.0f, 200, 440, 0.002f);Laser_PID[0].DeadZone = 10;
    Laser_PID[1].SetPIDParam(0.5f, 0.0f, 0.0f, 4000, 10000, 0.002f);Laser_PID[1].DeadZone = 30;

    for(int i=0;i<4;i++)
    {
        FRONT[i] = 0;//为0时，轮子朝前
        XYZ_PreTar_Angle[i] = 90;//默认指向为XYZ_PreTar_Angle设定值
    }

    Pos_Target[0] = 0;
    Pos_Target[1] = 0;
    Pos_Target[2] = 0;
}


/*------------------------------------------------------------ 控制 ------------------------------------------------------------*/
//底盘总控制函数
int clamp_time = 0;
int clamp_init = 0;
float cssss = 2000;
void Chassis_classdef::Control()
{
    //微动开关检测
    Sensor();
	
    //问题检测
    if(ProblemDetection()){return;}

    //底盘更新数据
    ChassisTar_Update();
		
    Send_Data();
}

void Chassis_classdef::Sensor(void)
{
    //右
    EdgeDete[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);//右
    EdgeDete[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);//左
    //前
    EdgeDete[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);//右
    EdgeDete[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);//左
    //左
    EdgeDete[4] = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_5);//右
    EdgeDete[5] = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);//左	

    Ready_Flag = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_2);
}

uint8_t Chassis_classdef::ProblemDetection(void)
{
    if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line &&  Buzzer.error == 1)
    {
        for(uint8_t i = 0 ; i < 4 ; i++)
        {
            RUD_Motor[i].Out = 0;
            RUD_PID[i][PID_Outer].Reset();
            RUD_PID[i][PID_Inner].Reset();
            Cal_Speed[i] = 0;
            Buzzer.error = 1;
        }
        Send_Data();
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
        return 1;
    }
    else if(DevicesMonitor.Get_State(CHAS_RUD1_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUD2_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUD3_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUD4_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider1_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider2_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider3_MONITOR) == Off_line || \
        DevicesMonitor.Get_State(CHAS_RUDEncider4_MONITOR) == Off_line)
    {
        for(uint8_t i = 0 ; i < 4 ; i++)
        {
            RUD_Motor[i].	Out = 0;
            RUD_PID[i][PID_Outer].Reset();
            RUD_PID[i][PID_Inner].Reset();
            Cal_Speed[i] = 0;
            Buzzer.error = 1;
        }
        Send_Data();
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
        return 1;
    }
    else
    {
        return 0;
    }
}

void Chassis_classdef::ChassisTar_Update()
{
    switch ((int)Mode)
    {
        case CHAS_TransiMode:
            POS_PID[Posture_X][PID_Outer].Target = Auto.Posture.POS_X();//Auto.Posture.POS_W();
            POS_PID[Posture_Y][PID_Outer].Target = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
            POS_PID[Posture_W][PID_Outer].Target = Auto.Posture.POS_W();//Auto.Posture.POS_W();
            Auto.Vx = Auto.Vy = Auto.Vw = 0;
            Mode = Next_Mode;
            Last_Mode = CHAS_TransiMode;
        break;

        case CHAS_ControlMode://视觉控制底盘
            Process(0, 0, 0);
            Last_Mode = CHAS_ControlMode;
        break;
    
        case CHAS_MoveMode:
            Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
            Last_Mode = CHAS_MoveMode;
        break;

        case CHAS_LockMode:
            Process(0, 0, 0);
            Last_Mode = CHAS_LockMode;
        break;

        case CHAS_AutoMode://跑自动
            Auto.Process();
            switch(NO_PostureMode)
            {
                case 0:
                    POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
                    POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
                    POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();

                    Process(POS_PID[Posture_X][PID_Outer].Cal(), POS_PID[Posture_Y][PID_Outer].Cal(), POS_PID[Posture_W][PID_Outer].Cal());
                break;
                
                case 1:
                    Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
                break;
                
                case 2:
                    Process(0, 0, 0);
                break;
                    
                case 3:
                    Process(Auto.Vx, Auto.Vy, Auto.Vw);
                break;

                case 4:
                    Process(Auto.Vx, Auto.Vy, Auto.Vw);
                break;

                case 66:
                    POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
                    POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
                    POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();

                    Process(POS_PID[Posture_X][PID_Outer].Cal()-770, POS_PID[Posture_Y][PID_Outer].Cal(), POS_PID[Posture_W][PID_Outer].Cal());
                break;
                
                case 99:
                    Process(0, 0, 0);
                break;
                
            }
            Last_Mode = CHAS_AutoMode;
        break;

        case CHAS_IMUMode://使用C陀螺仪/全场定位陀螺仪进行修正
            if(Last_Mode != CHAS_IMUMode){Repair_PID[PID_Outer].Target = RecvCan_Msg.Pack.Yaw_Z;}//Auto.Posture.POS_W();}
            if((CTRL_DR16.Get_ExptVx() != 0 || CTRL_DR16.Get_ExptVy() != 0) && CTRL_DR16.Get_ExptVw() == 0)
            {
                Repair_PID[PID_Outer].Current = RecvCan_Msg.Pack.Yaw_Z;//Auto.Posture.POS_W();
                Repair_PID[PID_Inner].Target = Repair_PID[PID_Outer].Cal();
                Repair_PID[PID_Inner].Current = RecvCan_Msg.Pack.Gz;
                Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), -Repair_PID[PID_Inner].Cal());//*(sqrt(pow(CTRL_DR16.Get_ExptVx(),2)+pow(CTRL_DR16.Get_ExptVy(),2))/16970));
            }
            else
            {
                Repair_PID[PID_Outer].Target = RecvCan_Msg.Pack.Yaw_Z;//.POS_W();
                Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
            }
            Last_Mode = CHAS_IMUMode;
        break;

        case CHAS_LaserMode://使用激光测距进行修正
            Laser_PID[0].Target = 32768+32665;
            Laser_PID[0].Current = Auto.Analog.LaserRanging[9]+Auto.Analog.LaserRanging[8];
            Laser_PID[1].Target = 32665-Auto.Analog.LaserRanging[8];
            Laser_PID[1].Current = 32768-Auto.Analog.LaserRanging[9];
            Process(CTRL_DR16.Get_ExptVx(), Laser_PID[0].Cal(), Laser_PID[1].Cal());
            Last_Mode = CHAS_LaserMode;
        break;

        case CHAS_PostureMode://
            Pos_Target[0] += CTRL_DR16.Get_ExptVx()*0.0001;
            Pos_Target[1] -= CTRL_DR16.Get_ExptVy()*0.0001;
            Pos_Target[2] -= CTRL_DR16.Get_ExptVw()*0.00001;
            POS_PID[Posture_X][PID_Outer].Target = Pos_Target[0];//Auto.Posture.POS_W();
            POS_PID[Posture_Y][PID_Outer].Target = Pos_Target[1];//Auto.Posture.POS_W();
            POS_PID[Posture_W][PID_Outer].Target = Pos_Target[2];

            POS_PID[Posture_X][PID_Outer].Current = Auto.Posture.POS_X();//Auto.Posture.POS_W();
            POS_PID[Posture_Y][PID_Outer].Current = Auto.Posture.POS_Y();//Auto.Posture.POS_W();
            POS_PID[Posture_W][PID_Outer].Current = Auto.Posture.POS_W();//Auto.Posture.POS_W();

            Process(POS_PID[Posture_X][PID_Outer].Cal(), -POS_PID[Posture_Y][PID_Outer].Cal(), -POS_PID[Posture_W][PID_Outer].Cal());

            Last_Mode = CHAS_PostureMode;
        break;

        case CHAS_DisableMode:
            for(uint8_t j = 0 ; j < 4 ; j++)
            {
                RUD_PID[j][PID_Outer].Reset();
                RUD_PID[j][PID_Inner].Reset();
                Cal_Speed[j] = 0;

                RUD_Motor[j].Out = 0;  
                Cal_Speed[j] = 0;
            }
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
            Last_Mode = CHAS_DisableMode;
        break;
    }
}

//底盘数据处理
float Ramp_Vy, Ramp_Vx, Ramp_Vw;
float ACCCCC_VAL = 20.0f, DECCCCC_VAL = 40.0f;
void Chassis_classdef::Process(float Vx, float Vy, float Vw)
{		
    //--- 速度斜坡	作用?
    Drv_Slow(&Ramp_Vx, Vx, 10.0f, ACCCCC_VAL, DECCCCC_VAL);//
    Drv_Slow(&Ramp_Vy, Vy, 10.0f, ACCCCC_VAL, DECCCCC_VAL);
    Drv_Slow(&Ramp_Vw, Vw*0.5, 6.0f, ACCCCC_VAL*0.6, DECCCCC_VAL*0.6);
//    Ramp_Vx=Vx;Ramp_Vy=Vy;Ramp_Vw=Vw;

    //运动解算
    Rudder_Solve(Ramp_Vx, Ramp_Vy, Ramp_Vw, Cal_Speed);
    
    //PID计算
    for(uint8_t i = 0; i < 4; i++)
    {
        RUD_PIDCalc(i);  
        Three_Speed[i] = Cal_Speed[i] * falsh[i];
    }

    SS1.speed[0] = Three_Speed[0];
    SS1.speed[1] = Three_Speed[2];

    SS2.speed[0] = Three_Speed[1];
    SS2.speed[1] = Three_Speed[3];
}

void Chassis_classdef::Send_Data()
{
    MotorMsgSend(&hcan1, RUD_Motor);
    MotorMsgSend(&hcan2, RUD_Motor);
}

/**
 * @brief      底盘速度斜坡
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_classdef::Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal)
{
    if(abs(*rec) - abs(target) < 0)//加速时
    {
        // if(abs(*rec) > 10)
        // {
            slow_Inc = slow_Inc * Accval;//速度提起来的时候增大到5倍
        // }
    }else if(abs(*rec) - abs(target) > 0)
    {
        slow_Inc = slow_Inc * DecVal;//减速时放大15倍
    }

    if(abs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }
}

static float const Ltheta = acos(-0.5),Rtheta = acos(0.5);//三轮后
static float const Ftheta = atan(1.0);//四轮
//轮舵轮运动解算
void Chassis_classdef::Rudder_Solve(float Vx, float Vy, float Vw, float *cal_speed)
{
    float Param = 1.0f;
    float MaxSpeed = 0.0f;

	//速度限制
    Constrain(&Vx, (float)(-CHASSIS_MAX_SPEED), (float)(CHASSIS_MAX_SPEED));
    Constrain(&Vy, (float)(-CHASSIS_MAX_SPEED), (float)(CHASSIS_MAX_SPEED));
    Constrain(&Vw, (float)(-CHASSIS_MAX_VW), (float)(CHASSIS_MAX_VW));

    RudAngle_Calc(Vx, Vy, Vw);
    
    /* 驱动轮 速度解算 ---------------------------------------------------------------------------------*/
    cal_speed[RF_Rud] =  sqrt(pow(Vx + Vw*arm_cos_f32(Ftheta),2) + pow(Vy - Vw*arm_sin_f32(Ftheta),2));
    cal_speed[LF_Rud] =  sqrt(pow(Vx + Vw*arm_cos_f32(Ftheta),2) + pow(Vy + Vw*arm_sin_f32(Ftheta),2));
    cal_speed[LB_Rud] =  sqrt(pow(Vx - Vw*arm_cos_f32(Ftheta),2) + pow(Vy + Vw*arm_sin_f32(Ftheta),2));
    cal_speed[RB_Rud] =  sqrt(pow(Vx - Vw*arm_cos_f32(Ftheta),2) + pow(Vy - Vw*arm_sin_f32(Ftheta),2));

    // 寻找最大速度
    for (uint8_t i = 0; i < 4; i++)
    {
        //RUD_PID[i][PID_Outer].Current = Vw; 
        if(abs(cal_speed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(cal_speed[i]);
        }
    }

    // 速度分配  
    if (MaxSpeed > CHASSIS_MAX_SPEED)
    {
        Param = (float)CHASSIS_MAX_SPEED / MaxSpeed;
    }

    if(abs(RUD_PID[RF_Rud][PID_Outer].Target) <= 1500 &&
        abs(RUD_PID[LF_Rud][PID_Outer].Target) <= 1500 &&
        abs(RUD_PID[LB_Rud][PID_Outer].Target) <= 1500 &&
        abs(RUD_PID[RB_Rud][PID_Outer].Target) <= 1500)
    {
        cal_speed[RF_Rud] *= Param;
        cal_speed[LF_Rud] *= Param;
        cal_speed[LB_Rud] *= Param;
        cal_speed[RB_Rud] *= Param;
    }
}

/**
 * @brief      底盘转电机位置式式PID计算
 * @param[in]  motor
 * @param[in]  target
 * @param[in]  current
 * @note       None
 * @retval     None
 */
void Chassis_classdef::RUD_PIDCalc(uint8_t motor)
{
	RUD_PID[motor][PID_Inner].Target = RUD_PID[motor][PID_Outer].Cal();
	RUD_PID[motor][PID_Inner].Current = RUD_Motor[motor].getSpeed();
	RUD_Motor[motor].Out = RUD_PID[motor][PID_Inner].Cal();
}

//三轴速度解算四轮舵轮 舵角度 (直角坐标系)
void Chassis_classdef::RudAngle_Calc(float Vx, float Vy, float Vw)
{
    //--- TODO 因为陀螺仪零漂会产生小幅度的Vw速度导致这里的InitAngle会来回跳变(暂时用个低级的方法解决)√

    //--- TODO 还有就是45度刹车可能由于车的重心较高，高速运动的时候会刹的比较猛会出现前倾
    //---      一个思路是不同的功率给不同的斜坡
    //---      另一个是刹车的时候先保持原本的角度，先让前轮失能，等车稳定后再呈45度归中
    //---      再一个是用高级点的加速曲线？

    if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        if(Mode == CHAS_LockMode || NO_PostureMode == 2)//轮子45度X型朝向
        {
            XYZ_Angle[0] = 45;
            XYZ_Angle[1] = 135;
            XYZ_Angle[2] = 45;
            XYZ_Angle[3] = 135;
        }
        else if(0)//轮子45度◇型朝向
        {
            // XYZ_Angle[0] = 135;
            // XYZ_Angle[1] = 45;
            // XYZ_Angle[2] = 135;
            // XYZ_Angle[3] = 45;
        }
        else
        {
            //--- 目标角度为最后速度指向;
            for(uint8_t i = 0 ; i < 4 ; i++)
            {
                XYZ_Angle[i] = XYZ_PreTar_Angle[i];
            }
        }
    }
    else
    {
        //--- 有目标速度的时候才进行舵轮解算的计算
        XYZ_Angle[RF_Rud] = atan2(Vy - Vw*(Radius*arm_sin_f32(Ftheta)), Vx + Vw*(Radius*arm_cos_f32(Ftheta)))*(180/PI);
        XYZ_Angle[LF_Rud] = atan2(Vy + Vw*(Radius*arm_sin_f32(Ftheta)), Vx + Vw*(Radius*arm_cos_f32(Ftheta)))*(180/PI);
        XYZ_Angle[LB_Rud] = atan2(Vy + Vw*(Radius*arm_sin_f32(Ftheta)), Vx - Vw*(Radius*arm_cos_f32(Ftheta)))*(180/PI);
        XYZ_Angle[RB_Rud] = atan2(Vy - Vw*(Radius*arm_sin_f32(Ftheta)), Vx - Vw*(Radius*arm_cos_f32(Ftheta)))*(180/PI);
        //--- 无目标速度的时候不使用上一次角度来保存是因为跟随模式下IMU静止的瞬间会产生轻微的Vw速度

        for(uint8_t i = 0 ; i < 4 ; i++)
        {
            XYZ_Angle[i] = XYZ_Angle[i]<0 ? (360+XYZ_Angle[i]) : abs(XYZ_Angle[i]);
        }
    }

    // XYZ_Angle[0] = 135;
    //     XYZ_Angle[1] = 45;
    //     XYZ_Angle[2] = 135;
    //     XYZ_Angle[3] = 45;

    //--- 更改为保存经过劣弧处理的目标角度(现在暂时不用这个上一帧的角度)
    for(uint8_t i = 0 ; i < 4 ; i++) 
    {
        XYZ_PreTar_Angle[i] = XYZ_Angle[i];
    }
    
    //转成对应编码器的度量数值求取误差
    Angle_Treatment();
}

void Chassis_classdef::Angle_Treatment(void)
{
    float Error;
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        //转舵分辨率
        XYZ_Angle[i] *= DEGREE_TURN_CIRCLE;
				
        XYZ_Angle[i] += (FRONT[i]-24576);
        XYZ_Angle[i] = XYZ_Angle[i] - (floor(XYZ_Angle[i]/VALUE_CIRCLE)*VALUE_CIRCLE);

        //劣弧+反转判断
        Error = XYZ_Angle[i] - (RUD_Encider[i].getTotolAngle()%VALUE_CIRCLE);//(RUD_Encider[i].getTotolAngle() - floor(RUD_Encider[i].getTotolAngle()/VALUE_CIRCLE) * VALUE_CIRCLE);
        //
        if (Error > 49152)
        {
            Error -= VALUE_CIRCLE;
        }
        else if (Error < -49152)
        {
            Error += VALUE_CIRCLE;
        }
				//
        if(Error > 24576)
        {
            Error -= 49152;
            falsh[i] = -1;
        }
        else if(Error <= -24576)
        {
            Error += 49152;
            falsh[i] = -1;
        }
        else
        {
            falsh[i] = 1;
        }
        RUD_PID[i][PID_Outer].Target = (int)Error;
    }
}

/*------------------------------------------------------------ 处理相关 ------------------------------------------------------------*/
uint8_t twa_count;
void Chassis_classdef::CAN_Send(void)
{
    if(Mode != CHAS_DisableMode &&  Buzzer.error != 1)
    {
        if(twa_count == 1)
        {
            CANx_SendData(&hcan1, 0x341, SS1.data, 8);
            twa_count = 0;
        }
        else
        {
            CANx_SendData(&hcan1, 0x342, SS2.data, 8);
            twa_count = 1;
        }
        
        
    }
}
void Chassis_classdef::CAN_Recvd(uint8_t can_rx_data[])
{
	for(int i=0;i<8;i++)
	{
		RecvCan_Msg.data[i] = can_rx_data[i];
	}
}
void Chassis_classdef::Set_Mode(CHAS_CtrlMode_e mode)
{   
    if(Mode != mode)
    {
        Next_Mode = mode;
        Mode = CHAS_TransiMode;
    }
}
