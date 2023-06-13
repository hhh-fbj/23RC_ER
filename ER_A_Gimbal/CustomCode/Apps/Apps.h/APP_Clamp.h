#ifndef __APP_CLAMP_H
#define __APP_CLAMP_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "DEV_MX106.h"

enum Clamp_CtrlMode_e
{
	Clamp_DisableMode = 0,	// 失能
	Clamp_TransiMode,
	Clamp_DebugMode,	// �??通运�??模式
	Clamp_AutoMode,
	Clamp_LockMode,
};


class Clamp_classdef
{
private:
	Clamp_CtrlMode_e Clamp_Mode;
	Clamp_CtrlMode_e Clamp_Next_Mode;
	Clamp_CtrlMode_e Clamp_Last_Mode;

	GPIO_PinState last_A1 = GPIO_PIN_SET;
	GPIO_PinState last_A0 = GPIO_PIN_SET;

	uint8_t Top_Lift_Flag;
	uint8_t Top_PickPlace_Flag;

	int servo_time=0;

	float AddVar[3];
	float UseTarget[3];
	float ServoPos_Target;
	float Top_Lift = 0;
	float Top_PickPlace = 0;
	float now_place;


	struct{
		float Stretch_Max;//伸出：软件限制最远位置—放环点
		float Stretch_Min;//伸出：软件限制最近位置—取环点
		float Stretch_Speed;//伸出：每次控制位置环目标的运动距离
		float Stretch_Speed_Ramp;//伸出：每次控制位置环目标的运动距离的叠加距离
		float Lift_Max;//抬伸：软件限制最大范围
		float Lift_Hold;//抬伸：取环前的抬伸位置
		float Lift_Max_Speed;//抬伸：每次控制位置环目标的最大运动距离
		float Lift_Min_Speed;//抬伸：每次控制位置环目标的最小运动距离
		float Lift_addSpeed_Ramp;//抬伸：每次控制位置环目标的运动距离的叠加距离
		float Lift_subSpeed_Ramp;//抬伸：每次控制位置环目标的运动距离的负叠加距离
		float Lift_Sub_SPortion;//抬伸：每次控制位置环目标上升的运动距离的减速段大小
		float Lift_Sub_XPortion;//抬伸：每次控制位置环目标下降的运动距离的减速段大小
		float Lift_PickWaitTime;//抬伸：取环抬伸到底时等待的时间
		float PickPlace_Max;//放环：软件限制最大范围
		float PickPlace_Release;//放环：取环时与最高点的距离
		float PickPlace_Loop;//放环：放下一个环的距离
		float PickPlace_Speed;//放环：每次控制位置环目标的运动距离
		float Shoot_WaitTime;//发射：发射时等待环掉落后稳定需要的时间
		
		float PickPlace_Ready;//放环：最高点与十个环的距离
		float PickPlace_D;
		float PickPlace_A1;

		float Servo_CtrlTime;//舵机：控制间隔*2,需为2的倍数
		float Servo_TorqueCtrl;//舵机：使能失能的控制指令
		float Servo_PosCtrl;//舵机：控制位置的控制指令
		float Servo_InitPos;//舵机：初始位置
		float Servo_OverPos;//舵机：发射位置
		float Servo_ErrorPos;//舵机：判断条件的范围
		float Stretch_ErrorPos;//伸出：判断条件的范围
		float PickPlace_ErrorPos;//放环：判断条件的范围
		float Lift_ErrorPos;//抬伸：判断条件的范围
	}Param;

	uint8_t ProblemDetection(void);
	void Servo_Control(void);
	void Tar_Update(void);
	void AngleLimit(void);
	void Motor_PIDCalc(void);

	bool Set_TurnPlacel(int mode,int position);
	bool Lift(float pos,bool datum = false,float*rp=NULL);
	bool Stretch(float pos,bool datum = false,float*rp=NULL);
	bool PickPlace(float pos,bool datum = false,float*rp=NULL);
	
public:
	Clamp_classdef();
	// 发送电机数�?�?
	Motor_M3508 Motor[4] = {Motor_M3508(1), Motor_M3508(2), Motor_M3508(3), Motor_M3508(4)};

	//伸出电机1+伸出编码�?+PID
	Motor_M3508 Stretch_Motor = Motor_M3508(1);
	Encider_Brt Stretch_Encider = Encider_Brt(5);
	PositionPID Stretch_PID[2];
	//抬升电机2
	Motor_M3508 Lift_Motor = Motor_M3508(2);
	PositionPID Lift_PID[2];
	// 取放电机3
	Motor_M2006 PickPlace_Motor = Motor_M2006(3);
	PositionPID PickPlace_PID[2];

	MX106_classdef TurnPlace_Servo = MX106_classdef(1);
	
	uint8_t step;
	uint8_t MX106_Data[MXDATASIZE];
	
	uint8_t Init_Flag;
	uint8_t Pick_Flag;
	uint8_t Place_Point_Flag;
	uint8_t Place_Flag;
	
	uint8_t Homeing_Flag;
	uint8_t Ctrl_Flag;
	uint8_t Auto_Aim_Flag=0;
	
	uint8_t PickPlace_Num = 0;
	
	GPIO_PinState A0;
	GPIO_PinState A1;

	void Control();
	void Init(void);
	void Pick(void);
	void Place_Point(void);
	void Place_Point_Cs(void);
	void Place_Auto(void);
	void Place_Manual(void);
	void Place_NoShoot(void);
	void Place_Cs(void);
	void setMode(Clamp_CtrlMode_e mode);
};
#endif
