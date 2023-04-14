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
	float AddVar[3];
	float UseTarget[3];
	float ServoPos_Target;
	
	struct{
		float Stretch_Hold;
		float Lift_Max;
		float PickPlace_Max;
		float PickPlace_Release;
		float PickPlace_Loop;
		float Stretch_Speed;
		float Lift_Speed;
		float PickPlace_Speed;

		float Stretch_Max;
		float Stretch_Take;
		float Stretch_Shoot;
		float Stretch_Min;
		float PickPlace_Ready;
	}Param;

	uint8_t ProblemDetection(void);
	void Servo_Control(void);
	void Set_TurnPlacel(uint8_t mode,float position);
	void Tar_Update(void);
	void AngleLimit(void);
	void Motor_PIDCalc(void);
public:
	Clamp_classdef();
	// 发送电机数�?�?
	Motor_M3508 Motor[4] = {Motor_M3508(1), Motor_M3508(2), Motor_M3508(3), Motor_M3508(4)};
	

	//伸出电机1+伸出编码�?+PID
	Motor_M3508 Stretch_Motor = Motor_M3508(1);
	Encider_Brt Stretch_Encider = Encider_Brt(5);
	PositionPID Stretch_PID[2];
	//�?升电�?2
	Motor_M3508 Lift_Motor = Motor_M3508(2);
	PositionPID Lift_PID[2];
	// 取放电机3
	Motor_M2006 PickPlace_Motor = Motor_M2006(3);
	PositionPID PickPlace_PID[2];

	MX106_classdef TurnPlace_Servo = MX106_classdef(1);
	
	uint8_t Init_Flag;
	uint8_t Pick_Flag;
	uint8_t Place_Point_Flag;
	uint8_t Place_Flag;
	uint8_t TakeOut_Flag;
	uint8_t Homeing_Flag;
	uint8_t Pos_Flag;
	uint8_t Should_Init_Flag = 1;
	uint8_t Should_Place_Flag;
	uint8_t Auto_Aim_Flag=1;
	
	GPIO_PinState I7;
	GPIO_PinState E14;

	void Control();
	void Init(void);
	void Pick(void);
	void Place_Point(void);
	void Place(void);
	void setMode(Clamp_CtrlMode_e mode);
};
#endif
