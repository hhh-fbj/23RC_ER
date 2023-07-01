#ifndef __APP_SHOOT_H
#define __APP_SHOOT_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "APP_DR16.h"
#include "APP_Clamp.h"
#include "DEV_MX106.h"

enum Shoot_CtrlMode_e
{
	Shoot_DisableMode,	// 失能
	Shoot_ManualMode,
	Shoot_LockMode,
	Shoot_TransiMode,
	Shoot_NewManualMode,
	Shoot_DebugMode,
	Shoot_NewAutoMode,
};

enum Pull_CtrlMode_e
{
	Pull_DisableMode,	// 失能
	Pull_DebugMode,
	Pull_LockMode,
	Pull_TransiMode,
	Pull_NewDebugMode,
	Pull_GearSetMode,
	Pull_AutoMode,
};


class Shoot_classdef
{
private:
	GPIO_PinState A2;
	GPIO_PinState A3;
	GPIO_PinState I5;
	GPIO_PinState last_A3 = GPIO_PIN_SET;
	GPIO_PinState last_I5 = GPIO_PIN_SET;

	bool Launch_Switch = false;

	uint8_t first;
	int servo_time=0;

	float Shoot_TarAngle,AddAngle;
	float Pull_AddAngle[2];
	// float Pre_Tar[2];
	float Shoot_Flag;
	float Pull_Lock_Flag = 1;
	float shoot_time;
	float stop_time;
	float stop_shoot;
	float Top_LeftPull = 0;
	float Top_LeftPull_Flag = 0;
	float Top_RightPull = 0;
	float Top_RightPull_Flag = 0;

	float clamp_pos_R;
	float clamp_pos_L;
	
	float ServoPos_Target;
	
	struct{
		float Shoot_Circle;
		float Shoot_Hold;
		float Shoot_Speed;
		float Shoot_ErrorPos;
		float Pull_Max;

		float Shoot_StopTime;
		float Shoot_LowSpeed;
		float Shoot_HightSpeed;
		float Pull_InitSpeed;
		float Pull_LeftFirst;
		float Pull_RightFirst;
		float Pull_LeftSecond;
		float Pull_RightSecond;
		float Pull_LeftThird;
		float Pull_RightThird;
		
		float Servo_CtrlTime;//舵机：控制间隔*2,需为2的倍数
		float Servo_TorqueCtrl;//舵机：使能失能的控制指令
		float Servo_PosCtrl;//舵机：控制位置的控制指令
		float Servo_InitPos;//舵机：初始位置
		float Servo_OverPos;//舵机：发射位置
		float Servo_ErrorPos;//舵机：判断条件的范围
	}Param;

	uint8_t ProblemDetection(void);
	void AngleLimit(void);
	void Shoot_Sensor(GPIO_PinState io_pin);
	void Servo_Control(void);
	void PullTar_Update(void);
	void ShootSpe_Update(void);
	void PullMotor_PIDCalc(void);
	void ShootMotor_PIDCalc(void);
	bool Set_TurnPlacel(int mode,int position);
public:
	Shoot_classdef();
	//发送发射电机数据用
	Motor_M2006 Motor[4] =  {Motor_M2006(1), Motor_M2006(2), Motor_M2006(3), Motor_M2006(4)};  
	
	// BsqJnP2_Classdef BsqJnP2;

	uint8_t Turn_Pull_Flag;
	float LeftPull_TarAngle, RightPull_TarAngle;
	float Manual_Flag;
	float Shoot_C6;
	uint8_t Ctrl_Flag;
	uint8_t sj;
	
	//发射电机1
	Motor_M3508 Shoot_Motor =  Motor_M3508(1);  
	PositionPID Shoot_PID[2];   /*<! 发射电机 位置式PID */
	//左拉力调整电机2
  	Motor_M2006 LeftPull_Motor =  Motor_M2006(2);
	PositionPID LeftPull_PID[2];
	//右拉力调整电机3
	Motor_M2006 RightPull_Motor =  Motor_M2006(3);  
	PositionPID RightPull_PID[2];

	MX106_classdef Shoot_Servo = MX106_classdef(2);
		
	// PositionPID Pre_PID[2];

	Shoot_CtrlMode_e Shoot_Mode;
	Shoot_CtrlMode_e Shoot_Next_Mode;
	Pull_CtrlMode_e Pull_Mode;
	Pull_CtrlMode_e Pull_Next_Mode;

	
  void Control();          /*<! 发射控制 */

	bool Set_Shoot(bool shoot); 
	bool Set_ShootServo(Tar_Select_e sta);
	void Shoot_Mode_Set(Shoot_CtrlMode_e mode);
	void Pull_Mode_Set(Pull_CtrlMode_e mode);
	bool Pull_Move(Tar_Select_e pos);
	
	int Left_CS_SJ,Right_CS_SJ;
};
#endif
