#ifndef __APP_SHOOT_H
#define __APP_SHOOT_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "APP_DR16.h"
#include "DEV_BsqJnP2.h"

enum Shoot_CtrlMode_e
{
	Shoot_DisableMode,	// 失能
	Shoot_AutoMode,
	Shoot_ManualMode,
	Shoot_LockMode,
	Shoot_TransiMode,
	Shoot_NewManualMode,
	Shoot_NewAutoMode,
};

enum Pull_CtrlMode_e
{
	Pull_DisableMode,	// 失能
	Pull_FixedMode,
	Pull_DebugMode,
	Pull_LockMode,
	Pull_TransiMode,
	Pull_NewDebugMode,
};


class Shoot_classdef
{
private:
	GPIO_PinState C6;
	GPIO_PinState E13;
	GPIO_PinState E11;
	GPIO_PinState last_E13 = GPIO_PIN_SET;
	GPIO_PinState last_E11 = GPIO_PIN_SET;

	bool Launch_Switch = false;
	bool Shoot_Continue;

	uint8_t first;

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

	struct{
		float Shoot_Circle;
		float Shoot_Hold;
		float Shoot_Speed;
		float Pull_Max;
	}Param;

	uint8_t ProblemDetection(void);
	void AngleLimit(void);
	void Shoot_Sensor(GPIO_PinState io_pin);
	void PullTar_Update(void);
	void ShootSpe_Update(void);
	void PullMotor_PIDCalc(void);
	void ShootMotor_PIDCalc(void);
public:
	Shoot_classdef();
	//发送发射电机数据用
	Motor_M2006 Motor[4] =  {Motor_M2006(1), Motor_M2006(2), Motor_M2006(3), Motor_M2006(4)};  

	// BsqJnP2_Classdef BsqJnP2;

	uint8_t Shoot_Place_Flag=0;
	float Shoot_Speed_BL;
	float LeftPull_TarAngle, RightPull_TarAngle;
	float Manual_Flag;
	float Shoot_C6;

	//发射电机1
	Motor_M3508 Shoot_Motor =  Motor_M3508(1);  
	PositionPID Shoot_PID[2];   /*<! 发射电机 位置式PID */
	//左拉力调整电机2
  	Motor_M2006 LeftPull_Motor =  Motor_M2006(2);
	PositionPID LeftPull_PID[2];
	//右拉力调整电机3
	Motor_M2006 RightPull_Motor =  Motor_M2006(3);  
	PositionPID RightPull_PID[2];

	// PositionPID Pre_PID[2];

	Shoot_CtrlMode_e Shoot_Mode;
	Shoot_CtrlMode_e Shoot_Next_Mode;
	Pull_CtrlMode_e Pull_Mode;
	Pull_CtrlMode_e Pull_Next_Mode;
	
  void Control();          /*<! 发射控制 */

	void Set_Shoot(bool sta){Launch_Switch = sta;}; 
	bool Get_ShootCon(void){return Shoot_Continue;}; 
	void Shoot_Mode_Set(Shoot_CtrlMode_e mode);
	void Pull_Mode_Set(Pull_CtrlMode_e mode);
};
#endif
