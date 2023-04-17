#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "APP_DR16.h"
/* Exported types ------------------------------------------------------------*/
#define GIMBAL_YAW_CENTRE	2738    // Yaw 中心点

// Yaw 反向中心点

// 顶部与底部的相应数值根据编码器的安装方式而定-------------*/
#define GIMBAL_PIT_MIN		1300
#define GIMBAL_PIT_MAX		2400

/* --- 云台控制模式 -----------------------------------------------------------*/
enum Gimbal_CtrlMode_e
{
	Gimbal_DisableMode = 0,	// 失能
	Gimbal_NormalMode,	// 普通运转模式
	Gimbal_PCMode,       // PC控制(自瞄)
	Gimbal_LockMode,

	Gimbal_HalfAutoMode,
};

/* --- 云台电机 ID ------------------------------------------------------------*/
enum Gimbal_type_e
{
	Yaw,
	Pit,
	Rol
};

typedef struct
{
	float Angle[3];       /*<! IMU 角度 */
    float Pre_Angle[3];   /*<! 上一次角度 */
    float TotalAngle[3];  /*<! IMU 总角度*/
    float Gyro[3];        /*<! IMU 角速度 */
	float OffsetGyro[3];  /*<! 解算后的角速度 */
    float Cnt[3]; 	  /*<! 累计圈数 */
}IMU_Data_t;

class Gimbal_classdef
{
private:
	GPIO_PinState last_I6;

	bool init_mode = true;
	float once;
	uint8_t init_cnt;	
	float UseTarget[3];

	struct APP_Gimbal
	{
		float Yaw_Max;
		float Yaw_Centre;
		float Yaw_Min;
		float Yaw_Speed;
	}Param;
	
	
	void IMU_Update(float *angle, float *gyro); /*<! IMU数据更新 */
	void wait_imuInit(void);
	uint8_t ProblemDetection(void);
	void TargetAngle_Update(void);
	void AngleLimit(void);
	void Motor_PIDCalc();
	float Imu_Det(float eup);
	void Set_InitAngle();	   /*<! 设置初始角度 */
	float *Get_TargetAngle(Gimbal_type_e type); /*<! 获取用于计算的目标参数 */
public:
	Gimbal_classdef();
	GPIO_PinState I6;
	GPIO_PinState Last_I6;
	uint8_t Midpoint_Flag;
	Gimbal_CtrlMode_e Mode; /*<! 云台运作模式 */
	
	Motor_GM6020 Yaw_Motor = Motor_GM6020(5);//6

	Encider_Brt Yaw_Encider = Encider_Brt(6);

	PositionPID UsePID[3][3];  /*<! 云台电机各模式PID */

	IMU_Data_t UseIMU;  /*<! A or C 板陀螺仪 */

	void Control();
};
#endif

