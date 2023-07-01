#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "APP_DR16.h"
#include "APP_Clamp.h"
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
	Gimbal_TransiMode,
	Gimbal_NormalMode,	// 普通运转模式
	Gimbal_PCMode,       // PC控制(自瞄)
	Gimbal_LockMode,
	Gimbal_AutoMode,
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

#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
	int error;
	float Yaw_Z;
} CIMU_Pack_t;
#pragma pack()
typedef union
{
	uint8_t data[8];
	CIMU_Pack_t Pack;
}CIMUMsg_u;

class Gimbal_classdef
{
private:
	GPIO_PinState last_I6;

	bool init_mode = true;
	float once;
	uint8_t init_cnt;	
	float UseTarget[3];

	float clamp_angle;

	struct APP_Gimbal
	{
		float Yaw_Max;
		float Yaw_Centre;
		float Yaw_Min;
		float Yaw_Speed;

		float Angle_Big;
		float Angle_small;
		float Yaw_TurnAngle;
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
	GPIO_PinState D12;
	GPIO_PinState Last_D12;
	uint8_t Midpoint_Flag;
	uint8_t Ding_TEXT_Flag;//云台中间打四柱子的转动标识符
	CIMUMsg_u CIMU;

	

	Gimbal_CtrlMode_e Mode; /*<! 云台运作模式 */
	Gimbal_CtrlMode_e Next_Mode;
	Gimbal_CtrlMode_e Last_Mode;
	
	Motor_GM6020 Yaw_Motor = Motor_GM6020(5);//6

	Encider_Brt Yaw_Encider = Encider_Brt(6);

	PositionPID UsePID[3][3];  /*<! 云台电机各模式PID */

	IMU_Data_t UseIMU;  /*<! A or C 板陀螺仪 */

	void Control();
	void CIMU_Rev(uint8_t data[8]);
	void setMode(Gimbal_CtrlMode_e mode);
	bool TarPos_Move(Tar_Select_e angle);
	
	int CS_SJ;
};
#endif

