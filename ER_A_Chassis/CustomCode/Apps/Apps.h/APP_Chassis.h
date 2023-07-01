#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "APP_DR16.h"
#include "DEV_BrtEncoder.h"
#include "DEV_Bldc.h"
#include "DEV_MX106.h"

#ifdef __cplusplus

//#pragma diag_suppress 550

/* Exported types ------------------------------------------------------------*/
#define AUSARTC_TX_SIZE 21
#define CUSARTA_RX_SIZE 11
#define IMU_RecvCan_SIZE 8


//四轮
//右上 逆时针旋转
/* --- 底盘驱动电机 ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    Front_Drv = 0,Left_Drv = 1,Right_Drv = 2
};
/* --- 底盘转向电机 ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    Front_Rud = 0,Left_Rud = 1,Right_Rud = 2
};

/* --- 底盘控制模式 ------------------------------------------------------------*/
enum CHAS_CtrlMode_e
{
	CHAS_DisableMode = 0,	    // 失能模式
    CHAS_LockMode,		    // 锁住底盘
	CHAS_MoveMode,          // 底盘运动模式
	CHAS_ControlMode,
    CHAS_IMUMode,
    CHAS_LaserMode,
    CHAS_AutoMode, 
    CHAS_TransiMode,
    CHAS_PostureMode,       //
		CHAS_FZSPMode,
	CHAS_TQQHMode,
	CHAS_CSMode,
	CHAS_InitMode,          // 初始化阶段
};


#pragma pack(1)
typedef struct
{
	float Yaw_Z;
	float Gz;
}IMU_RecvCan_Pack;
#pragma pack()
typedef union
{
    IMU_RecvCan_Pack Pack;
	uint8_t data[IMU_RecvCan_SIZE];
}IMURecvCanMsg_u;



/* --- 转向轮电机相关参数 -------------------------------------------------------*/

/* 滤波后的目标值 */
typedef struct 
{
    float Vx;
    float Vy;
    float Vw;
}FilterTarget_t;

/*
    (2)0x202|12    (1)0x201|11
    (3)0x203|13    (4)0x204|14
*/
class Chassis_classdef : public CTRL_DR16_classdef
{
private:
    float FRONT[3], XYZ_Angle[3], XYZ_PreTar_Angle[3];/*<! 舵向轮 */
    float falsh[3];//驱动轮方向
    float Three_Speed[3];//32767
    float Radius = 1.0f;  // 圆心距
    float AF_WtoXY,AF_WtoXY_Stand;//根据yaw轴限xy轴速度参数

    bool two_count;
    float Pos_Target[3];

    float Ramp_Vy, Ramp_Vx, Ramp_Vw;
    float ACCCCC_VAL = 20.0f, DECCCCC_VAL = 40.0f;

    CHAS_CtrlMode_e Next_Mode;
    CHAS_CtrlMode_e Mode;
    CHAS_CtrlMode_e Last_Mode;   

    //PID
    PositionPID Repair_PID[2];
    PositionPID RUD_PID[3][2];
    IncrementPID DRV_PID[3]; 

    void Sensor(void);
    uint8_t ProblemDetection(void);
    void ChassisTar_Update(void);
    void Process(float Vx, float Vy, float Vw);             /*<! 底盘数据处理 */
    void Send_Data();
    void Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal); /*<! 底盘速度斜坡 */
    void Rudder_Solve(float Vx, float Vy, float Vw, float *cal_speed);
    void RUD_PIDCalc(uint8_t motor);
    void RudAngle_Calc(float Vx, float Vy, float Vw);
    void Angle_Treatment(void);


public:
    Chassis_classdef(); 
    
    float Cal_Speed[4];//速度
    GPIO_PinState EdgeDete[8];
    uint8_t over_init;//底盘初始化标记
    uint8_t NO_PostureMode = 0;
    GPIO_PinState Ready_Flag = GPIO_PIN_RESET;
    GPIO_PinState Last_Ready_Flag = GPIO_PIN_RESET;
		uint8_t xxx_flag;
    
    PositionPID POS_PID[3][2];
    PositionPID Laser_PID[2];
		PositionPID POS_X_PID;
		
    //电机驱动+编码器
    Motor_M2006 RUD_Motor[3] = {Motor_M2006(1), Motor_M2006(2), Motor_M2006(3)}; /*<! ת���� 2006��� */
    Encider_Brt RUD_Encider[3] = {Encider_Brt(2), Encider_Brt(4), Encider_Brt(3)}; /*<! ת���־���ʽ������ */	
    //电机驱动
    BldcDrive_VESC DRV_Motor[3] = {BldcDrive_VESC(11), BldcDrive_VESC(12), BldcDrive_VESC(13)};/*<! 驱动轮 本杰明电调  */
	
    IMURecvCanMsg_u RecvCan_Msg; 

    IIRLowPassFilter Vx_LPF = IIRLowPassFilter(1.0f); /*<! 低通滤波 */
    IIRLowPassFilter Vy_LPF = IIRLowPassFilter(1.0f); /*<! 低通滤波 */
    IIRLowPassFilter Vw_LPF = IIRLowPassFilter(1.0f); /*<! 低通滤波 */

    void Control();

    void CAN_Send(void);
    void CAN_Recvd(uint8_t can_rx_data[]);
    void Set_Mode(CHAS_CtrlMode_e mode);
};

// extern float test_Vz;

#endif

#endif
