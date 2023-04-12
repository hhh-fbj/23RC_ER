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


//����
//���� ��ʱ����ת
/* --- ����������� ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_Drv = 0,LF_Drv = 1,LB_Drv = 2,RB_Drv = 3
};
/* --- ����ת���� ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    RF_Rud = 0,LF_Rud = 1,LB_Rud = 2,RB_Rud = 3
};

/* --- ���̿���ģʽ ------------------------------------------------------------*/
enum CHAS_CtrlMode_e
{
	CHAS_DisableMode = 0,	    // ʧ��ģʽ
    CHAS_LockMode,		    // ��ס����
	CHAS_MoveMode,          // �����˶�ģʽ
	CHAS_ControlMode,
    CHAS_IMUMode,
    CHAS_LaserMode,
    CHAS_AutoMode, 
    CHAS_TransiMode,
    CHAS_PostureMode,       //
	CHAS_InitMode,          // ��ʼ���׶�
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

typedef union
{
    float speed[2];
	uint8_t data[8];
}SendSpeed_u;

/* --- ת���ֵ����ز��� -------------------------------------------------------*/

/* �˲����Ŀ��ֵ */
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
    float FRONT[4], XYZ_Angle[4], XYZ_PreTar_Angle[4];/*<! ������ */
    float falsh[4];//�����ַ���
    float Three_Speed[4];//32767
    SendSpeed_u SS1,SS2;

    float Pos_Target[3];

    CHAS_CtrlMode_e Next_Mode;
    CHAS_CtrlMode_e Mode;
    CHAS_CtrlMode_e Last_Mode;   

    //PID
    PositionPID Repair_PID[2];
    PositionPID RUD_PID[4][2];

    void Sensor(void);
    uint8_t ProblemDetection(void);
    void ChassisTar_Update(void);
    void Process(float Vx, float Vy, float Vw);             /*<! �������ݴ��� */
    void Send_Data();
    void Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal); /*<! �����ٶ�б�� */
    void Rudder_Solve(float Vx, float Vy, float Vw, float *cal_speed);
    void RUD_PIDCalc(uint8_t motor);
    void RudAngle_Calc(float Vx, float Vy, float Vw);
    void Angle_Treatment(void);


public:
    Chassis_classdef(); 
    
    float Cal_Speed[4];//�ٶ�
    GPIO_PinState EdgeDete[6];
    uint8_t over_init;//���̳�ʼ�����
    uint8_t NO_PostureMode = 0;
    float try_bl;
    
    PositionPID POS_PID[3][2];
    PositionPID Laser_PID[2];
		
    //�������+������
    Motor_M2006 RUD_Motor[4] = {Motor_M2006(1), Motor_M2006(2), Motor_M2006(3), Motor_M2006(4)}; /*<! ת���� 2006��� */
    Encider_Brt RUD_Encider[4] = {Encider_Brt(1), Encider_Brt(2), Encider_Brt(3), Encider_Brt(4)}; /*<! ת���־���ʽ������ */	

    IMURecvCanMsg_u RecvCan_Msg; 

    IIRLowPassFilter Vx_LPF = IIRLowPassFilter(1.0f); /*<! ��ͨ�˲� */
    IIRLowPassFilter Vy_LPF = IIRLowPassFilter(1.0f); /*<! ��ͨ�˲� */
    IIRLowPassFilter Vw_LPF = IIRLowPassFilter(1.0f); /*<! ��ͨ�˲� */

    void Control();

    void CAN_Send(void);
    void CAN_Recvd(uint8_t can_rx_data[]);
		
		
    void Set_Mode(CHAS_CtrlMode_e mode);
};

// extern float test_Vz;

#endif

#endif
