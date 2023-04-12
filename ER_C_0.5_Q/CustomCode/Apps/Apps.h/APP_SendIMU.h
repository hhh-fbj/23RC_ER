#ifndef __APP_SENDIMU_H
#define __APP_SENDIMU_H

#define IMU_Send_SIZE 11
#define IMU_SendCan_SIZE 8

#include "include.h"

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
typedef struct
{
	uint8_t start_tag;		/*<! 帧头'S' */
	uint8_t mode;
	float Yaw_Z;
	float Gz;
	uint8_t end_tag;		/*<! 帧尾'E' */
}IMU_Send_Pack;
#pragma pack()
typedef union
{
    IMU_Send_Pack Pack;
	uint8_t data[IMU_Send_SIZE];
}IMUSendMsg_u;

#pragma pack(1)
typedef struct
{
	float Yaw_Z;
	float Gz;
}IMU_SendCan_Pack;
#pragma pack()
typedef union
{
    IMU_SendCan_Pack Pack;
	uint8_t data[IMU_SendCan_SIZE];
}IMUSendCanMsg_u;

class SendIMU_classdef
{
private:
    float Last_Yaw, Last_Gz;
public:
	SendIMU_classdef();
    IMUSendMsg_u Send_Msg; 
		IMUSendCanMsg_u SendCan_Msg;
    int16_t init_cnt;
    uint8_t init_mode = true;
    IMU_Data_t UseIMU;  /*<! A or C 板陀螺仪 */

    void IMU_Process(void);
	void IMU_Update(float *angle, float *gyro); /*<! IMU数据更新 */
	void wait_imuInit(void);
	float Imu_Det(float eup);
    void IMU_Send(void);
};



#endif
