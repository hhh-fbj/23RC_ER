/**
 * @file DJI_IMU.c
 * @author Miraggio (w1159904119@gmail)
 * @brief C板板载陀螺仪配置
 * @version 0.1
 * @date 2021-04-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "DEV_CIMU.h"

CIMU_Data_Classdef::CIMU_Data_Classdef(){;}

//处理更新数据
void CIMU_Data_Classdef::Updata_Hand_Euler_Gyro_Data(void)
{
	//yaw轴的过零处理
	if (DJIC_IMU.yaw - DJIC_IMU.last_yaw < -300.0f)
	{
		DJIC_IMU.yaw_turnCounts++;
	}
	if (DJIC_IMU.last_yaw - DJIC_IMU.yaw < -300.0f)
	{
		DJIC_IMU.yaw_turnCounts--;
	}
	DJIC_IMU.total_yaw = DJIC_IMU.yaw + DJIC_IMU.yaw_turnCounts * 360.0f;
	DJIC_IMU.last_yaw = DJIC_IMU.yaw;

	//Pitch
	if (DJIC_IMU.pitch - DJIC_IMU.last_pitch < -300.0f)
	{
		DJIC_IMU.pitch_turnCounts++;
	}
	if (DJIC_IMU.last_pitch - DJIC_IMU.pitch < -300.0f)
	{
		DJIC_IMU.pitch_turnCounts--;
	}
	DJIC_IMU.total_pitch = DJIC_IMU.pitch + DJIC_IMU.pitch_turnCounts * 360.0f;
	DJIC_IMU.last_pitch = DJIC_IMU.pitch;

    CIMU_angle[0] = DJIC_IMU.total_yaw;
    CIMU_angle[1] = DJIC_IMU.total_pitch;
}

//解包DJI_C_IMU的Euler
void CIMU_Data_Classdef::Euler_Update(uint8_t can_rx_data[])
{
	memcpy(DJI_C_Euler_Receive.BraodData, can_rx_data, 8);
		//角度
	// DJIC_IMU.yaw = (float)DJI_C_Euler_Receive.yaw * Angle_turn_Radian + 180.0f;		//将弧度转为度
	// DJIC_IMU.pitch = (float)DJI_C_Euler_Receive.pitch * Angle_turn_Radian + 180.0f; //(-180° ~ 180°)
    
	CIMU_onlyAngle[0] = (float)DJI_C_Euler_Receive.yaw;
	CIMU_onlyAngle[1] = (float)DJI_C_Euler_Receive.pitch;
}

//解包DJI_C_IMU的Gyro
void CIMU_Data_Classdef::Gyro_Update(uint8_t can_rx_data[])
{
	memcpy(DJI_C_Gyro_Receive.BraodData, can_rx_data, 8);
	//角速度
	// DJIC_IMU.Gyro_z = (float)DJI_C_Gyro_Receive.Gyro_z * Angle_turn_Radian;
	// DJIC_IMU.Gyro_y = (float)DJI_C_Gyro_Receive.Gyro_y * Angle_turn_Radian;

	//云台更新陀螺仪角速度的顺序是X,Y,Z三个轴
    
	CIMU_gyro[2] = (float)DJI_C_Gyro_Receive.Gyro_z;
    CIMU_gyro[1] = (float)DJI_C_Gyro_Receive.Gyro_y;
}
