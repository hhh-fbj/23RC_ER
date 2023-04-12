/**
 * @file DJI_IMU.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DEV_CIMU_H
#define __DEV_CIMU_H
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <cmsis_compiler.h>
#include "stm32f4xx_hal.h"
#include "BSP_CAN.h"

#ifdef  __cplusplus

#define DJI_C_Angle 0x195
#define DJI_C_Gyro 0x165
#define DJI_C_Yaw 0x370

#define DJI_C_BuffSIZE 18

#define Angle_turn_Radian 57.295779513082320876798154814105f

//处理过数据
typedef struct
{
	float yaw;
	float last_yaw;
	int32_t yaw_turnCounts;
	float total_yaw;
	float pitch;
	float last_pitch;
	int32_t pitch_turnCounts;
	float total_pitch;
	float Gyro_z;
	float Gyro_y;
} DJIC_IMU_t;



//DJI_C 陀螺仪角度接收联合体
typedef union
{
	struct
	{
		float yaw;
		float pitch;
	};
	uint8_t BraodData[8];
} DJI_C_Euler_u;

//DJI_C 陀螺仪角速度接收联合体
typedef union
{
	struct
	{
		
		float Gyro_z;
		float Gyro_y;
	};
	uint8_t BraodData[8];
} DJI_C_Gyro_u;





class CIMU_Data_Classdef
{
private: 
    

public: 
	CIMU_Data_Classdef();
    DJI_C_Gyro_u DJI_C_Gyro_Receive;
    DJI_C_Euler_u DJI_C_Euler_Receive;
    DJIC_IMU_t DJIC_IMU;

    float CIMU_onlyAngle[3];
    float CIMU_angle[3];
    float CIMU_gyro[3];

    void Euler_Update(uint8_t can_rx_data[]);
    void Gyro_Update(uint8_t can_rx_data[]);
    void Updata_Hand_Euler_Gyro_Data(void);
};

#endif

#endif