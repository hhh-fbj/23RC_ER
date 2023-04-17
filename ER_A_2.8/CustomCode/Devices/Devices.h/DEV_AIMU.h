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
#ifndef __DEV_AIMU_H
#define __DEV_AIMU_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <cmsis_compiler.h>
#include "stm32f4xx.h"
#include "ALGO_PID.h"

#ifdef  __cplusplus

#define AIMU_Calibrate     1 //是否开启陀螺仪校准
#define AIMU_Calibrate_On  1
#define AIMU_Calibrate_Off 0

#define CIMU_Code New_CIMU_Code
#define Old_CIM_Code 0
#define New_CIMU_Code 1

#define MPU_DELAY(x) HAL_Delay(x)
#define BOARD_DOWN (1)

#define INS_DELTA_TICK 1 //任务调用的间隔

#define DMA_RX_NUM 23
#define MPU6500_RX_BUF_DATA_OFFSET 0
#define IST8310_RX_BUF_DATA_OFFSET 15

#define MPU_DATA_READY_BIT 0 //陀螺仪数据准备
#define MPU_MOT_BIT 1        //mpu6500 运动检测

#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
#define WOM_THR_Set 0x0F

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为?MPU6500_TEMP_PWM_MAX?-?1
#define GYRO_CONST_MAX_TEMP 40.0f

enum AIMU_DataType_e
{
	Original_a = 0,
  Original_m = 1,
  Original_g = 2,
  Offset_a = 3,
  Offset_g = 4,
  imu_a = 5,
  imu_m = 6,
  imu_other = 7,

  imu_w = 0,
  imu_v = 1,
  imu_euler = 2,
  imu_Total = 3,
  imu_Previous = 4,
  imu_Other = 4
};

enum AIMU_ShaftType_e
{
  IMU_X = 0,
  imu_Rol = 0,
  imu_Temp = 0,

  IMU_Y = 1,
  imu_Pit = 1,

  IMU_Z = 2,
  imu_Yaw = 2
};

enum AIMU_ExportType_e
{
  gz,
  gy,
  Finally_gz,
  Total_Yaw,
  Total_Pit,
};

class AIMU_Data_Classdef
{
private: 
  volatile float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  int16_t xyz[8][3];
  float wve[5][3];
  HAL_StatusTypeDef InfoUpdateFlag;
  float Export_Datas[5];

  PositionPID IMU_TEMP_PID;
  float Out = 0; /*!< Output ampere value that sent to ESC */
  uint8_t Temp_ReachFlag;
  
  uint8_t mpu_buff[15]; /* buffer to save imu raw data */
  uint8_t ist_buff[6];  /* buffer to save IST8310 raw data */

  float inv_sqrt(float x);//牛顿迭代法
  HAL_StatusTypeDef mpu_read_bytes(uint8_t const regAddr, uint8_t *pData, uint8_t len);//SPI通讯状态
  
  uint8_t mpu_read_byte(uint8_t const reg);
  uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data);
  void ist_reg_write_by_mpu(uint8_t addr, uint8_t data);
  uint8_t ist_reg_read_by_mpu(uint8_t addr);

  uint8_t mpu_set_gyro_fsr(uint8_t fsr);
  uint8_t mpu_set_accel_fsr(uint8_t fsr);
  void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num);
  void ist8310_get_data(uint8_t *buff);

  uint8_t ist8310_init();
  void mpu_offset_call();
  void mpu_offset_fetch();
  uint8_t mpu_device_init();

  void init_quaternion();
  void HAL_mpu_get_data();
  void HAL_imu_ahrs_update();
  void HAL_imu_attitude_update();

public: 
		int16_t wxy_send[3];

  uint8_t IMU_Init_Condition;
  uint8_t first_temperate = 0;
  int8_t Calibrate_Temperate = 0;
  uint32_t IMU_Offset[6];
  uint32_t IMUwriteFlashData[6] = {0};
	float AIMU_onlyAngle[3];
  float AIMU_angle[3];
  float AIMU_gyro[3];

  AIMU_Data_Classdef();

  void DJI_IMU_Init();

  float Imu_Det(float eup);
  void IMU_GetData_Compensate();
  void Preserve_temp(float Real_temp);
  void IMU_temp_Control(float temp);
  float get_control_temperate();
  
  float getExport(AIMU_ExportType_e mode);
};
 #endif

/* #ifdef  __cplusplus
// extern "C" {
// #endif
#ifdef __cplusplus
// }
#endif */

#endif /*_DJI_IMU_H*/


