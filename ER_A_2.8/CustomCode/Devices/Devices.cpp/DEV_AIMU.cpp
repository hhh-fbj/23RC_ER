/**
 * @file DJI_IMU.c
 * @author Miraggio (w1159904119@gmail)
 * @brief A板板载陀螺仪配置
 * @version 0.1
 * @date 2021-04-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "DEV_AIMU.h"
/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "tim.h"

#include "ALGO_Filter.h"

#include "DEV_mpu6500_Add.h"
#include "DEV_ist8310_Add.h"
#include "DEV_mytype.h"

#include "System_DataPool.h"

static uint8_t tx, rx;

/* --- 陀螺仪 主控制 -----------------------------------------------------------*/
//陀螺仪相关 初始化
AIMU_Data_Classdef::AIMU_Data_Classdef()
{
	Preserve_temp(40.0f); //初始化温度值
	
	//温补PID设置
  IMU_TEMP_PID.SetPIDParam(1600.0f, 100.0f, 0.0f, 4400, 4500, 1.0f);
  IMU_TEMP_PID.I_SeparThresh = 60000;
}

void AIMU_Data_Classdef::DJI_IMU_Init()
{
  mpu_device_init();//初始化陀螺仪
	init_quaternion();//初始化四元数
}
//温补加上获取数据
void AIMU_Data_Classdef::IMU_GetData_Compensate()
{
  IMU_temp_Control(Calibrate_Temperate);
  HAL_mpu_get_data();
  HAL_imu_ahrs_update();
  HAL_imu_attitude_update();
}
//输出数据获取
float AIMU_Data_Classdef::getExport(AIMU_ExportType_e mode)
{
  return Export_Datas[mode];
}
//陀螺仪检测
float AIMU_Data_Classdef::Imu_Det(float eup)
{
#if CIMU_Code == Old_CIM_Code
  if(-PI <= abs(eup) && abs(eup) <= PI)
  {
    return 1;
  }
  else
  {
    return 0;
  }
#elif CIMU_Code == New_CIMU_Code
  if(0 <= abs(eup) && abs(eup) <= 360)
  {
    return 1;
  }
  else
  {
    return 0;
  }
#endif
}

/* --- 陀螺仪 温度控制+获取数据 -----------------------------------------------------------*/
//温度补偿
void AIMU_Data_Classdef::IMU_temp_Control(float temp)
{
  uint16_t tempPWM;
  // static uint8_t temp_constant_time = 0;
  if (first_temperate)
  {
    IMU_TEMP_PID.Target = temp;
    IMU_TEMP_PID.Current = get_control_temperate();
    Out = IMU_TEMP_PID.Cal();
    if (Out < 0.0f)
    {
      Out = 0.0f;
    }
    if(get_control_temperate() >= temp)
    {
      Temp_ReachFlag = 1;
    }
    tempPWM = (uint16_t)Out;
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, tempPWM);
  }
  else
  {
    //在没有达到设置的温度，一直最大功率加热
    if (temp * 0.8f > get_control_temperate())
    {
      // temp_constant_time++;
      // if (temp_constant_time > 100)
      // {
      //     //达到设置温度，将积分项设置为一半最大功率，加速收敛
      //     first_temperate = 1;
      //     imuTempPid.i_out = MPU6500_TEMP_PWM_MAX / 2.0f;
      // }
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, MPU6500_TEMP_PWM_MAX - 1);
    }
    else
    {
        IMU_TEMP_PID.I_Term = MPU6500_TEMP_PWM_MAX / 2.0f;
        first_temperate = 1;
    }

      
  }
}
//获取陀螺仪 温度℃
float AIMU_Data_Classdef::get_control_temperate()
{
  return wve[imu_Other][imu_Temp];
}

//得到陀螺仪原始数据——扩展
void AIMU_Data_Classdef::HAL_mpu_get_data()
{
  mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
	
//	xyz[Original_a][IMU_X] = mpu_buff[0] << 8 | mpu_buff[1];
//  xyz[Original_a][IMU_Y] = mpu_buff[2] << 8 | mpu_buff[3];
//  xyz[Original_a][IMU_Z] = mpu_buff[4] << 8 | mpu_buff[5];
//  xyz[imu_other][imu_Temp] = mpu_buff[6] << 8 | mpu_buff[7];
//	
//  xyz[Original_g][IMU_X] = (mpu_buff[8] << 8 | mpu_buff[9]);
//  xyz[Original_g][IMU_Y] = (mpu_buff[10] << 8 | mpu_buff[11]);
//  xyz[Original_g][IMU_Z] = (mpu_buff[12] << 8 | mpu_buff[13]);
	
//	wxy_send[0] = xyz[Original_g][IMU_X];
//	wxy_send[1] = xyz[Original_g][IMU_Y];
//	wxy_send[2] = xyz[Original_g][IMU_Z];

  xyz[Original_a][IMU_X] = (mpu_buff[0] << 8 | mpu_buff[1] - xyz[Offset_a][IMU_X]);
  xyz[Original_a][IMU_Y] = (mpu_buff[2] << 8 | mpu_buff[3] - xyz[Offset_a][IMU_Y]);
  xyz[Original_a][IMU_Z] = (mpu_buff[4] << 8 | mpu_buff[5] -  xyz[Offset_a][IMU_Z]);
  xyz[imu_other][imu_Temp] = mpu_buff[6] << 8 | mpu_buff[7];

  xyz[Original_g][IMU_X] = ((mpu_buff[8] << 8 | mpu_buff[9]) - xyz[Offset_g][IMU_X]);
  xyz[Original_g][IMU_Y] = ((mpu_buff[10] << 8 | mpu_buff[11]) - xyz[Offset_g][IMU_Y]);
  xyz[Original_g][IMU_Z] = ((mpu_buff[12] << 8 | mpu_buff[13]) - xyz[Offset_g][IMU_Z]);

  ist8310_get_data(ist_buff);
  memcpy(&xyz[Original_m][IMU_X], ist_buff, 6);

  memcpy(&xyz[imu_a][IMU_X], &xyz[Original_a][IMU_X], 6 * sizeof(int16_t));

  wve[imu_Other][imu_Temp] = 21 + xyz[imu_other][imu_Temp] / 333.87f;
  /* 2000dps -> rad/s */
  wve[imu_w][IMU_X] = xyz[Original_g][IMU_X] / 16.384f / 57.3f;
  wve[imu_w][IMU_Y] = xyz[Original_g][IMU_Y] / 16.384f / 57.3f;
  wve[imu_w][IMU_Z] = xyz[Original_g][IMU_Z] / 16.384f / 57.3f;
}
//转四元数 处理
void AIMU_Data_Classdef::HAL_imu_ahrs_update()
{
  const float Kp = 2.0f;  /*                                           \
                  * proportional gain governs rate of         \
                  * convergence to accelerometer/magnetometer \
                  */
  const float Ki = 0.01f; /*                                 \
                  * integral gain governs rate of   \
                  * convergence of gyroscope biases \
                  */

  static volatile uint32_t last_update, now_update;
  static volatile float exInt, eyInt, ezInt; /* error integral */
  static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
  
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez, halfT;
  float tempq0, tempq1, tempq2, tempq3;

  float q0q0 = q[0] * q[0];
  float q0q1 = q[0] * q[1];
  float q0q2 = q[0] * q[2];
  float q0q3 = q[0] * q[3];
  float q1q1 = q[1] * q[1];
  float q1q2 = q[1] * q[2];
  float q1q3 = q[1] * q[3];
  float q2q2 = q[2] * q[2];
  float q2q3 = q[2] * q[3];
  float q3q3 = q[3] * q[3];

  gx = wve[imu_w][IMU_X];
  gy = wve[imu_w][IMU_Y];
  gz = wve[imu_w][IMU_Z];
  ax = xyz[imu_a][IMU_X];
  ay = xyz[imu_a][IMU_Y];
  az = xyz[imu_a][IMU_Z];
  mx = xyz[imu_m][IMU_X];
  my = xyz[imu_m][IMU_Y];
  mz = xyz[imu_m][IMU_Z];

  now_update = HAL_GetTick(); //ms
  halfT = ((float)(now_update - last_update) / 2000.0f);
  last_update = now_update;

  /* Fast inverse square-root */
  norm = inv_sqrt(ax * ax + ay * ay + az * az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

#ifdef IST8310
  norm = inv_sqrt(mx * mx + my * my + mz * mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
#else
  mx = 0;
  my = 0;
  mz = 0;
#endif
  /* compute reference direction of flux */
  hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
  hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
  hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = hz;

  /* estimated direction of gravity and flux (v and w) */
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

  /* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

  /* PI */
  if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;
    ezInt = ezInt + ez * Ki * halfT;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
  }

  tempq0 = q[0] + (-q[1] * gx - q[2] * gy - q[3] * gz) * halfT;
  tempq1 = q[1] + (q[0] * gx + q[2] * gz - q[3] * gy) * halfT;
  tempq2 = q[2] + (q[0] * gy - q[1] * gz + q[3] * gx) * halfT;
  tempq3 = q[3] + (q[0] * gz + q[1] * gy - q[2] * gx) * halfT;

  /* normalise quaternion */
  norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
  q[0] = tempq0 * norm;
  q[1] = tempq1 * norm;
  q[2] = tempq2 * norm;
  q[3] = tempq3 * norm;
}
//转欧拉角 更新
void AIMU_Data_Classdef::HAL_imu_attitude_update()
{
#if CIMU_Code == Old_CIMU_Code
  //为了适应云台取陀螺仪数据
  AIMU_onlyAngle[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1);
  AIMU_onlyAngle[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
  AIMU_onlyAngle[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
  for(uint8_t i = 0 ; i < 3 ; i++)
  {
    AIMU_gyro[i] = wve[imu_w][i];
  }
	
  if(Imu_Det(AIMU_onlyAngle[0]) && Imu_Det(AIMU_onlyAngle[1]) && Imu_Det(AIMU_onlyAngle[2]))
  {
    DevicesMonitor.Update(Frame_IMBAL_AIMU);
  }
#elif CIMU_Code == New_CIMU_Code
  // //旧的
	 /* yaw    -pi----pi */
	 wve[imu_euler][imu_Yaw] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 57.3 + 180.0f;
	 /* pitch  -pi/2----pi/2 */
	 wve[imu_euler][imu_Pit] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 57.3 + 180.0f;// TODO pit与roll没反了
	 /* roll   -pi----pi  */
	 wve[imu_euler][imu_Rol] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 57.3 + 180.0f;//

	//  if (wve[imu_euler][imu_Yaw] - wve[imu_Previous][imu_Yaw] < -300)
	//  { //经过跳变边沿。
	// 	 xyz[imu_other][imu_Yaw]++;
	//  }
	//  if (wve[imu_Previous][imu_Yaw] - wve[imu_euler][imu_Yaw] < -300)
	//  {
	// 	 xyz[imu_other][imu_Yaw]--;
	//  }
	//  wve[imu_Total][imu_Yaw] = wve[imu_euler][imu_Yaw] + (360 * xyz[imu_other][imu_Yaw]);

	 wve[imu_Previous][imu_Yaw] = wve[imu_euler][imu_Yaw];

	//  if (wve[imu_euler][imu_Pit] - wve[imu_Previous][imu_Pit] < -300)
	//  { //经过跳变边沿。
	// 	 xyz[imu_other][imu_Pit]++;
	//  }
	//  if (wve[imu_Previous][imu_Pit] - wve[imu_euler][imu_Pit] < -300)
	//  {
	// 	 xyz[imu_other][imu_Pit]--;
	//  }
	//  wve[imu_Total][imu_Pit] = wve[imu_euler][imu_Pit] + (360 * xyz[imu_other][imu_Pit]);

	 wve[imu_Previous][imu_Pit] = wve[imu_euler][imu_Pit];

	//  Export_Datas[gz] = (float)xyz[Original_g][IMU_Z] / 16.384f ;
	//  Export_Datas[gy] = (float)xyz[Original_g][IMU_Y] / 16.384f ;

	//  Export_Datas[Total_Yaw] = wve[imu_Total][imu_Yaw];
	//  Export_Datas[Total_Pit] = wve[imu_Total][imu_Pit];
	 
	 AIMU_onlyAngle[0] = wve[imu_euler][imu_Yaw];
	 AIMU_onlyAngle[1] = wve[imu_euler][imu_Pit];
   AIMU_onlyAngle[2] = wve[imu_euler][imu_Rol];
	 
	 AIMU_gyro[0] = (float)xyz[Original_g][IMU_X] / 16.384f ;
	 AIMU_gyro[1] = (float)xyz[Original_g][IMU_Y] / 16.384f ;
   AIMU_gyro[2] = (float)xyz[Original_g][IMU_Z] / 16.384f ;
	 
	//  AIMU_angle[0] = wve[imu_Total][imu_Yaw];
	//  AIMU_angle[1] = wve[imu_Total][imu_Pit];
  //  AIMU_angle[2] = wve[imu_Total][imu_Rol];

//  if(Imu_Det(AIMU_onlyAngle[0]) && Imu_Det(AIMU_onlyAngle[1]) && Imu_Det(AIMU_onlyAngle[2]))
//  {
//    DevicesMonitor.Update(Frame_IMBAL_AIMU);
//  }

	//  Filter_IIRLPF(&Export_Datas[gz], &Export_Datas[Finally_gz], 0.1);
#endif
}



/* --- 陀螺仪相关 初始化 -----------------------------------------------------------*/
//陀螺仪 设置初始化
uint8_t AIMU_Data_Classdef::mpu_device_init()
{
  static uint8_t id;
  MPU_DELAY(100);

  id = mpu_read_byte(MPU6500_WHO_AM_I);
  uint8_t i = 0;
  uint8_t MPU6500_Init_Data[10][2] = {
      {MPU6500_PWR_MGMT_1, 0x80},     /* Reset Device */
      {MPU6500_PWR_MGMT_1, 0x03},     /* Clock Source - Gyro-Z */
      {MPU6500_PWR_MGMT_2, 0x00},     /* Enable Acc & Gyro */
      {MPU6500_CONFIG, 0x04},         /* LPF 41Hz */
      {MPU6500_GYRO_CONFIG, 0x18},    /* +-2000dps */
      {MPU6500_ACCEL_CONFIG, 0x10},   /* +-8G */
      {MPU6500_ACCEL_CONFIG_2, 0x02}, /* enable LowPassFilter  Set Acc LPF */
      {MPU6500_USER_CTRL, 0x20},
  }; /* Enable AUX */
  for (i = 0; i < 10; i++)
  {
    mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    MPU_DELAY(1);
  }

  //设置精度
  mpu_set_gyro_fsr(3);
  mpu_set_accel_fsr(2);

  ist8310_init();
  IMU_Init_Condition = 1;

#if AIMU_Calibrate == AIMU_Calibrate_On
  while (Temp_ReachFlag == 0)
  {
    Temp_ReachFlag = 1;
  };
  mpu_offset_call();
#elif AIMU_Calibrate == AIMU_Calibrate_Off
  mpu_offset_fetch();

#endif
  return 0;
}
//四元数 初始化
void AIMU_Data_Classdef::init_quaternion()
{
  int16_t hx, hy, hz;

  hx = xyz[imu_m][IMU_X];
  hy = xyz[imu_m][IMU_Y];
  hz = xyz[imu_m][IMU_Z];

#ifdef BOARD_DOWN
  if (hx < 0 && hy < 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = -0.005;
      q[1] = -0.199;
      q[2] = 0.979;
      q[3] = -0.0089;
    }
    else
    {
      q[0] = -0.008;
      q[1] = -0.555;
      q[2] = 0.83;
      q[3] = -0.002;
    }
  }
  else if (hx < 0 && hy > 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = 0.005;
      q[1] = -0.199;
      q[2] = -0.978;
      q[3] = 0.012;
    }
    else
    {
      q[0] = 0.005;
      q[1] = -0.553;
      q[2] = -0.83;
      q[3] = -0.0023;
    }
  }
  else if (hx > 0 && hy > 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = 0.0012;
      q[1] = -0.978;
      q[2] = -0.199;
      q[3] = -0.005;
    }
    else
    {
      q[0] = 0.0023;
      q[1] = -0.83;
      q[2] = -0.553;
      q[3] = 0.0023;
    }
  }
  else if (hx > 0 && hy < 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = 0.0025;
      q[1] = 0.978;
      q[2] = -0.199;
      q[3] = 0.008;
    }
    else
    {
      q[0] = 0.0025;
      q[1] = 0.83;
      q[2] = -0.56;
      q[3] = 0.0045;
    }
  }
#else
  if (hx < 0 && hy < 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = 0.195;
      q[1] = -0.015;
      q[2] = 0.0043;
      q[3] = 0.979;
    }
    else
    {
      q[0] = 0.555;
      q[1] = -0.015;
      q[2] = 0.006;
      q[3] = 0.829;
    }
  }
  else if (hx < 0 && hy > 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = -0.193;
      q[1] = -0.009;
      q[2] = -0.006;
      q[3] = 0.979;
    }
    else
    {
      q[0] = -0.552;
      q[1] = -0.0048;
      q[2] = -0.0115;
      q[3] = 0.8313;
    }
  }
  else if (hx > 0 && hy > 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = -0.9785;
      q[1] = 0.008;
      q[2] = -0.02;
      q[3] = 0.195;
    }
    else
    {
      q[0] = -0.9828;
      q[1] = 0.002;
      q[2] = -0.0167;
      q[3] = 0.5557;
    }
  }
  else if (hx > 0 && hy < 0)
  {
    if (fabs(hx / hy) >= 1)
    {
      q[0] = -0.979;
      q[1] = 0.0116;
      q[2] = -0.0167;
      q[3] = -0.195;
    }
    else
    {
      q[0] = -0.83;
      q[1] = 0.014;
      q[2] = -0.012;
      q[3] = -0.556;
    }
  }
#endif
}
//温度设置
void AIMU_Data_Classdef::Preserve_temp(float Real_temp)
{
    Calibrate_Temperate = Real_temp;
    if (Calibrate_Temperate > (int8_t)GYRO_CONST_MAX_TEMP)
    {
        Calibrate_Temperate = (int8_t)GYRO_CONST_MAX_TEMP;
    }
}

//磁力计 初始化
uint8_t AIMU_Data_Classdef::ist8310_init()
{
  /* enable iic master mode */
  mpu_write_byte(MPU6500_USER_CTRL, 0x30);
  MPU_DELAY(10);
  /* enable iic 400khz */
  mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
  MPU_DELAY(10);

  /* turn on slave 1 for ist write and slave 4 to ist read */
  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  MPU_DELAY(10);
  mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  MPU_DELAY(10);

  /* IST8310_R_CONFB 0x01 = device rst */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_DELAY(10);
  if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
    return 1;

  /* soft reset */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_DELAY(10);

  /* config as ready mode to access register */
  ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
    return 2;
  MPU_DELAY(10);

  /* normal state, no int */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
    return 3;
  MPU_DELAY(10);

  /* config low noise mode, x,y,z axis 16 time 1 avg */
  ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
  if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
    return 4;
  MPU_DELAY(10);

  /* Set/Reset pulse duration setup,normal mode */
  ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
  if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
    return 5;
  MPU_DELAY(10);

  /* turn off slave1 & slave 4 */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(10);
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_DELAY(10);

  
  mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  MPU_DELAY(100);
  return 0;
}



/* --- 精度设置+校准 -----------------------------------------------------------*/
//设置角速度精度
uint8_t AIMU_Data_Classdef::mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}
//设置加速度精度
uint8_t AIMU_Data_Classdef::mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

//陀螺仪 校正 数据采集——非零
void AIMU_Data_Classdef::mpu_offset_call()
{
  int i;
  for (i = 0; i < 300; i++)
  {
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    xyz[Offset_a][IMU_X] += mpu_buff[0] << 8 | mpu_buff[1];
    xyz[Offset_a][IMU_Y] += mpu_buff[2] << 8 | mpu_buff[3];
    xyz[Offset_a][IMU_Z] += mpu_buff[4] << 8 | mpu_buff[5];

    xyz[Offset_g][IMU_X] += mpu_buff[8] << 8 | mpu_buff[9];
    xyz[Offset_g][IMU_Y] += mpu_buff[10] << 8 | mpu_buff[11];
    xyz[Offset_g][IMU_Z] += mpu_buff[12] << 8 | mpu_buff[13];

    MPU_DELAY(5);
  }
  xyz[Offset_a][IMU_X] = xyz[Offset_a][IMU_X] / 300;
  xyz[Offset_a][IMU_Y] = xyz[Offset_a][IMU_Y] / 300;
  xyz[Offset_a][IMU_Z] = xyz[Offset_a][IMU_Z] / 300;
  xyz[Offset_g][IMU_X] = xyz[Offset_g][IMU_X] / 300;
  xyz[Offset_g][IMU_Y] = xyz[Offset_g][IMU_Y] / 300;
  xyz[Offset_g][IMU_Z] = xyz[Offset_g][IMU_Z] / 300;

  IMUwriteFlashData[0] = xyz[Offset_a][IMU_X];
  IMUwriteFlashData[1] = xyz[Offset_a][IMU_Y];
  IMUwriteFlashData[2] = xyz[Offset_a][IMU_Z];
  IMUwriteFlashData[3] = xyz[Offset_g][IMU_X];
  IMUwriteFlashData[4] = xyz[Offset_g][IMU_Y];
  IMUwriteFlashData[5] = xyz[Offset_g][IMU_Z];
}
//陀螺仪 校正 数据采集——全0
void AIMU_Data_Classdef::mpu_offset_fetch()
{
  xyz[Offset_a][IMU_X] = (uint16_t)IMU_Offset[0];
  xyz[Offset_a][IMU_Y] = (uint16_t)IMU_Offset[1];
  xyz[Offset_a][IMU_Z] = (uint16_t)IMU_Offset[2];
  xyz[Offset_g][IMU_X] = (uint16_t)IMU_Offset[3];
  xyz[Offset_g][IMU_Y] = (uint16_t)IMU_Offset[4];
  xyz[Offset_g][IMU_Z] = (uint16_t)IMU_Offset[5];
}



/* --- 其他设置 -----------------------------------------------------------*/
//设置自动读写 ISP 数据 到 MPU
void AIMU_Data_Classdef::mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  MPU_DELAY(2);

  /* use slave0,auto read data */
  mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
  MPU_DELAY(2);

  /* every eight mpu6500 internal samples one i2c master read */
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
  MPU_DELAY(2);
  /* enable slave 0 and 1 access delay */
  mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  MPU_DELAY(2);
  /* enable slave 1 auto transmit */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  /* Wait 6ms (minimum waiting time for 16 times internal average setup) */
  MPU_DELAY(6);
  /* enable slave 0 with data_num bytes reading */
  mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  MPU_DELAY(2);
}

//SPI通讯状态
//读 MPU 多字节
HAL_StatusTypeDef AIMU_Data_Classdef::mpu_read_bytes(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t tx_buff[14] = {0xff};
  HAL_StatusTypeDef InfoUpdateFlag = HAL_ERROR;
  MPU_NSS_LOW;
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  InfoUpdateFlag = HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
  MPU_NSS_HIGH;
  return InfoUpdateFlag;
}

//牛顿迭代法
float AIMU_Data_Classdef::inv_sqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;

  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));

  return y;
}



/* ---陀螺仪读写 -----------------------------------------------------------*/
//读 MPU 单字节
uint8_t AIMU_Data_Classdef::mpu_read_byte(uint8_t const reg)
{
  MPU_NSS_LOW;
  tx = reg | 0x80;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH;
  return rx;
}
//写 MPU 单字节
uint8_t AIMU_Data_Classdef::mpu_write_byte(uint8_t const reg, uint8_t const data)
{
  MPU_NSS_LOW;
  tx = reg & 0x7F;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  tx = data;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH;
  return 0;
}
//从 MPU 写 IST 单字节
void AIMU_Data_Classdef::ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
  /* turn off slave 1 at first */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
  MPU_DELAY(2);
  /* turn on slave 1 with one byte transmitting */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  /* wait longer to ensure the data is transmitted from slave 1 */
  MPU_DELAY(10);
}
//从 MPU 读 IST 单/多字节
uint8_t AIMU_Data_Classdef::ist_reg_read_by_mpu(uint8_t addr)
{
  uint8_t retval;
  mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
  MPU_DELAY(10);
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
  MPU_DELAY(10);
  retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
  /* turn off slave4 after read */
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_DELAY(10);
  return retval;
}
//读 MPU 的 ISP 多字节
void AIMU_Data_Classdef::ist8310_get_data(uint8_t *buff)
{
  mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}


