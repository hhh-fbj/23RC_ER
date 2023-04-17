#include "APP_SendIMU.h"

#include "System_DataPool.h"

#include "INS_task.h"

SendIMU_classdef::SendIMU_classdef()
{
	Send_Msg.Pack.start_tag = 'S';
	Send_Msg.Pack.end_tag = 'E';
}

void SendIMU_classdef::IMU_Process(void)
{
		IMU_Update(INS_angle, INS_gyro);
				
    if(init_mode)
    {
				//--- �ȴ�IMU��ʼ�����
				wait_imuInit();
        return;
    }

    IMU_Send();
}

void SendIMU_classdef::IMU_Send(void)
{
		SendCan_Msg.Pack.Yaw_Z = UseIMU.TotalAngle[Yaw];
		SendCan_Msg.Pack.Gz = UseIMU.Gyro[Yaw];
		CANx_SendData(&hcan1, 0x342, SendCan_Msg.data, 8);
//    HAL_UART_Transmit_DMA(&huart6 , Send_Msg.data, sizeof(Send_Msg.Pack));
}

void SendIMU_classdef::IMU_Update(float *angle, float *gyro)
{
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        // //--- �Ƕ�
        UseIMU.Angle[i] = angle[i]*(180/PI) + 180; //--- ����ת��Ϊ��(0 ~ 360��)
        //--- ���ٶ�
        UseIMU.Gyro[i] = gyro[2-i]*(180/PI);
        //--- �����Ľ��ٶ� Yaw
        UseIMU.OffsetGyro[i] = INS_OffsetGyro[2-i]*(180/PI);
        //--- ���㴦��,�ۼƽǶ�
        if(UseIMU.Angle[i] - UseIMU.Pre_Angle[i] < -180)
        {
            UseIMU.Cnt[i]++;
        }
        else if(UseIMU.Angle[i] - UseIMU.Pre_Angle[i] > 180)
        {
            UseIMU.Cnt[i]--;
        }

        UseIMU.TotalAngle[i] = UseIMU.Angle[i] + UseIMU.Cnt[i] * 360.0f;
        UseIMU.Pre_Angle[i] = UseIMU.Angle[i];
    }
	UseIMU.Gyro[1] = gyro[0]*(180/PI);
    //����������Ƿ������쳣
	if(Imu_Det(INS_angle[0]) && Imu_Det(INS_angle[1]) && Imu_Det(INS_angle[2]))
	{
		while(1){;}
//        Send_Msg.Pack.Yaw_Z = UseIMU.Angle[Yaw];
//        Send_Msg.Pack.Gz = UseIMU.Gyro[Yaw];
//        Send_Msg.Pack.mode = 1;
	}
    else
    {
//        Send_Msg.Pack.Yaw_Z = Last_Yaw;
//        Send_Msg.Pack.Gz = Last_Gz;
//        Send_Msg.Pack.mode = 0; 
    }

//    Last_Yaw = Send_Msg.Pack.Yaw_Z;
//    Last_Gz = Send_Msg.Pack.Gz;
}

void SendIMU_classdef::wait_imuInit(void)
{
    if(UseIMU.Angle[Yaw] != 0 || UseIMU.Angle[Pit] != 0 || UseIMU.Angle[Rol] != 0)
    {
        init_cnt++;
        if(init_cnt > 100) //--- �ȴ�IMU�ȶ� 800+500
        {
            //����Ŀ���ʼ�Ƕ�
            init_mode = false;
        }
    }
}

float SendIMU_classdef::Imu_Det(float eup)
{
  if(0 <= abs(eup) && abs(eup) <= 360)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}