#ifndef __DEV_POSTURE_H
#define __DEV_POSTURE_H

#include "include.h"
#include "ALGO_Math.h"


#define Post_Buffer_SIZE   28			//数组长度
#define	POS_TF_DATA				0.0f//168.92f			//全场定位到中心点距离，单位mm 


enum XYZ_Posture
{
   Posture_X=0,Posture_Y=1,
   Posture_Z=2,Posture_W=2
};

#pragma pack(1)
typedef struct
{
	uint8_t  HeadFrame[2];
	float  ActVal[6];
	uint8_t TailFrame[2];
}Posture_t;
#pragma pack()
	
typedef union
{
    uint8_t  data[Post_Buffer_SIZE];
    Posture_t Pack;
}PostureRecvMsg_u;


class Posture_Classdef
{
private: 
    float RAM_Angle[3],RAM_Value[3];
    float TF_Value[3], TF_ANGLE;
    float Z_LastAngle;
    int32_t Z_count;  //圈数
    float Vary_Limit;
    //原坐标系
//    float POS_X(void);
//    float POS_Y(void);
    
    //tf变换后的坐标系
    float POS_TF_ANGLE(void);
    float cos_cal(float angle);
    void POS_TF_Change(float RAM_X,float RAM_Y,float RAM_ANGLE,float *TF_X,float *TF_Y);
    void TF_Change(float X,float Y,float angle,float *tf_X,float *tf_Y);
public: 
    Posture_Classdef();
    float Final_Value[3], Final_ANGLE;
    bool error;
		float TenMi;
		float time_long[2];
    float Last_POS[3];
		float Office_Value[3],Other_Value[3];
    PostureRecvMsg_u Recv_Msg;
    void getMessage(uint8_t *PostureBuf);
    void POSTure_Init(void);
    void Devices_Posture_Reset(void);//全场定位复位

    //tf变换后的坐标系
    float POS_TF_X(void);
    float POS_TF_Y(void);
    float POS_X(void);
    float POS_Y(void);
    float POS_W(void);
};

#endif


