//东大全场定位
#include "DEV_Posture.h"
#include "System_DataPool.h"

#define SPEED_LIMIT 4000 //单位：rpm
#define WHEEL_R 106 //单位：mm
#define DATA_RATE 200 //单位：帧/s


Posture_Classdef::Posture_Classdef()
{
	Vary_Limit = SPEED_LIMIT/60 * WHEEL_R / DATA_RATE;
	// VaryAngle_Limit = SPEED_LIMIT/60 * WHEEL_R / DATA_RATE
}

/**  
* @Data    
* @brief	全场定位数据解码
* @param   PostureBuf : usart6串口所使用DMA接受到的数据
						AV_Data ： ActVal_Data_t
* @retval  void
*/
void Posture_Classdef::getMessage(uint8_t *PostureBuf)
{
	static float TF_data[2];
	if(Recv_Msg.Pack.HeadFrame[0]==0x0D && Recv_Msg.Pack.HeadFrame[1]==0x0A &&\
		Recv_Msg.Pack.TailFrame[0]==0x0A && Recv_Msg.Pack.TailFrame[1]==0x0D)
	{   //将PostureBuf[2]往后24字节复制给data
		RAM_Angle[Posture_Z] = Recv_Msg.Pack.ActVal[0];
		RAM_Angle[Posture_X] = Recv_Msg.Pack.ActVal[1];
		RAM_Angle[Posture_Y] = Recv_Msg.Pack.ActVal[2];
		RAM_Value[Posture_X] = Recv_Msg.Pack.ActVal[3];
		RAM_Value[Posture_Y] = Recv_Msg.Pack.ActVal[4];
		RAM_Value[Posture_W] = Recv_Msg.Pack.ActVal[5];
		DevicesMonitor.Update(Frame_CHAS_POSTURE);
	}
	else return;
	//过零处理
	if(RAM_Angle[Posture_Z] - Z_LastAngle > 90.0f)
	{
		Z_count--;
	}
	else if(RAM_Angle[Posture_Z] - Z_LastAngle < -90.0f)
	{
		Z_count++;
	}
	Z_LastAngle = -RAM_Angle[Posture_Z];
	
	//最终数据上次
	Last_POS[0] = Final_Value[Posture_X];
	Last_POS[1] = Final_Value[Posture_Y];
	Last_POS[2] = Final_ANGLE;
	//最终数据
	Final_ANGLE = -(RAM_Angle[Posture_Z] + (360.0 * Z_count));
	Final_Value[Posture_X] = - RAM_Value[Posture_X];
	Final_Value[Posture_Y] = -RAM_Value[Posture_Y];
	
	// 数据变化检测
	if(abs(Final_Value[Posture_X] - Last_POS[0]) > Vary_Limit)
	{
		error = true;
	}
	else if(abs(Final_Value[Posture_Y] - Last_POS[1]) > Vary_Limit)
	{
		error = true;
	}
	else
	{
		error = false;
	}
	// if(Final_ANGLE - Last_POS[2] > Vary_Limit)
	// {
		
	// }

	//tf坐标系变换
	TF_ANGLE = Final_ANGLE;
//	AV_Data.TF_DATA.X_value = cos(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.X_value	\
//														-sin(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.Y_value;
//	AV_Data.TF_DATA.Y_value = sin(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.X_value	\
//														+cos(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.Y_value;
	
	//TF坐标变换
	POS_TF_Change(Final_Value[Posture_X], Final_Value[Posture_Y], RAM_Angle[Posture_Z],&TF_Value[Posture_X], &TF_Value[Posture_Y]);
	
}

/**
  * @brief   全场定位复位
  * @param   void
  * @retval  void
 */
void Posture_Classdef::Devices_Posture_Reset(void)
{
	uint8_t  sendData[4];
	
	sendData[0] =	'A';
	sendData[1] =	'C';
	sendData[2] =	'T';
	sendData[3] =	'0';
	// HAL_UART_Transmit_DMA(&huart6, sendData, 4);
	HAL_UART_Transmit(&huart6, sendData, 4, 0xffff);
}



//原坐标系
// #define TF_POS
float Posture_Classdef::POS_X(void)
{
	#ifdef TF_POS
	return TF_Value[Posture_X];
	#else
	return Final_Value[Posture_X];
	#endif
}
float Posture_Classdef::POS_Y(void)
{
	#ifdef TF_POS
	return TF_Value[Posture_Y];
	#else
	return Final_Value[Posture_Y];
	#endif
}
float Posture_Classdef::POS_W(void)
{
	// return RAM_Angle[Posture_W];
	return Final_ANGLE;
}
//tf坐标系变换
float Posture_Classdef::POS_TF_X(void)
{
	return TF_Value[Posture_X];
}
float Posture_Classdef::POS_TF_Y(void)
{
	return TF_Value[Posture_Y];
}
float Posture_Classdef::POS_TF_ANGLE(void)
{
	return TF_ANGLE;
}

float Posture_Classdef::cos_cal(float angle)
{
	return cos(radians(angle));
}

void Posture_Classdef::TF_Change(float X,float Y,float angle,float *tf_X,float *tf_Y)
{
	float TF_data[2] = {NULL};
	if(angle>=0.0f && angle < 90.0f)
	{
		TF_data[1] = (cos_cal(90-angle)/(sq(cos_cal(90-angle))+sq(cos_cal(angle))))*X + (cos_cal(angle)/(sq(cos(90-angle))+sq(cos_cal(angle))))*Y;
		TF_data[0] = (cos_cal(angle)*TF_data[1] - Y)/(cos_cal(90-angle));
	}
	else if(angle>=90.0f && angle < 180.0f)
	{
		TF_data[1] = (cos_cal(angle-90)/(sq(cos_cal(angle-90))+sq(cos_cal(180-angle))))*X - (cos_cal(180-angle)/(sq(cos(angle-90))+sq(cos_cal(180-angle))))*Y;
		TF_data[0] = (cos_cal(180-angle)*TF_data[1] - Y)/(cos_cal(angle-90));
	}
	else if(angle>=-90 && angle<0)
	{
		TF_data[1] =  - (cos_cal(90-angle)/(sq(cos(90-angle))+sq(cos_cal(angle))))*X + (cos_cal(angle)/(sq(cos_cal(90-angle))+sq(cos_cal(angle))))*Y;
		TF_data[0] = (cos_cal(90-angle)*TF_data[1] + X)/(cos_cal(angle));
	}
	else if(angle>=-180 && angle<-90)
	{
		TF_data[1] =  - (cos_cal(angle-90)/(sq(cos(angle-90))+sq(cos_cal(180-angle))))*X + (cos_cal(180-angle)/(sq(cos_cal(angle-90))-sq(cos_cal(180-angle))))*Y;
		TF_data[0] = (cos_cal(angle-90)*TF_data[1] + X)/(cos_cal(180-angle));
	}
	
	*tf_X = TF_data[0];
	*tf_Y = TF_data[1];

}

//坐标系变换
void Posture_Classdef::POS_TF_Change(float RAM_X,float RAM_Y,float RAM_ANGLE,float *TF_X,float *TF_Y)
{
	if(RAM_ANGLE >= 0.0f && RAM_ANGLE < 90.0f)
	{
		*TF_X = RAM_X + POS_TF_DATA*sin(radians(RAM_ANGLE));
		*TF_Y = RAM_Y + POS_TF_DATA*cos(radians(RAM_ANGLE)) - POS_TF_DATA;
	}
	else if(RAM_ANGLE < 0.0f && RAM_ANGLE >= -90.0f)
	{
		*TF_X = RAM_X - POS_TF_DATA*sin(radians(-RAM_ANGLE));
		*TF_Y = RAM_Y + POS_TF_DATA*cos(radians(-RAM_ANGLE)) - POS_TF_DATA;
	}
}
