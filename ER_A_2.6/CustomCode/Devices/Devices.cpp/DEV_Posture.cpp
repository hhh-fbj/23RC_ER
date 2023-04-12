//����ȫ����λ
#include "DEV_Posture.h"
#include "System_DataPool.h"

/**
* @Data    
* @brief	ȫ����λ���ݽ���
* @param   PostureBuf : usart6������ʹ��DMA���ܵ�������
						AV_Data �� ActVal_Data_t
* @retval  void
*/
void Posture_Classdef::getMessage(uint8_t *PostureBuf)
{
	static float TF_data[2];
	if(Recv_Msg.Pack.HeadFrame[0]==0x0D && Recv_Msg.Pack.HeadFrame[1]==0x0A &&\
		Recv_Msg.Pack.TailFrame[0]==0x0A && Recv_Msg.Pack.TailFrame[1]==0x0D)
	{   					//��PostureBuf[2]����24�ֽڸ��Ƹ�data
		RAM_Angle[Posture_Z] = Recv_Msg.Pack.ActVal[0];
		RAM_Angle[Posture_X] = Recv_Msg.Pack.ActVal[1];
		RAM_Angle[Posture_Y] = Recv_Msg.Pack.ActVal[2];
		RAM_Value[Posture_X] = Recv_Msg.Pack.ActVal[3];
		RAM_Value[Posture_Y] = Recv_Msg.Pack.ActVal[4];
		RAM_Value[Posture_W] = Recv_Msg.Pack.ActVal[5];
	}
	else return;
	//���㴦��
	if(RAM_Angle[Posture_Z] - Z_LastAngle > 90.0f)
	{
		Z_count--;
	}
	else if(RAM_Angle[Posture_Z] - Z_LastAngle < -90.0f)
	{
		Z_count++;
	}
	Z_LastAngle = RAM_Angle[Posture_Z];
	
	//���������ϴ�
	Last_POS[0] = Final_Value[Posture_X];
	Last_POS[1] = Final_Value[Posture_Y];
	Last_POS[2] = Final_ANGLE;
	//��������
	Final_ANGLE = -(RAM_Angle[Posture_Z] + (360.0 * Z_count));
	Final_Value[Posture_X] = - RAM_Value[Posture_X];
	Final_Value[Posture_Y] = -RAM_Value[Posture_Y];
	
	// ���ݱ仯���

	//tf����ϵ�任
	TF_ANGLE = Final_ANGLE;
//	AV_Data.TF_DATA.X_value = cos(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.X_value	\
//														-sin(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.Y_value;
//	AV_Data.TF_DATA.Y_value = sin(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.X_value	\
//														+cos(radians(AV_Data.RAM_DATA.ZAngle)) 	* AV_Data.Final_data.Y_value;
	
	//TF����任
	POS_TF_Change(Final_Value[Posture_X], Final_Value[Posture_Y], RAM_Angle[Posture_Z],&TF_Value[Posture_X], &TF_Value[Posture_Y]);
	
}

/**
  * @brief   ȫ����λ��λ
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
	HAL_UART_Transmit(&huart6, sendData, 4, 0xffff);
}



//ԭ����ϵ
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
//tf����ϵ�任
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

//����ϵ�任
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
