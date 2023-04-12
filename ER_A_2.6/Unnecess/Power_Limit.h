#ifndef __Power_Limit_H
#define __Power_Limit_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct
{
	int32_t SumCurrent_IN;       //����ĵ����ܺ�
	int32_t SumCurrent_Out;     //����������ܺ�

	//float LimitPower;        //��ǰ���ƵĹ��ʴ�С ��λW    

	float powerBuffRatio;

	//����ϵͳ������
	uint16_t  *Real_chassis_volt;                   //���������ѹ ��λ ����
	uint16_t *Real_chassis_current;                //����������� ��λ ����
	float *Real_chassis_power;
}PowerLimit_t;

extern PowerLimit_t chassis_powerLimit;

void PowerLimit(PowerLimit_t *powerlimit, int16_t * wheelCurrent, int16_t amount);

#ifdef  __cplusplus
}
#endif

#endif
