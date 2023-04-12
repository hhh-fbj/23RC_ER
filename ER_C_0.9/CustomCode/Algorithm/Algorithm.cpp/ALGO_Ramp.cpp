/**
 ------------------------------------------------------------------------------
 * @file    Ramp.cpp
 * @author  Shake
 * @brief   ����б�´�������
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (cpp) 2021
 ------------------------------------------------------------------------------
 */

#include "ALGO_Ramp.h"
/* Includes ------------------------------------------------------------------*/
#include "ALGO_Math.h"

/**
  * @brief  б�º�������
  * @param	б�º����ṹ��
  * @retval None
  */
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp)
{

	SpeedRamp->count += SpeedRamp->rate;
	Constrain(&SpeedRamp->count, SpeedRamp->mincount, SpeedRamp->maxcount);

	return SpeedRamp->count;
}

/**
  * @brief  б�¼���ֵ����
  * @param	б�º����ṹ��
  * @retval None
  */
void CountReset(SpeedRamp_t *SpeedRamp)
{

	if (abs(SpeedRamp->count) < abs(SpeedRamp->rate))
	{
		SpeedRamp->count = 0;
	}
	else
	{

		SpeedRamp->count -= SpeedRamp->count * 0.2;
	}
	//if (SpeedRamp->count > abs(SpeedRamp->rate))
	//{
	//	SpeedRamp->count -= abs(SpeedRamp->rate);
	//}
	//else if (SpeedRamp->count < -abs(SpeedRamp->rate))
	//{
	//	SpeedRamp->count += abs(SpeedRamp->rate);
	//}
	//else
	//{
	//
	//}
}

/**
  * @brief      б�º���,ʹĿ�����ֵ������������ֵ
  * @param[in]  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval     ��ǰ���
  * @attention  
  */
float RAMP_Output(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;

	if (buffer > 0)
	{
		if (buffer > ramp)
		{
			now += ramp;
		}
		else
		{
			now += buffer;
		}
	}
	else
	{
		if (buffer < -ramp)
		{
			now += -ramp;
		}
		else
		{
			now += buffer;
		}
	}

	return now;
}
