/**
  ==============================================================================
						    How to use this library 
  ==============================================================================
    @note
			- ��ͨ�˲� 
				-# LowPassFilter LF(trust);  
				-# ����LF.f(num) ���� LF << (in)  LF >> out �����ʱ�������

			- ��ֵ�˲� 
				-# MedianFilter<Length> MDF;  
				-# ����MDF.f(num) ���� MDF << (in) MDF >> out �����ʱ������� 

			- ��ֵ�˲� 
				-# MeanFilter<Length> MF;  
				-# ����MF.f(num) ����MF << (in)  MF >> out �����ʱ������� 

  	@warning 
			- ��ͨ�˲�����trust (0,1) ������ע�ⳬ��������   ��ֵ�˲� ��ֵ�˲�(����[1,100])
			- Standard C++11 required! 
  
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

#include "ALGO_Filter.h"
 /* Includes ------------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/* LowPassFilter */
void LowPassFilter::in(float num)							
{
	last_num = now_num;
	now_num = num;
}

float LowPassFilter::out()							
{
	return (last_num + last_num * (1 - Trust));
}


void LowPassFilter::operator <<(const float& num)			
{
	in(num);
}

void LowPassFilter::operator >>(float& num)
{
	num = out();
}

float LowPassFilter::f(float num)						
{
	in(num);
	return (out());
}


/* IIRLowPassFilter */
void IIRLowPassFilter::in(float num)							
{
	last_out_num = out_num;
	in_num = num;
}

float IIRLowPassFilter::out()							
{
	out_num = (last_out_num + Trust * (in_num - last_out_num));
	return out_num;
}

float IIRLowPassFilter::f(float num)						
{
	in(num);
	return (out());
}