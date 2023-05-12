#include "DEV_DT35.h"			
#include "System_DataPool.h"



/**
  * @brief   DT35初始化函数
  * @param   NONE
  * @retval  
  */
DT35_classdef::DT35_classdef(void)
{
	
}



/**
  * @brief   DT35获取距离值函数
  * @param   NONE
  * @retval  
  */
void DT35_classdef::DT35_GetDistance(void)
{
	static uint32_t sum1,sum2,sum3,sum4;
	//static是0
	sum1 = 0;
	sum2 = 0;
	sum3 = 0;
	
	for(int i=0;i<DT35_DMA_SIZE;i++)
	{
		if(i%3==0)
		{
			sum1+=adc_buf[i];
		}
		else if(i%2==0)
		{
			sum2+=adc_buf[i];
		}
		else
		{
			sum3+=adc_buf[i];
		}
	}

	//这样需要注意，不然有精度丢失问题
	Vrefint_ADC = (float)(sum1*1.0f)/(DT35_FILTER_N*1.0f);
	//电压误差系数
	Veef_coeff=((float)VOLTAGEtoRAITO(1.21f)/(Vrefint_ADC));

	In12_ADC = (float)(sum2*1.0f)/(DT35_FILTER_N*1.0f)*Veef_coeff;
	In13_ADC = (float)(sum3*1.0f)/(DT35_FILTER_N*1.0f)*Veef_coeff;
}




