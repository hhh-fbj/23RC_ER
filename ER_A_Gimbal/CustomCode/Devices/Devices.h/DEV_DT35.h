#ifndef __DEV_DT35_H
#define __DEV_DT35_H

#include "include.h"
#define switch_voltage    		(3.3/4096)				/* ADC值转换电压 系数*/
#define voltage_change 	 			(14.7/4.7)				/* DT35电阻转实际电压  系数 (10k+4.7k)/4.7k*/
#define voltage_calibrator	 	(3.079/2.59)			/* DT35误差补偿 系数  用内部电压误差系数代替*/	
/* 转换的系数值  电压->距离*/
#define DT35_coefficient    ((DT35_distanceMax-DT35_distanceMin)/DT35_MaxVoltage)

#define temp_change(x)				((x-0.76f)/400.0f+25.0f)		/* 电压转成温度值*/

#define ADC_RATIO				4096.0f	//ADC分辨率
#define ADC_MAX_VOLTAGE			3.3f	//ADC最大电压

#define RAITOtoVOLTAGE(x)		((x)*(ADC_MAX_VOLTAGE/ADC_RATIO))		//分辨率转电压
#define VOLTAGEtoRAITO(x)		((x)*(ADC_RATIO/ADC_MAX_VOLTAGE))


/* 内部电压系数*/
#define VREFINT_change 			(4096* 1.21/3.3)

#define ADC_DMA_size       20   /* ADC1_DMA数组的大小*/

#define DT35_NUMBERS				3																//DT35的数量
#define DT35_FILTER_N				100															//DT35 DMA过滤个数
#define DT35_DMA_SIZE  			(DT35_NUMBERS*DT35_FILTER_N)  //单个DMA的个数

#define sq(x) 					((x)*(x))
#define cube(x)					((x)*(x)*(x))
#define biquadrate(x)		((x)*(x)*(x)*(x))

#define DT35_FUN_INIT {	\
DT35_Init,	\
DT35_GetDistance,	\
}


class DT35_classdef
{
private:
	float Veef_coeff;//内部电压系数
	uint16_t Vrefint_ADC,In12_ADC,In13_ADC;
public: 
	DT35_classdef();
	uint16_t adc_buf[DT35_DMA_SIZE];

	//DT35初始化函数
	void DT35_Init(void);
	//DT35获取距离值函数
	void DT35_GetDistance(void);
	float Dt35_DistanceCal(float vel,float max_vel,float dis_min,float dis_max);
	float DT35_DistanceCal(float *cal_buf,float adc);
};







#endif


