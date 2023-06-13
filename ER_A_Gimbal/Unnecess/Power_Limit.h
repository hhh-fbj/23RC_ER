#ifndef __Power_Limit_H
#define __Power_Limit_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct
{
	int32_t SumCurrent_IN;       //输入的电流总和
	int32_t SumCurrent_Out;     //最后计算电流总和

	//float LimitPower;        //当前限制的功率大小 单位W    

	float powerBuffRatio;

	//裁判系统变量绑定
	uint16_t  *Real_chassis_volt;                   //底盘输出电压 单位 毫伏
	uint16_t *Real_chassis_current;                //底盘输出电流 单位 毫安
	float *Real_chassis_power;
}PowerLimit_t;

extern PowerLimit_t chassis_powerLimit;

void PowerLimit(PowerLimit_t *powerlimit, int16_t * wheelCurrent, int16_t amount);

#ifdef  __cplusplus
}
#endif

#endif
