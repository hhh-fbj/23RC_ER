#include "Power_Limit.h"
#include "System_DataPool.h"

PowerLimit_t chassis_powerLimit;

int16_t Power_Buffer;    //功率缓冲.

void  PowerLimit_Calculate(PowerLimit_t *powerlimit) {

	Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;   //底盘功率缓冲热量 
	//Power_Buffer = ext_power_heat_data.chassis_power_buffer;   //底盘功率缓冲热量 

	// if (JudgeSystem.OffLineFlag == 1)
	// {
	// 	Power_Buffer = 120;  // 掉线则设置最大功率，非无限功率.
	// }



	int16_t powerBuffErr = 60 - Power_Buffer; //用掉的缓冲能量。

	powerlimit->powerBuffRatio = 0;
	powerlimit->powerBuffRatio = (float)Power_Buffer / 75.0; //用掉的缓冲能量。    //---
	powerlimit->powerBuffRatio *= powerlimit->powerBuffRatio;   //立方的关系。   
	//powerlimit->powerBuffRatio *= powerlimit->powerBuffRatio;   //立方的关系。   

	if (powerBuffErr > 0)   //没有用到缓冲功率则不做处理，即未超功率.
	{
		powerlimit->SumCurrent_Out = powerlimit->SumCurrent_IN *powerlimit->powerBuffRatio;
	}

}


void PowerLimit(PowerLimit_t *powerlimit, int16_t * wheelCurrent, int16_t amount) 
{
	float coe[4] = { 0.0 };   //系数	动态分配数组内存。
	int i = 0;
	powerlimit->SumCurrent_IN = powerlimit->SumCurrent_Out = 0;

	//计算总电流
	for (i = 0; i < amount; i++)
	{
		powerlimit->SumCurrent_IN += abs(wheelCurrent[i]);
	}
	powerlimit->SumCurrent_Out = powerlimit->SumCurrent_IN; //不处理时为原来的值。

	//计算每个电流占总电流的百分比
	for (i = 0; i < amount; i++)
	{
		coe[i] = ((float)(wheelCurrent[i])) / ((float)(powerlimit->SumCurrent_IN));
	}

	//计算此时限制功率后的最大电流
	PowerLimit_Calculate(powerlimit);//内部指针的方式修改 SumCurrent_Out

	//按照百分比分配最大电流
	for (i = 0; i < amount; i++)
	{
		wheelCurrent[i] = (int16_t)(powerlimit->SumCurrent_Out  * coe[i]);
	}
}

