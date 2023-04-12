#include "DEV_Buzzer.h"
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "System_DataPool.h"


/**
 * @brief      设置蜂鸣器响铃类型
 * @param[in]  ringtype
 * @retval     None
 */
void Buzzer_Classdef::Set_RingType(int8_t ringtype)
{
	Ring_Type = ringtype;
}

uint16_t buzzer_val = 300;
void Buzzer_Classdef::Process()
{
	static uint8_t state_cnt = 0; // BB音间隔
	static uint8_t time_cnt = 0;  // BB音频次计数
	static int8_t last_times = 0; // 记录是否有状态切换

	if(Ring_Type != last_times)
	{
		time_cnt = Ring_Type;
		state_cnt = 0;
	}

	switch(Ring_Type)
	{
	case Ring_Stop:	//--- 停止
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
		state_cnt = 0;
		break;

	case Ring_ContLong: //--- 持续响铃 长间隔
		state_cnt++;
		if((state_cnt%3)==0 && (state_cnt<time_cnt*3))
		{
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			__HAL_TIM_SET_AUTORELOAD(&htim4, buzzer_val);
		}
		else if((state_cnt%=5)==0 && (state_cnt<time_cnt*3))
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, 0);
		}
		break;

	case Ring_ContShort: //--- 持续响铃 短间隔
		state_cnt++;
		if((state_cnt%=2)==0 && (state_cnt<time_cnt*2))
		{
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			__HAL_TIM_SET_AUTORELOAD(&htim4, buzzer_val);
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, 0);
		}
		break;

	default: //--- 响铃n声
    	state_cnt++;
		if((state_cnt%=2)==0 && (state_cnt<time_cnt*2))
		{
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			__HAL_TIM_SET_AUTORELOAD(&htim4, buzzer_val);
			time_cnt--;
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, 0);
		}
		break;
	}

	last_times = Ring_Type;
}


/**
  * @brief  马里奥音乐播放
  * @param  None
  * @retval None
  */
void Buzzer_Classdef::SuperMario_Music(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	for (int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++)
	{
		if (SuperMario[i].note == 0)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, 0);
			HAL_Delay(SuperMario[i].time);
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, 100000 / SuperMario[i].note);
			HAL_Delay(SuperMario[i].time);
		}
	}
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
