#include "DEV_Buzzer.h"
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "System_DataPool.h"



void Buzzer_Classdef::Buzzer_Switch(bool State)
{
	if (State == ON)
	{
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
		__HAL_TIM_SET_AUTORELOAD(&htim12, 1000000 / 500);
	}
	else
	{
		__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
		// HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	}
}

/**
  * @brief  初始化提示音
  * @param  None
  * @retval None
  */
void Buzzer_Classdef::Error_PromptTone(void)
{
	if (error!=0)
	{
		Error_TickTIME++;
		switch (Error_TickTIME)
		{
		case 1:
			Buzzer_Switch(ON);
		break;

		case 2:
			Buzzer_Switch(OFF);
			break;

		default:
			Error_TickTIME = 0;
			break;
		}
	}
	else
	{
		Buzzer_Switch(OFF);
	}
}

/**
  * @brief  初始化提示音
  * @param  None
  * @retval None
  */
void Buzzer_Classdef::Initialize_PromptTone(void)
{
	if (0)
	{
		Switch_Flag = true;
		Init_TickTIME++;
		switch (Init_TickTIME)
		{
		case 1:
			Buzzer_Switch(ON);
			All_LED(ON);
			break;

		case 2:
			Buzzer_Switch(OFF);
			All_LED(OFF);
			break;

		default:
			Init_TickTIME = 0;
			break;
		}
	}
	else if (0)
	{
		//Init_TickTIME = 2;
		Buzzer_Switch(OFF);
		All_LED(OFF);
		// SuperMario_Music();
		Switch_Flag = false;
	}
}

/**
  * @brief  马里奥音乐播放
  * @param  None
  * @retval None
  */
void Buzzer_Classdef::SuperMario_Music(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	for (int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++)
	{
		if (SuperMario[i].note == 0)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
			HAL_Delay(SuperMario[i].time);
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 1000000 / SuperMario[i].note);
			HAL_Delay(SuperMario[i].time);
		}
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}

/**
  * @brief  全部LED灯开关
  * @param  State
  * @retval None
  */
void Buzzer_Classdef::All_LED(bool State)
{
	switch((uint8_t)State)
	{
	case ON:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
		break;

	case OFF:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}

/**
  * @brief  正常运行流水灯
  * @param  None
  * @retval None
  */
void Buzzer_Classdef::Waterfall_LED(void)
{
	static uint8_t LED_Port = LED_1; //--- 引脚
    static uint8_t Dir = true;   //--- 方向
	if (LED_Port == LED_8)
	{
		Dir = false;
	}
	else if (LED_Port == LED_1)
	{
		Dir = true;
	}

	(Dir == true) ? (LED_Port++) : (LED_Port--);

    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1 << LED_Port);
}
