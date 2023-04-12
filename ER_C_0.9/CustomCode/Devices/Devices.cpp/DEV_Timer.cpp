#include "DEV_Timer.h"
/* Includes ------------------------------------------------------------------*/


volatile uint32_t SystemTimerCnt;

struct timer_manage_obj_t
{
	TIM_HandleTypeDef*	htim_x;
	EDelay_src	delay_ms_src;
}Timer_Manager;

void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src)
{
	/* Check the parameters */
	assert_param(htim != NULL);
	
	Timer_Manager.htim_x = htim;
	Timer_Manager.delay_ms_src = src;
	
//	htim4.Instance->SR = 0;

	HAL_TIM_Base_Start_IT(Timer_Manager.htim_x);
	HAL_TIM_Base_Start(Timer_Manager.htim_x);
//   __HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF);
//	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
//	htim4.Instance->SR = 0;
//  if(HAL_TIM_Base_Start_IT(Timer_Manager.htim_x)!=HAL_OK)
////      Error_Handler();
//  if(HAL_TIM_Base_Start(Timer_Manager.htim_x)!= HAL_OK)
////      Error_Handler();
}

uint32_t Get_SystemTimer(void)
{
	return Timer_Manager.htim_x->Instance->CNT + SystemTimerCnt * 0xffff;
}

void Update_SystemTick(void)
{
	SystemTimerCnt++;
}


void delay_us_nos(uint32_t cnt)
{
	uint32_t temp = cnt  + Get_SystemTimer();

	while(temp >= Get_SystemTimer());
}

void delay_ms_nos(uint32_t cnt)
{
	if(Timer_Manager.htim_x != NULL && Timer_Manager.delay_ms_src == USE_MODULE_DELAY)
	{
		uint32_t temp = cnt * 1000 + Get_SystemTimer();
		while(temp >= Get_SystemTimer());
	}
	else
		HAL_Delay(cnt);
}

