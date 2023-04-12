#ifndef __DEV_TIMER_H
#define __DEV_TIMER_H

#ifdef  __cplusplus
extern "C"{
#endif

#include <stm32f4xx_hal.h>
#include <stdint.h>

//#define microsecond()    Get_SystemTimer()

/* Private type --------------------------------------------------------------*/
typedef struct{
  uint32_t last_time;	/*!< Last recorded real time from systick*/
  float dt;				/*!< Differentiation of real time*/
}TimeStamp;

typedef enum 
{
	USE_MODULE_DELAY = 1,	/*!< Use module function to implement delay_ms_nos()*/
	USE_HAL_DELAY			/*!< Use HAL_Delay() for delay_ms_nos()*/
}EDelay_src;


extern void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src);
void Update_SystemTick(void);
uint32_t Get_SystemTimer(void);
void delay_ms_nos(uint32_t cnt);
void delay_us_nos(uint32_t cnt);

#ifdef  __cplusplus
}
#endif

#endif