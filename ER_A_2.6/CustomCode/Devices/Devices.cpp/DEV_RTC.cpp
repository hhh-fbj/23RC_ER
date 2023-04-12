#include "DEV_RTC.h"

RTC_Classdef::RTC_Classdef()
{
    RTC_SetTime(11,59,50);
}

void RTC_Classdef::RTC_SetTime(uint8_t hour, uint8_t minute, uint8_t second)
{
  /*##-2- Configure the Time #################################################*/
  GetTime.Hours = hour;
  GetTime.Minutes = minute;
  GetTime.Seconds = second;
//  GetTime.TimeFormat = RTC_HOURFORMAT12_AM;
  GetTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  GetTime.StoreOperation = RTC_STOREOPERATION_RESET;
	
//	HAL_RTC_SetTime(&hrtc, &GetTime, RTC_FORMAT_BIN) != HAL_OK;

//   if (HAL_RTC_SetTime(&hrtc, &GetTime, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    Error_Handler();
//  }
}
