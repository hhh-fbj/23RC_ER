#ifndef __DEV_RTC_H
#define __DEV_RTC_H

#include "stdint.h"
#include "rtc.h"
#include "stm32f4xx_hal.h"




class RTC_Classdef
{
public:
    RTC_Classdef();

    RTC_TimeTypeDef GetTime;
    void RTC_SetTime(uint8_t hour, uint8_t minute, uint8_t second);
};

#endif