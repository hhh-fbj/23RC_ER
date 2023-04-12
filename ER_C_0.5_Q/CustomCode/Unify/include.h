#ifndef __INCLUDE_H
#define __INCLUDE_H

/* Includes Files ---------------------------------------------------------------------------*/
//启动头文件
#include "confing.h"

//DEV 代码头文件 3
#ifdef _DEV

#ifdef _DR16
#include "DEV_DR16.h"
#endif

#ifdef _BRTENCODER
#include "DEV_BrtEncoder.h"
#endif

#ifdef _MOTOR
#include "DEV_Motor.h"
#endif

#endif

//BSP 代码头文件 7
#ifdef _BSP

#ifdef _UART
#include "BSP_UART.h"
#endif

#ifdef _CAN
#include "BSP_CAN.h"
#endif

#ifdef _FLASH
#include "BSP_FLASH.h"
#endif

#ifdef _FLASHSTORE
#include "BSP_FlashStore.h"
#endif

#ifdef _PVD
#include "BSP_PVD.h"
#endif

#ifdef _ADC
#include "BSP_ADC.h"
#endif

#ifdef _SPI
#include "BSP_SPI.h"
#endif

#endif

//ALGO 代码头文件 5
#ifdef _ALGO

#ifdef _PID
#include "ALGO_PID.h"
#endif

#ifdef _FILTER
#include "ALGO_Filter.h"
#endif

#ifdef _RAMP
#include "ALGO_Ramp.h"
#endif

#ifdef _MATH
#include "ALGO_Math.h"
#endif

#ifdef _KALMAN
#include "ALGO_Kalman.h"
#endif


#endif

#endif
/* File Of End ------------------------------------------------------------------------------*/