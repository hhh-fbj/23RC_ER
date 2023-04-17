/**
 * @file PVD.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __BSP_PVD_H
#define __BSP_PVD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

void PVD_Config(void);
void HAL_PWR_PVDCallback(void);

#ifdef __cplusplus
}
#endif

#endif

