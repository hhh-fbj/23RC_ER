/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIMU_Heat__B5_Pin GPIO_PIN_5
#define AIMU_Heat__B5_GPIO_Port GPIOB
#define Vision_TX_G14_Pin GPIO_PIN_14
#define Vision_TX_G14_GPIO_Port GPIOG
#define RMBoard_LASER_G13_Pin GPIO_PIN_13
#define RMBoard_LASER_G13_GPIO_Port GPIOG
#define RM_OLED_B4_Pin GPIO_PIN_4
#define RM_OLED_B4_GPIO_Port GPIOB
#define RM_OLED_B3_Pin GPIO_PIN_3
#define RM_OLED_B3_GPIO_Port GPIOB
#define OLED_DC_B9_Pin GPIO_PIN_9
#define OLED_DC_B9_GPIO_Port GPIOB
#define DR16_RX_B7_Pin GPIO_PIN_7
#define DR16_RX_B7_GPIO_Port GPIOB
#define DR16_OFF_B6_Pin GPIO_PIN_6
#define DR16_OFF_B6_GPIO_Port GPIOB
#define Vision_RX_G9_Pin GPIO_PIN_9
#define Vision_RX_G9_GPIO_Port GPIOG
#define LED8_G8_Pin GPIO_PIN_8
#define LED8_G8_GPIO_Port GPIOG
#define LED7_G7_Pin GPIO_PIN_7
#define LED7_G7_GPIO_Port GPIOG
#define LED6_G6_Pin GPIO_PIN_6
#define LED6_G6_GPIO_Port GPIOG
#define OnBorad_AIMU_F7_Pin GPIO_PIN_7
#define OnBorad_AIMU_F7_GPIO_Port GPIOF
#define AIMU_Heat_Pin GPIO_PIN_6
#define AIMU_Heat_GPIO_Port GPIOF
#define Device_H12_Pin GPIO_PIN_12
#define Device_H12_GPIO_Port GPIOH
#define LED5_G5_Pin GPIO_PIN_5
#define LED5_G5_GPIO_Port GPIOG
#define LED4_G4_Pin GPIO_PIN_4
#define LED4_G4_GPIO_Port GPIOG
#define LED3_G3_Pin GPIO_PIN_3
#define LED3_G3_GPIO_Port GPIOG
#define OnBorad_AIMU_F9_Pin GPIO_PIN_9
#define OnBorad_AIMU_F9_GPIO_Port GPIOF
#define OnBorad_AIMU_F8_Pin GPIO_PIN_8
#define OnBorad_AIMU_F8_GPIO_Port GPIOF
#define LED2_G2_Pin GPIO_PIN_2
#define LED2_G2_GPIO_Port GPIOG
#define LED1_G1_Pin GPIO_PIN_1
#define LED1_G1_GPIO_Port GPIOG
#define RMBorad_Buzzer_H6_Pin GPIO_PIN_6
#define RMBorad_Buzzer_H6_GPIO_Port GPIOH
#define Sensor_D12_Pin GPIO_PIN_12
#define Sensor_D12_GPIO_Port GPIOD
#define DJI_OLED_A6_Pin GPIO_PIN_6
#define DJI_OLED_A6_GPIO_Port GPIOA
#define Debug_TX_E8_Pin GPIO_PIN_8
#define Debug_TX_E8_GPIO_Port GPIOE
#define RM_Referee_RX_D9_Pin GPIO_PIN_9
#define RM_Referee_RX_D9_GPIO_Port GPIOD
#define RM_Referee_TX_D8_Pin GPIO_PIN_8
#define RM_Referee_TX_D8_GPIO_Port GPIOD
#define RM_OLED_B7_Pin GPIO_PIN_7
#define RM_OLED_B7_GPIO_Port GPIOA
#define Debug_RX_E7_Pin GPIO_PIN_7
#define Debug_RX_E7_GPIO_Port GPIOE
#define SD_EXIT_Pin GPIO_PIN_15
#define SD_EXIT_GPIO_Port GPIOE
#define OLED_RST_B10_Pin GPIO_PIN_10
#define OLED_RST_B10_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
