#ifndef __SYS_DATAPOOL_H
#define __SYS_DATAPOOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <cmsis_os.h>
/* Middlewares & Drivers Support */
#include <stm32f4xx.h>

#include "include.h"

#include "usart.h"

#include "Wolf_Infantry.h"

#include "APP_DR16.h"
#include "APP_Devices.h"
#include "APP_Chassis.h"
#include "APP_Auto.h"

#include "DEV_Timer.h"
#include "DEV_AIMU.h"
#include "DEV_CIMU.h"
#include "DEV_Buzzer.h"
#include "DEV_UpComputer.h"
#include "DEV_SD.h"
#include "DEV_DT35.h"
#include "DEV_DRF1609H.h"
#include "DEV_L1TTL.h"

/* Macro Definitions ---------------------------------------------------------*/
#define TASK_STACK_SIZE_64    64
#define TASK_STACK_SIZE_128   128
#define TASK_STACK_SIZE_256   256
#define TASK_STACK_SIZE_512   512
#define TASK_STACK_SIZE_1024  1024

#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8


/* Exported types ------------------------------------------------------------*/
enum PID_calctype_e
{
  PID_Outer = 0,
  PID_Inner,
  PID_Withiner
};

enum Switchtype_e
{
    OFF = 0,
    ON
};

typedef struct
{
  uint8_t  port_num;
  int16_t  len;
  void*    address;
}USART_COB;

/* RTOS Resources ------------------------------------------------------------*/
//extern QueueHandle_t xQueueCan1Receive_Handle;
//extern QueueHandle_t xQueueCan2Receive_Handle;
//extern QueueHandle_t xQueueCan1_TransmitHandle;
//extern QueueHandle_t Referee_QueueHandle;

extern QueueHandle_t Can1Recv_QueueHandle;
extern QueueHandle_t Can2Recv_QueueHandle;
extern QueueHandle_t Can1Send_QueueHandle;
extern QueueHandle_t Referee_QueueHandle;
extern QueueHandle_t Vision_QueueHandle;

extern TaskHandle_t RefereeRecv_TaskHandle;
extern TaskHandle_t Visionrecv_TaskHandle;

/* HAL Handlers --------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

#define USART1_RX_BUFFER_SIZE 32
#define USART2_RX_BUFFER_SIZE 64
#define USART3_RX_BUFFER_SIZE 256
#define USART4_RX_BUFFER_SIZE 256
#define USART5_RX_BUFFER_SIZE 512
#define USART6_RX_BUFFER_SIZE 1024

extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
extern uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];  



/* Global Objects ------------------------------------------------------------*/
extern DR16_Classdef DR16;
extern CTRL_DR16_classdef CTRL_DR16;
extern DevicesMonitor_classdef DevicesMonitor;
extern Chassis_classdef Chassis;
extern Auto_classdef Auto;
extern DRF1609H_classdef DRF1609H;
//extern L1TTL_classdef L1TTL;

extern AIMU_Data_Classdef imu_Export;
extern Buzzer_Classdef Buzzer;
extern DT35_classdef DT35;

#endif
