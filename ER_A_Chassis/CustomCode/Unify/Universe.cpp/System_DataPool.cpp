#include "System_DataPool.h"
/* Includes ------------------------------------------------------------------*/

//用于存储所有需要的变量
/* RTOS Resources ------------------------------------------------------------*/
//Queue
QueueHandle_t Can1Recv_QueueHandle;
QueueHandle_t Can2Recv_QueueHandle;
QueueHandle_t Can1Send_QueueHandle;

QueueHandle_t xQueueDR16Receive_Handle;

QueueHandle_t Posture_QueueHandle;
QueueHandle_t Vision_QueueHandle;

TaskHandle_t Posture_TaskHandle;
TaskHandle_t Visionrecv_TaskHandle;


/* Other Resources ------------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart1 */
uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart3 */


/* Global Objects ------------------------------------------------------------*/
DR16_Classdef DR16;
CTRL_DR16_classdef CTRL_DR16;
DevicesMonitor_classdef DevicesMonitor;
Chassis_classdef Chassis;
Auto_classdef Auto;
DRF1609H_classdef DRF1609H;
//L1TTL_classdef L1TTL;

AIMU_Data_Classdef imu_Export;
Buzzer_Classdef Buzzer;
DT35_classdef DT35;
