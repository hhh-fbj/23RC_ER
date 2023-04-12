#include "System_DataPool.h"
/* Includes ------------------------------------------------------------------*/

//用于存储所有需要的变量
/* RTOS Resources ------------------------------------------------------------*/
//Queue
QueueHandle_t Can1Recv_QueueHandle;
QueueHandle_t Can2Recv_QueueHandle;
QueueHandle_t Can1Send_QueueHandle;

QueueHandle_t xQueueDR16Receive_Handle;

QueueHandle_t Vision_QueueHandle;
TaskHandle_t Visionrecv_TaskHandle;


/* Other Resources ------------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart1 */
uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart3 */


float text_vision;

/* Global Objects ------------------------------------------------------------*/
DR16_Classdef DR16;
CTRL_DR16_classdef CTRL_DR16;
DevicesMonitor_classdef DevicesMonitor;
Gimbal_classdef Gimbal;
Shoot_classdef Shoot;
Clamp_classdef Clamp;
Vision_classdef Vision;

CIMU_Data_Classdef Cimu_Export;
Buzzer_Classdef Buzzer;
LED_classdef LED;