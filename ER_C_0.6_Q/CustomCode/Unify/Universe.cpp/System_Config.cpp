#include "System_Config.h"
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "can.h"

#include "BSP_ADC.h"

#include "DEV_CIMU.h"
#include "DEV_COMM.h"
#include "DEV_Timer.h"
#include "calibrate_task.h"


#include "Task_ALL.h"

#include "System_DataPool.h"
#include "include.h"

#include "INS_task.h"
#include "stm32f4xx.h"

void User_System_Init(void)
{
  // taskENTER_CRITICAL();
	//弹仓盖
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	/* DJI-C IMU Init */  
	HAL_TIM_Base_Start(&htim10);
	cali_param_init();
	/* PWR Init */
  	PVD_Config();
	/* 使用基准电压来校准ADC电压采样 */
   init_vrefint_reciprocal();
	
	//
//	Uart_Init(&huart6,Chassis.Recv_Msg.data,AusartC_SIZE,AusartC_Recv_Callback);

	//CAN
	CAN_Init(&hcan1,User_CAN1_RxCpltCallback);
	CanFilter_Init(&hcan1);
	CAN_Init(&hcan2,User_CAN2_RxCpltCallback);
	CanFilter_Init(&hcan2);

	Timer_Init(&htim2, USE_HAL_DELAY);
	PIDTimer::getMicroTick_regist(Get_SystemTimer);
	LED.Init();

	HAL_Delay(100);
//	  
  // taskEXIT_CRITICAL();
}

/**
  * @brief      创建用户任务
  * @param[in]  None
  * @return     None
  */
void System_Tasks_Init(void)
{
	taskENTER_CRITICAL();
    //--- LED Buzzer任务
 	xTaskCreate(Task_LED, "LED_Buzzer", TASK_STACK_SIZE_128, NULL, PriorityNormal, NULL);

	//--- CAN1 接收任务
	Can1Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	xTaskCreate(Task_CAN1_Receive, "CAN1_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);
	//--- CAN2 接收任务
	Can2Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	xTaskCreate(Task_CAN2_Receive, "CAN2_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

	//--- 设变检测任务
  	xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, NULL);
//	
//	//--- IMU数据读取任务
	xTaskCreate((TaskFunction_t)INS_task, "IMU_Update", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);
//	
	//--- 上位机任务
	// xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, NULL);

	//--- 总控制任务(放最后创建)
	xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_512, NULL, PrioritySuperHigh, NULL);
	
//	//LED与音乐
	Buzzer.SuperMario_Music(); //--- 蜂鸣器处理
	taskEXIT_CRITICAL();  
}


