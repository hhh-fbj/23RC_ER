#include "System_Config.h"
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "can.h"

//#include "BSP_ADC.h"

#include "DEV_COMM.h"
#include "DEV_Timer.h"


#include "Task_ALL.h"

#include "System_DataPool.h"
#include "include.h"

#include "stm32f4xx.h"

void User_System_Init(void)
{
#if AIMU_Calibrate == AIMU_Calibrate_Off
  flash_read(STM32_FLASH_BASE, imu_Export.IMU_Offset, 6);
#endif
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	imu_Export.DJI_IMU_Init();
#if AIMU_Calibrate == AIMU_Calibrate_On
  flash_write_single_address(STM32_FLASH_BASE, imu_Export.IMUwriteFlashData, 6);
  DJI_IMUWriteOffset(imu_Export.IMUwriteFlashData,6);
#endif
	
//	DJI_IMUFUN.DJI_IMU_Init();

//	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

	/* PWR Init */
//  	PVD_Config();
//	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)DT35.adc_buf,DT35_DMA_SIZE);		//开启ADC1的DMA传输	

		
		Uart_Init(&huart1,Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,DR16_Recv_Callback);
		Uart_Init(&huart6,Clamp.TurnPlace_Servo.MX106_Data,MXDATASIZE,ClampMX106_Recv_Callback);
		Uart_Init(&huart7, Shoot.Shoot_Servo.MX106_Data, MXDATASIZE, ShootMX106_Recv_Callback);

//	//上位机
//	
//	Uart_Init(&huart6, DRF1609H.TJC4827X343_Data, TJC4827X343_SIZE, DRF1609H_Recv_Callback);
//	HAL_UART_Receive_IT(&huart6, DRF1609H.TJC4827X343_Data, TJC4827X343_SIZE);
	//
	

	/* RTOS Resources Init --------------*/
	Can1Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	Can2Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	//CAN
	CAN_Init(&hcan1,User_CAN1_RxCpltCallback);
	CanFilter_Init(&hcan1);
	CAN_Init(&hcan2,User_CAN2_RxCpltCallback);
	CanFilter_Init(&hcan2);

	Timer_Init(&htim4, USE_HAL_DELAY);
	PIDTimer::getMicroTick_regist(Get_SystemTimer);
	
//	HAL_Delay(100);
//	Gimbal.Yaw_Encider.SetInstruction_U8(&hcan2,0x04,0xAA);
//// Gimbal.Yaw_Encider.SetInstruction_U16(&hcan2,0x05,2000);
//	HAL_Delay(100);
////	Clamp.Stretch_Encider.SetInstruction_U8(&hcan2,0x04,0xAA);
//// Clamp.Stretch_Encider.SetInstruction_U16(&hcan2,0x05,2000);
	
	// Chassis.RUD_Encider[0].SetInstruction_U16(&hcan2,0x05,1000);
	// Chassis.RUD_Encider[1].SetInstruction_U16(&hcan2,0x05,1000);
	// Chassis.RUD_Encider[3].SetInstruction_U16(&hcan2,0x05,1000);
	// Chassis.RUD_Encider[4].SetInstruction_U16(&hcan2,0x05,1000);
//	Gimbal.Yaw_Encider.SetInstruction_U16(&hcan1,0x05,1000);
//	Gimbal.Pit_Encider.SetInstruction_U16(&hcan1,0x05,1000);

	//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x04,0xAA);
	//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x04,0xAA);
	//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x04,0xAA);
	//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x04,0xAA);
	//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan1,0x04,0xAA);
	//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan1,0x04,0xAA);
	//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan1,0x04,0xAA);
	//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan1,0x04,0xAA);

//		Gimbal.Yaw_Encider.SetInstruction_U8(&hcan1,0x04,0XAA);
//		Gimbal.Yaw_Encider.SetInstruction_U8(&hcan1,0x02,0X06);
//		Gimbal.Pit_Encider.SetInstruction_U8(&hcan1,0x02,0X06);

//		Gimbal.Yaw_Encider.SetInstruction_U8(&hcan1,0x03,0x01);
//		Gimbal.Pit_Encider.SetInstruction_U8(&hcan1,0x03,0x01);

	//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x03,0X01);
	//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x03,0X01);
	//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x03,0X01);
	//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x03,0X01);
	//	Chassis.RUD_Encider[4].SetInstruction_U8(&hcan2,0x03,0X01);
	//	Chassis.RUD_Encider[RF_Rud].SetInstruction_U8(&hcan2,0x07,0X01);
	//	Chassis.RUD_Encider[LF_Rud].SetInstruction_U8(&hcan2,0x07,0X01);
	//	Chassis.RUD_Encider[LB_Rud].SetInstruction_U8(&hcan2,0x07,0X01);
	//	Chassis.RUD_Encider[RB_Rud].SetInstruction_U8(&hcan2,0x07,0X01);
	//	Chassis.RUD_Encider[4].SetInstruction_U8(&hcan1,0x07,0X00);

	//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan1,0x0C,0X01);
	//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan1,0x0C,0X01);
	//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan1,0x0C,0X01);
	//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan1,0x0C,0X01);
	//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x0C,0X01);
	//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x0C,0X01);
	//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x0C,0X01);
	//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x0C,0X01);
//	DRF1609H.TJCPrintf("\x00"); 
	HAL_Delay(100);
	Buzzer.All_LED(ON);
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
	//--- CAN1 接收任务
	xTaskCreate(Task_CAN1_Receive, "CAN1_Receive", TASK_STACK_SIZE_128, NULL, PriorityHigh, NULL);
	//--- CAN2 接收任务
	xTaskCreate(Task_CAN2_Receive, "CAN2_Receive", TASK_STACK_SIZE_128, NULL, PriorityHigh, NULL);

	//--- 设变检测任务
  	xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_256, NULL, PriorityAboveNormal, NULL);
	
	//--- IMU数据采样任务
	xTaskCreate(Task_DataSampling, "IMUSampling", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);
	
	//--- 上位机任务
	// xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, NULL);

	//--- 总控制任务(放最后创建)
	xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_1024, NULL, PrioritySuperHigh, NULL);
	
//	//LED与音乐
	taskENTER_CRITICAL();
	Buzzer.SuperMario_Music(); //--- 蜂鸣器处理
	Buzzer.All_LED(OFF);
	CTRL_DR16.start = 1;
	taskEXIT_CRITICAL();  
}


