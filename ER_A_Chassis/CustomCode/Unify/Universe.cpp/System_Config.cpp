#include "System_Config.h"
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "iwdg.h"

#include "BSP_PVD.h"
#include "BSP_UART.h"
#include "BSP_FLASH.h"
#include "BSP_FlashStore.h"

#include "DEV_AIMU.h"
#include "DEV_COMM.h"
#include "DEV_Timer.h"
#include "DEV_OLED.h"


#include "Task_ALL.h"

#include "System_DataPool.h"
#include "include.h"
RTC_TimeTypeDef  H_S_M_Time; // 时间结构??
RTC_DateTypeDef  Y_M_D_Data; // 日期结构??



void User_System_Init(void)
{
	
#if AIMU_Calibrate == AIMU_Calibrate_Off
  flash_read(STM32_FLASH_BASE, imu_Export.IMU_Offset, 6);
#endif
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	imu_Export.DJI_IMU_Init();
#if AIMU_Calibrate == AIMU_Calibrate_On
//  flash_write_single_address(STM32_FLASH_BASE, imu_Export.IMUwriteFlashData, 6);
#endif
	
//	DJI_IMUFUN.DJI_IMU_Init();

//	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

	/* PWR Init */
//  	PVD_Config();
//	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)DT35.adc_buf,DT35_DMA_SIZE);		//开启ADC1的DMA传输	

    /* Drivers Init ---------------------*/
//	DR16_USART_IT_Init();
	Uart_Init(&huart1,Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,DR16_Recv_Callback);
	Uart_Init(&huart6,Auto.Posture.Recv_Msg.data,Post_Buffer_SIZE,Posture_Recv_Callback);
//	Uart_Init(&huart7,Auto.Analog.ReceiveData, REC_DATA_NUM, Analog_Recv_Callback);
//	Uart_Init(&huart7,DRF1609H.TJC4827X343_Data, TJC4827X343_SIZE, DRF1609H_Recv_Callback);

	CAN_Init(&hcan1,User_CAN1_RxCpltCallback);
	CanFilter_Init(&hcan1);
	CAN_Init(&hcan2,User_CAN2_RxCpltCallback);
	CanFilter_Init(&hcan2);

	/* RTOS Resources Init --------------*/
	Can1Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	Can2Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));

	Timer_Init(&htim4, USE_HAL_DELAY);
	PIDTimer::getMicroTick_regist(Get_SystemTimer);

	HAL_Delay(100);
	Buzzer.All_LED(ON);
	HAL_Delay(100);


//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan1,0x02,4);
// Chassis.RUD_Encider[0].SetInstruction_U16(&hcan2,0x05,2000);
// Chassis.RUD_Encider[1].SetInstruction_U16(&hcan2,0x05,2000);
// Chassis.RUD_Encider[2].SetInstruction_U16(&hcan2,0x05,2000);
// Chassis.RUD_Encider[3].SetInstruction_U16(&hcan2,0x05,2000);

//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x04,0x00);
//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x04,0x00);
//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x04,0x00);
//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x04,0x00);
//	Chassis.RUD_Encider[0].SetInstruction_U8(&hcan1,0x04,0xAA);
//	Chassis.RUD_Encider[1].SetInstruction_U8(&hcan1,0x04,0xAA);
//	Chassis.RUD_Encider[2].SetInstruction_U8(&hcan1,0x04,0xAA);
//	Chassis.RUD_Encider[3].SetInstruction_U8(&hcan1,0x04,0xAA);

//	if(Chassis.RUD_Encider[0].ReadMode == 0x01){Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x04,0xAA);}
//	HAL_Delay(5);
//	if(Chassis.RUD_Encider[1].ReadMode == 0x01){Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x04,0xAA);}
//	HAL_Delay(5);
//	if(Chassis.RUD_Encider[2].ReadMode == 0x01){Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x04,0xAA);}
//	HAL_Delay(5);
//	if(Chassis.RUD_Encider[3].ReadMode == 0x01){Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x04,0xAA);}
	
	Chassis.RUD_Encider[0].SetInstruction_U16(&hcan1,0x05,4000);
	HAL_Delay(500);
	Chassis.RUD_Encider[1].SetInstruction_U16(&hcan1,0x05,5000);
	HAL_Delay(500);
	Chassis.RUD_Encider[2].SetInstruction_U16(&hcan1,0x05,5500);
	HAL_Delay(500);
//	Chassis.RUD_Encider[3].SetInstruction_U16(&hcan1,0x05,4500);
//	HAL_Delay(500);
//		Chassis.RUD_Encider[0].SetInstruction_U8(&hcan1,0x0C,0x01);
//		HAL_Delay(500);
//		Chassis.RUD_Encider[1].SetInstruction_U8(&hcan1,0x0C,0x01);
//		HAL_Delay(500);
//		Chassis.RUD_Encider[2].SetInstruction_U8(&hcan1,0x0C,0x01);
//		HAL_Delay(500);
//		Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x02,0x03);
//		HAL_Delay(5);
//		Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x04,0xAA);

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

	HAL_Delay(100);
	Chassis.over_init = 1;
}

/**
  * @brief      创建用户任务
  * @param[in]  None
  * @return     None
  */
void System_Tasks_Init(void)
{

	//--- CAN1 接收任务
	xTaskCreate(Task_CAN1_Receive, "CAN1_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);
	//--- CAN2 接收任务
	xTaskCreate(Task_CAN2_Receive, "CAN2_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

	//--- 设变检测任务
  	xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, NULL);
	//--- IMU数据采样任务
	xTaskCreate(Task_DataSampling, "IMUSampling", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);
	//--- 上位机任务
	// xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_128, NULL, PriorityNormal, NULL);

	//--- 总控制任务(放最后创建)
	xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_512, NULL, PriorityNormal, NULL);
	
  taskENTER_CRITICAL();
//	Buzzer.SuperMario_Music();
	// MX_IWDG_Init();
	Buzzer.All_LED(OFF);
	CTRL_DR16.start = 1;
  taskEXIT_CRITICAL();  
	
}


