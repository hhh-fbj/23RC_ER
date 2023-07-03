#include "Task_Control.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include "DEV_Timer.h"

/**
 * @brief      总控任务
 * @param[in]  None
 * @retval     None
 */
GPIO_PinState ooo[5];
void Task_Control(void *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS 
  for(;;)
  {
//		//抬伸
//		ooo[0] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);//抬伸
//		//夹子
//		ooo[1] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);//取环
//		//发射			
//		ooo[2] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);//抬伸
//		//	右
//		ooo[3] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);//取环
//		//左
//		ooo[4] = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10);//抬伸
		Gimbal.Control();
		Shoot.Control();
		Clamp.Control();
		// ---发送CAN
		MotorMsgSend(&hcan1, Shoot.Motor); 
		MotorMsgSend(&hcan1, Gimbal.Yaw_Motor);
		MotorMsgSend(&hcan2, Clamp.Motor);
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}
