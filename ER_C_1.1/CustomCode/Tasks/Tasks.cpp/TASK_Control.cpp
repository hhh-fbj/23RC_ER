#include "Task_Control.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include "INS_task.h"
#include "DEV_Timer.h"

/**
 * @brief      总控任务
 * @param[in]  None
 * @retval     None
 */

void Task_Control(void *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS 
  for(;;)
  {
		Gimbal.Control();
		Shoot.Control();
		Clamp.Control();
		// ---发送CAN
		MotorMsgSend(&hcan2, Shoot.Motor); 
		MotorMsgSend(&hcan1, Gimbal.Yaw_Motor);
		MotorMsgSend(&hcan1, Clamp.Motor);
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}
