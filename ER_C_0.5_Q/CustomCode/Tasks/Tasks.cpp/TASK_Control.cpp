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
		Chassis.Control();
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}
