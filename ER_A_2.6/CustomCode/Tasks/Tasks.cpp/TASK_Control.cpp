#include "Task_Control.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"

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
  static uint8_t led_cnt = 0;
  
  for(;;)
  {
    led_cnt++;
		Chassis.Control();
    if((led_cnt%=50) == 0)
    {   //--- 任务正常运行流水灯
        Buzzer.Waterfall_LED();
    }

    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}
