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
//		if(Auto.Posture.TenMi)
//		{
//			Auto.Posture.TenMi++;
//			if(Auto.Posture.TenMi>3000){Auto.Posture.TenMi=0;}
//			Chassis.Set_Mode(CHAS_DisableMode);
//			Auto.Posture.error = false;
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//默认
//		}
			led_cnt++;
			Chassis.Control();
//		L1TTL.control();
    if((led_cnt%=50) == 0)
    {   //--- 任务正常运行流水灯
        Buzzer.Waterfall_LED();
    }
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}
