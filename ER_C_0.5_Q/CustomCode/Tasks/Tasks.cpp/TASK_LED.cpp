#include "Task_LED.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"

/**
 * @brief      LED BUZZER
 * @param[in]  None
 * @retval     None
 */
void Task_LED(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 5MS 

    // static uint8_t init_flag = true;

    for(;;)
    {
        if(0)
        {
            LED.BLN_Ctrl();
        }
				
				
//        SendIMU.IMU_Process();

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

