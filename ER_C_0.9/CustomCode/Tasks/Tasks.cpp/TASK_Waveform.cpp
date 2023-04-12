#include "Task_UpComputer.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include "DEV_Timer.h"

/**
 * @brief      上位机任务
 * @param[in]  None
 * @retval     None
 */
uint32_t text,text1,text2;
void Task_Debug(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 5MS 

    for(;;)
    {
			osDelay(1);
    }
}
