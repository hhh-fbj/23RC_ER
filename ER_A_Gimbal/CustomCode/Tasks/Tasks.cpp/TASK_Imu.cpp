#include "TASK_Imu.h"
/* Includes ------------------------------------------------------------------*/
#include "DEV_AIMU.h"

#include "System_DataPool.h"


/**
 * @brief      数控取样任务
 * @param[in]  None
 * @retval     None
 */
 int BMQ_time;
void Task_DataSampling(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 1MS 

    for(;;)
    {
				imu_Export.IMU_GetData_Compensate();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}