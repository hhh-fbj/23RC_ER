#include "TASK_Vision.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"

void Task_Vision(void *argument)
{
  uint16_t CountNUM;

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 1MS
  for(;;)
  {
//    Vision_SendBufFunction(imu.yaw, imu_Export.gz/16.384f/57.3f, 2);
//		Update_VisionTarget(); //---给视觉发数据，更新目标.
    // Vision.SendToPC(&Vision.Send_Msg);
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}