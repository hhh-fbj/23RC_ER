#include "TASK_Vision.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"

void Task_Vision(void *argument)
{
  uint16_t CountNUM;

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  const TickType_t TimeIncrement = pdMS_TO_TICKS(10);  // --- 1MS
  for(;;)
  {
    //  Shoot.BsqJnP2.Read_Float();
//    Vision_SendBufFunction(imu.yaw, imu_Export.gz/16.384f/57.3f, 2);
//		Update_VisionTarget(); //---给视觉发数据，更新目标.
    // Vision.SendToPC(&Vision.Send_Msg);
	
	if(DRF1609H.DRF_page1Flag){DRF1609H.DRF_page1Flag--;DRF1609H.DRT_Printf("page page1");}
	else if(DRF1609H.DRF_page0Flag){DRF1609H.DRF_page0Flag--;DRF1609H.DRT_Printf("page page0");}
	if(DRF1609H.DRF_shejijieshuFlag){DRF1609H.DRF_shejijieshuFlag--;DRF1609H.DRT_Printf("t1.txt=\"无\"");}
  DRF1609H.DRT_Printf("n0.val++");
  vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}