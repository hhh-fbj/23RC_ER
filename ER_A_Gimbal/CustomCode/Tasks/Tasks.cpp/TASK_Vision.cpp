#include "TASK_Vision.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"

int vision_sssss;
uint8_t Read_Time=1;
int a,b,c;
void Task_Vision(void *argument)
{
  uint16_t CountNUM;

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  const TickType_t TimeIncrement = pdMS_TO_TICKS(50);  // --- 1MS
  for(;;)
  {
    //  Shoot.BsqJnP2.Read_Float();
//    Vision_SendBufFunction(imu.yaw, imu_Export.gz/16.384f/57.3f, 2);
//		Update_VisionTarget(); //---给视觉发数据，更新目标.
    // Vision.SendToPC(&Vision.Send_Msg);
	switch(vision_sssss)
	{
		case 1:DRF1609H.DRT_Printf("page page1");vision_sssss=0;break;
		case 2:DRF1609H.DRT_Printf("page page0");vision_sssss=0;break;
		case 3:DRF1609H.DRT_Printf("t1.txt=\"无\"");vision_sssss=0;break;
		case 4:DRF1609H.DRT_Printf("n3.val=%d",6);vision_sssss=0;break;
		case 5:DRF1609H.DRT_Printf("t1.txt=\"2号柱\"");vision_sssss=0;break;
		case 6:DRF1609H.DRT_Printf("page page0");vision_sssss=0;break;
	}
	if(DRF1609H.Read_Flag)
	{
		switch(Read_Time)
		{
			case 1:Read_Time=2;DRF1609H.DRT_Printf("p1_y=%d",Gimbal.CS_SJ);break;
			case 2:Read_Time=3;DRF1609H.DRT_Printf("p1_l=%d",Shoot.Left_CS_SJ);break;
			case 3:Read_Time=4;DRF1609H.DRT_Printf("p1_r=%d",Shoot.Right_CS_SJ);break;
			case 4:Read_Time=1;DRF1609H.DRT_Printf("n0.val++");break;
		}
		a++;b+=2;c-=5;
	}

//	if(DRF1609H.DRF_page1Flag){DRF1609H.DRF_page1Flag--;DRF1609H.DRT_Printf("page page1");}
//	else if(DRF1609H.DRF_page0Flag){DRF1609H.DRF_page0Flag--;DRF1609H.DRT_Printf("page page0");}
//	if(DRF1609H.DRF_shejijieshuFlag){DRF1609H.DRF_shejijieshuFlag--;DRF1609H.DRT_Printf("t1.txt=\"无\"");}
  DRF1609H.DRT_Printf("n0.val++");
  vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}