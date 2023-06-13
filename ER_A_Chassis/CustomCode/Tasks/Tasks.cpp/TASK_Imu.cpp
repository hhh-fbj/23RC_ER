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
			if(Chassis.over_init == 1 && CTRL_DR16.start == 1)
			{
				switch(BMQ_time)
				{
					case 0:
						if(DevicesMonitor.Get_State(CHAS_RUDEncider1_MONITOR) && Chassis.RUD_Encider[0].ReadMode != 0x02)
						{
							Chassis.RUD_Encider[0].ReadMode = 0x02;
							Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x01,0x00);
						}
						else if(Chassis.RUD_Encider[0].ReadMode == 0x02)
						{
							Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x01,0x00);
						}
						BMQ_time++;
					break;
					
					case 1:
						if(DevicesMonitor.Get_State(CHAS_RUDEncider2_MONITOR) && Chassis.RUD_Encider[1].ReadMode != 0x02)
						{
							Chassis.RUD_Encider[1].ReadMode = 0x02;
							Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x01,0x00);
						}
						else if(Chassis.RUD_Encider[1].ReadMode == 0x02)
						{
							Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x01,0x00);
						}
						BMQ_time++;
					break;
						
					case 2:
						if(DevicesMonitor.Get_State(CHAS_RUDEncider3_MONITOR) && Chassis.RUD_Encider[2].ReadMode != 0x02)
						{
							Chassis.RUD_Encider[2].ReadMode = 0x02;
							Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x01,0x00);
						}
						else if(Chassis.RUD_Encider[2].ReadMode == 0x02)
						{
							Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x01,0x00);
						}
						BMQ_time=0;
					break;
				}
			}
			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}