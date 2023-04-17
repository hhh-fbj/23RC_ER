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
		BMQ_time++;
		if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
		{
				for(uint8_t i = 0 ; i < 4 ; i++)
				{
					Chassis.Cal_Speed[i] = 0;
				}
		}
		else
		{
			Chassis.CAN_Send();
		}

		if(Chassis.over_init == 1 && CTRL_DR16.start == 1)
		{
			if(DevicesMonitor.Get_State(CHAS_RUDEncider1_MONITOR) && BMQ_time%2==0 && Chassis.RUD_Encider[0].ReadMode != 0x02)
			{
				Chassis.RUD_Encider[0].ReadMode = 0x02;
				Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			else if(Chassis.RUD_Encider[0].ReadMode == 0x02 && BMQ_time%6==0)
			{
				Chassis.RUD_Encider[0].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			
			if(DevicesMonitor.Get_State(CHAS_RUDEncider2_MONITOR) && BMQ_time%3==0 && Chassis.RUD_Encider[1].ReadMode != 0x02)
			{
				Chassis.RUD_Encider[1].ReadMode = 0x02;
				Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			else if(Chassis.RUD_Encider[1].ReadMode == 0x02 && BMQ_time%7==0)
			{
				Chassis.RUD_Encider[1].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			
			if(DevicesMonitor.Get_State(CHAS_RUDEncider3_MONITOR) && BMQ_time%5==0 && Chassis.RUD_Encider[2].ReadMode != 0x02)
			{
				Chassis.RUD_Encider[2].ReadMode = 0x02;
				Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			else if(Chassis.RUD_Encider[2].ReadMode == 0x02 && BMQ_time%8==0)
			{
				Chassis.RUD_Encider[2].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			
			if(DevicesMonitor.Get_State(CHAS_RUDEncider4_MONITOR) && BMQ_time%7==0 && Chassis.RUD_Encider[3].ReadMode != 0x02)
			{
				Chassis.RUD_Encider[3].ReadMode = 0x02;
				Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x01,0x00);
			}
			else if(Chassis.RUD_Encider[3].ReadMode == 0x02 && BMQ_time%5==0)
			{
				Chassis.RUD_Encider[3].SetInstruction_U8(&hcan2,0x01,0x00);
			}
		}

				
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}