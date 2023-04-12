#include "TASK_Check.h"
/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"
#include "DEV_UpComputer.h"
#include "DEV_OLED.h"
#include "APP_Devices.h" 

#include "System_DataPool.h"

/**
 * @brief      设备检测任务
 * @param[in]  None
 * @retval     None
 */
void Task_DevicesMonitor(void *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(20);  

    static uint8_t monitor_cnt = 0;
 
    static int16_t low_bat_voltage_cnt = 600;

    for(;;)
    {
        if(((++monitor_cnt)%=10)==0 && Chassis.over_init == 1 && CTRL_DR16.start == 1)// --- 200MS 
        {
            DevicesMonitor.Devices_Detec();
            //--- 遥控器离线
            if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
            {
                
            }
            if(DevicesMonitor.Get_State(CHAS_RUDEncider1_MONITOR) == Off_line||
								DevicesMonitor.Get_State(CHAS_RUDEncider2_MONITOR) == Off_line||
								DevicesMonitor.Get_State(CHAS_RUDEncider3_MONITOR) == Off_line||
								DevicesMonitor.Get_State(CHAS_RUDEncider4_MONITOR) == Off_line)
            {
                Buzzer.error = 0;//1
            }
            else
            {
                Buzzer.error = 0;
            }
					
            Buzzer.Error_PromptTone();

            // //看门狗
            // if(Infantry.State == Robot_Initializing)
            // {
			// 	HAL_IWDG_Refresh(&hiwdg); //喂狗
            // }
            // else
            // {
            //     if(CTRL_DR16.Data_Monitor() == false)
            //     {
            //         HAL_IWDG_Refresh(&hiwdg); //喂狗
            //     }
            // }
        }


        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
