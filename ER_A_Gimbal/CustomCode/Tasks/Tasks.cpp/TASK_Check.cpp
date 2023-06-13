#include "TASK_Check.h"
/* Includes ------------------------------------------------------------------*/
#include "DEV_UpComputer.h"
#include "APP_Devices.h" 

#include "System_DataPool.h"

/**
 * @brief      设备检测任务
 * @param[in]  None
 * @retval     None
 */
float bat_voltage; //电源电压
void Task_DevicesMonitor(void *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(20);  

    static uint8_t monitor_cnt = 0;
 
    static int16_t low_bat_voltage_cnt = 600;

    for(;;)
    {
        if(((++monitor_cnt)%=10)==0 && CTRL_DR16.start == 1)// --- 200MS 
        {
            DevicesMonitor.Devices_Detec();
            //--- 遥控器离线
            if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
            {
                
            }
            if(0)
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
				
	    // DevicesMonitor.Get_FPS(&TIME, &FPS);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
