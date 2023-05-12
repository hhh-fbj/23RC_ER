#include "Task_LED.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"

/**
 * @brief      LED BUZZER
 * @param[in]  None
 * @retval     None
 */
 int BMQ_time;
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
        BMQ_time++;
				
				//ÊÓ¾õÍ¨Ñ¶
//        if(BMQ_time % 25 == 0){Vision.SendToPC(&Vision.Send_Msg);}
				
        if(DevicesMonitor.Get_State(YAW_ENCODER_MONITOR)  == Off_line && (BMQ_time % 50 == 0))
        {
            Gimbal.Yaw_Encider.SetInstruction_U8(&hcan2,0x04,0xAA);
        }
        if(DevicesMonitor.Get_State(STRETCH_ENCODER_MONITOR)  == Off_line && (BMQ_time % 75 == 0))
        {
            Clamp.Stretch_Encider.SetInstruction_U8(&hcan2,0x04,0xAA);
        }

//				if(CTRL_DR16.start == 1)
//				{
//						if(DevicesMonitor.Get_State(YAW_ENCODER_MONITOR) && BMQ_time%2==0 && Gimbal.Yaw_Encider.ReadMode != 0x02)
//						{
//								Gimbal.Yaw_Encider.ReadMode = 0x02;
//								Gimbal.Yaw_Encider.SetInstruction_U8(&hcan2,0x01,0x00);
//						}
//						else if(Gimbal.Yaw_Encider.ReadMode == 0x02 && BMQ_time%6==0)
//						{
//								Gimbal.Yaw_Encider.SetInstruction_U8(&hcan2,0x01,0x00);
//						}

//						if(DevicesMonitor.Get_State(STRETCH_ENCODER_MONITOR) && BMQ_time%3==0 && Clamp.Stretch_Encider.ReadMode != 0x02)
//						{
//								Clamp.Stretch_Encider.ReadMode = 0x02;
//								Clamp.Stretch_Encider.SetInstruction_U8(&hcan2,0x01,0x00);
//						}
//						else if(Clamp.Stretch_Encider.ReadMode == 0x02 && BMQ_time%7==0)
//						{
//								Clamp.Stretch_Encider.SetInstruction_U8(&hcan2,0x01,0x00);
//						}

						if(BMQ_time>1000000){BMQ_time=0;}
//				}

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

