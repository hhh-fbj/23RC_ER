#include "TASK_Check.h"
/* Includes ------------------------------------------------------------------*/
#include "DEV_UpComputer.h"
#include "APP_Devices.h" 

#include "System_DataPool.h"

#include "calibrate_task.h"
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

    static uint8_t gimbal_init = false;
    static uint8_t imu_cali_flag = false;
    static uint8_t DR16_offline_flag = false;
	
    static uint8_t monitor_cnt = 0;
 
    static int16_t low_bat_voltage_cnt = 600;

    for(;;)
    {
			if(1)
			{

				//设备检测 蜂鸣器 --------------------------------------------------------------
        if(((++monitor_cnt)%=10)==0) //--- 200ms
        {
            //--- 设备检测
            DevicesMonitor.Devices_Detec();

            //--- 电源电压检测
            bat_voltage = get_battery_voltage();
            low_bat_voltage_cnt--;

            // 蜂鸣器 ---------------------------------------------------------------------
            if(DevicesMonitor.Get_State(CHASSIS_DRV1_MONITOR) ||
							DevicesMonitor.Get_State(CHASSIS_DRV2_MONITOR) ||
							DevicesMonitor.Get_State(CHASSIS_DRV3_MONITOR))
            {
                //--- 遥控器离线优先一直BBBB...
                Buzzer.Set_RingType(Ring_ContShort);
                DR16_offline_flag = true;
            }
            else if(DevicesMonitor.Get_State(CHASSIS_DRV1_MONITOR)==0&&
							DevicesMonitor.Get_State(CHASSIS_DRV2_MONITOR)==0&&
							DevicesMonitor.Get_State(CHASSIS_DRV3_MONITOR)==0&&
							DevicesMonitor.Get_State(CHASSIS_DRV4_MONITOR)==0&&
							DR16_offline_flag == true)
            {
                Buzzer.Set_RingType(Ring_Stop);
                DR16_offline_flag = false;
            }
            else if(bat_voltage < LOW_BATTER_VOLTAGE && low_bat_voltage_cnt <= 600) //--- 低电压警告
            {
                Buzzer.Set_RingType(Ring_5times); //--- 2分钟一次BBBBB
                if(low_bat_voltage_cnt < 0)
                {
                    Buzzer.Set_RingType(Ring_Stop);
                    low_bat_voltage_cnt = 600;
                }
            }
            else
            {
//                //--- 开机云台初始化完成 马里奥
//                if(Gimbal.init_mode == false && gimbal_init == false)
//                {
////                    Buzzer.Set_RingType(Ring_2times);
////                    // Buzzer.Music_Play(SuperMario, sizeof(SuperMario));
////                    gimbal_init = true;
//                }
                //--- IMU校准提示音 一直B~B~B~B~...
                
								if(cali_sensor[0].cali_cmd == true)
                {
                    Buzzer.Set_RingType(Ring_ContLong);
                    imu_cali_flag = true;
                }
                else if(cali_sensor[0].cali_cmd == false && imu_cali_flag == true)
                {
                    //--- 校准完成 马里奥
                    Buzzer.SuperMario_Music();
                    Buzzer.Set_RingType(Ring_Stop);
                    imu_cali_flag = false; 
                }
            }

            Buzzer.Process(); //--- 蜂鸣器处理
        }
			}

//            //看门狗
//            if(Infantry.State == Robot_Initializing)
//            {
//							HAL_IWDG_Refresh(&hiwdg); //喂狗
//            }
//            else
//            {
//                if(CTRL_DR16.Data_Monitor() == false)
//                {
//                    HAL_IWDG_Refresh(&hiwdg); //喂狗
//                }
//            }

				
	    // DevicesMonitor.Get_FPS(&TIME, &FPS);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
