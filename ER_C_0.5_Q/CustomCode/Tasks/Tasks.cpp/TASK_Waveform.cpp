#include "Task_UpComputer.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include "DEV_Timer.h"

/**
 * @brief      上位机任务
 * @param[in]  None
 * @retval     None
 */
uint32_t text,text1,text2;
void Task_Debug(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 5MS 

    for(;;)
    {
        // UpperComputer.ANO_DataShow(debug_speed[0],debug_speed[1],debug_speed[2],0);
        // UpperComputer.ANO_DataShow(Chassis.DRV_Motor[0].getSpeed(),-Chassis.DRV_Motor[1].getSpeed(),Chassis.DRV_Motor[2].getSpeed(),-Chassis.DRV_Motor[3].getSpeed());
//				text = Get_SystemTimer();
//			  text2 = text - text1;
//				text1 = text;
//        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
			
        // UpperComputer.ANO_DataShow(Gimbal.UsePID[Pit][PID_Outer].Target, Gimbal.UsePID[Pit][PID_Outer].Current, Gimbal.UsePID[Pit][PID_Inner].Target, Gimbal.UsePID[Pit][PID_Inner].Current);
		//  UpperComputer.ANO_DataShow(Gimbal.UsePID[Yaw][PID_Outer].Target, Gimbal.UsePID[Yaw][PID_Outer].Current, Gimbal.UsePID[Pit][PID_Outer].Target, Gimbal.UsePID[Pit][PID_Outer].Current);
        //  UpperComputer.ANO_DataShow(Gimbal.UsePID[Yaw][PID_Inner].Target, Gimbal.UsePID[Yaw][PID_Inner].Current, Gimbal.UsePID[Pit][PID_Inner].Target, Gimbal.UsePID[Pit][PID_Inner].Current);
//				UpperComputer.ANO_DataShow(Gimbal.UsePID[Yaw][PID_Outer].Target, Gimbal.UsePID[Yaw][PID_Outer].Current, Gimbal.UsePID[Yaw][PID_Inner].Target, Gimbal.UsePID[Yaw][PID_Inner].Current);
//			UpperComputer.ANO_DataShow(Gimbal.UsePID[Pit][PID_Outer].Target, Gimbal.UsePID[Pit][PID_Outer].Current, Gimbal.UsePID[Pit][PID_Inner].Target, Gimbal.UsePID[Pit][PID_Inner].Current);
//			UpperComputer.ANO_DataShow(Gimbal.UsePID[Pit][PID_Outer].Target, Gimbal.UsePID[Pit][PID_Outer].Current, Gimbal.UsePID[Pit][PID_Inner].Out, Gimbal.Motor[Pit].getencoder());


//        UpperComputer.ANO_DataShow(Gimbal.UsePID[Pit][PID_Outer].Target, Gimbal.UsePID[Pit][PID_Outer].Current, Gimbal.UsePID[Pit][PID_Inner].Target, Gimbal.UsePID[Pit][PID_Inner].Current);
//		 UpperComputer.ANO_DataShow((float)Shoot.Fric_PID[Fric_L].Current,  (float)-Shoot.Fric_PID[Fric_R].Current, (float)Shoot.Fric_PID[Fric_L].Target, (float)-Shoot.Fric_PID[Fric_R].Target);
			// UpperComputer.ANO_DataShow(Gimbal.UsePID[Yaw][PID_Inner].Error, Gimbal.UsePID[Yaw][PID_Inner].Out, Gimbal.UsePID[Yaw][PID_Inner].Target, Gimbal.UsePID[Yaw][PID_Inner].Current);
//			UpperComputer.ANO_DataShow(imu_Export.AIMU_gyro[0], imu_Export.AIMU_gyro[1], Gimbal.UsePID[Yaw][PID_Inner].Target, Gimbal.UsePID[Yaw][PID_Inner].Current);
    //  UpperComputer.ANO_DataShow(imu_Export.AIMU_gyro[0], imu_Export.AIMU_gyro[1], Gimbal.UsePID[Yaw][PID_Inner].Out, Gimbal.UsePID[Yaw][PID_Outer].Error);
//			text_vision=0;     
//			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
// UpperComputer.ANO_DataShow((float)Shoot.Reload_PID[0].Target,  (float)Shoot.Reload_PID[0].Current, (float)Shoot.Reload_PID[1].Target, (float)Shoot.Reload_PID[1].Current);
// UpperComputer.ANO_DataShow((float)Shoot.Fric_PID[Fric_L].Current,  (float)-Shoot.Fric_PID[Fric_R].Current, (float)Shoot.Fric_PID[Fric_L].Target, (float)Shoot.Get_GunSpeed()*100);
    //    UpperComputer.ANO_DataShow((float)PowerMeter.RecvData.Pack.Power_Val, (float)PowerMeter.RecvData.Pack.voltageVal, (float)PowerMeter.RecvData.Pack.Shunt_Current, (float)PowerMeter.RecvData.Pack.Shunt_voltage);
//        UpperComputer.ANO_DataShow((float)PowerMeter.Self_RecvData.Pack.Shunt_Current, (float)PowerMeter.Self_RecvData.Pack.voltageVal, (float)PowerMeter.Get_Power_Val()*1000, (float)SupCap.RecvData.chassis_power*1000);
        // UpperComputer.ANO_DataShow((float)PowerMeter.Get_Shunt_Current()*10, (float)PowerMeter.Get_voltageVal()*10, (float)SupCap.Get_Cell(), (float)SupCap.RecvData.chassis_power*10);
        // UpperComputer.ANO_DataShow((float)Cimu_Export.CIMU_onlyAngle[0], (float)imu_Export.AIMU_onlyAngle[0], (float)0, (float)0);
//        UpperComputer.ANO_DataShow(Gimbal.UsePID[Yaw][PID_Inner].Out, Gimbal.UsePID[Yaw][PID_Inner].P_Term, Gimbal.UsePID[Yaw][PID_Inner].Target, Gimbal.UsePID[Yaw][PID_Inner].Current);
        // UpperComputer.ANO_DataShow(Shoot.Get_GunSpeed()*100, Gimbal.UsePID[Yaw][PID_Inner].P_Term, Gimbal.UsePID[Yaw][PID_Inner].Target, Gimbal.UsePID[Yaw][PID_Inner].Current);
        // UpperComputer.ANO_DataShow((float)SupCap.Get_Cell(), (float)SupCap.SendData.charge_power, (float)Chassis.Get_Power(), (float)Chassis.Get_PowerBuffer());
        // UpperComputer.ANO_DataShow_F2((float)PowerMeter.Get_Shunt_Current()*10, (float)PowerMeter.Get_voltageVal()*10, (float)SupCap.RecvData.chassis_power*10, (float)(SupCap.RecvData.cap_usable*10+SupCap.SendData.is_cap_output*20));
        
//        UpperComputer.ANO_DataShow(Vision.Get_YawOffset(), Vision.Get_PitOffset(), Vision.Get_Mode()*10, 0);
        osDelay(1);
    }
}
