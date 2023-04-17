#include "TASK_Can.h"
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"


/**
 * @brief      CAN1 接收任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Receive(void *argument)
{
	static CanRxMsg_t CAN1_Rx;

    for(;;)
    {
        // --- 等待队列不为�?
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);
				//
        switch(CAN1_Rx.Header.StdId)
        {
            case 0x209:
                Gimbal.Yaw_Motor.update(CAN1_Rx.Data);
                DevicesMonitor.Update(Frame_YAW_MOTOR);
            break;

            case 0x201:
                Clamp.Stretch_Motor.update(CAN1_Rx.Data);
                DevicesMonitor.Update(Frame_STRETCH_MOTOR);
            break;

             case 0x202:
                Clamp.Lift_Motor.update(CAN1_Rx.Data);
                DevicesMonitor.Update(Frame_LIFT_MOTOR);
            break;

             case 0x203:
                Clamp.PickPlace_Motor.update(CAN1_Rx.Data);
                DevicesMonitor.Update(Frame_PICKPLACE_MOTOR);
            break;
        }
    }
}



/**
 * @brief      CAN2 接收任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN2_Receive(void *argument)
{
	static CanRxMsg_t CAN2_Rx;

    for(;;)
    {
        // --- 等待队列不为�?
        xQueueReceive(Can2Recv_QueueHandle, &CAN2_Rx, portMAX_DELAY);

        switch(CAN2_Rx.Header.StdId)
        {
			//拉力
            case 0x201:
                Shoot.Shoot_Motor.update(CAN2_Rx.Data);
                DevicesMonitor.Update(Frame_SHOOT_MOTOR);
            break;

             case 0x202:
                Shoot.LeftPull_Motor.update(CAN2_Rx.Data);
                DevicesMonitor.Update(Frame_LEFTPULL_MOTOR);
            break;

             case 0x203:
                Shoot.RightPull_Motor.update(CAN2_Rx.Data);
                DevicesMonitor.Update(Frame_RIGHTPULL_MOTOR);
            break;

            case 0x006:
                Gimbal.Yaw_Encider.getInfo(CAN2_Rx.Data);
                DevicesMonitor.Update(Frame_YAW_ENCODER);
            break;

            case 0x005:
                Clamp.Stretch_Encider.getInfo(CAN2_Rx.Data);
                DevicesMonitor.Update(Frame_STRETCH_ENCODER);
            break;
        }
    }
}

/**
 * @brief      CAN1 发送任�?
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Transmit(void *argument)
{ 
    static CanTxMsg_t CAN1_Tx;

    for(;;)
    {
        // 等待队列不为�?
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Tx, portMAX_DELAY);
        
    }
}
