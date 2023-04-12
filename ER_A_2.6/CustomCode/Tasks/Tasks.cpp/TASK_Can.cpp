#include "TASK_Can.h"
/* Includes ------------------------------------------------------------------*/
#include "APP_Chassis.h"

#include "System_DataPool.h"


/**
 * @brief      CAN1 接收任务
 * @param[in]  None
 * @retval     None
 */
int sss;
void Task_CAN1_Receive(void *argument)
{
	static CanRxMsg_t CAN1_Rx;

    for(;;)
    {
        // --- 等待队列不为空
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);
				//编码器
        switch(CAN1_Rx.Header.StdId)
        {
						case 0x202:
                Chassis.RUD_Motor[1].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD2);
            break;
            case 0x203:
                Chassis.RUD_Motor[2].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD3);
            break;	
            case 0x201:
                Chassis.RUD_Motor[0].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD1);
            break;
						case 0x204:
                Chassis.RUD_Motor[3].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD4);
						break;
						// case 0x342:
            //     Chassis.CAN_Recvd(CAN1_Rx.Data);
						// 		DevicesMonitor.Update(Frame_CHAS_CUSARTA);
						// break;
						
						case 0x205:
						break;
						
						case 0x206:
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
        // --- 等待队列不为空
        xQueueReceive(Can2Recv_QueueHandle, &CAN2_Rx, portMAX_DELAY);
				//2006
        switch(CAN2_Rx.Header.StdId)
        {
					case 0x002://
                Chassis.RUD_Encider[1].getInfo(CAN2_Rx.Data);
				DevicesMonitor.Update(Frame_CHAS_RUDEncider2);
            break;
            case 0x003://
                Chassis.RUD_Encider[2].getInfo(CAN2_Rx.Data);
				DevicesMonitor.Update(Frame_CHAS_RUDEncider3);
            break;
						case 0x001://
                Chassis.RUD_Encider[0].getInfo(CAN2_Rx.Data);
				DevicesMonitor.Update(Frame_CHAS_RUDEncider1);
            break;			
						case 0x004://
                Chassis.RUD_Encider[3].getInfo(CAN2_Rx.Data);
				DevicesMonitor.Update(Frame_CHAS_RUDEncider4);
            break;	
            
        }
    }
}

/**
 * @brief      CAN1 发送任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Transmit(void *argument)
{ 
    static CanTxMsg_t CAN1_Tx;
    for(;;)
    {
			// 等待队列不为空
			xQueueReceive(Can1Recv_QueueHandle, &CAN1_Tx, portMAX_DELAY);
    }
}
