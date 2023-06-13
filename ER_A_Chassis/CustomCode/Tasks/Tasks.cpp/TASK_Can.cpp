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
					case 0x004://
							Chassis.RUD_Encider[Left_Rud].getInfo(CAN1_Rx.Data);
							DevicesMonitor.Update(Frame_CHAS_RUDEncider1);
					break;			
					case 0x002://
                Chassis.RUD_Encider[Front_Rud].getInfo(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUDEncider2);
            break;
            case 0x003://
                Chassis.RUD_Encider[Right_Rud].getInfo(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUDEncider3);
            break;
					
            case 0x201:
                Chassis.RUD_Motor[Front_Rud].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD1);
            break;
						case 0x202:
                Chassis.RUD_Motor[Left_Rud].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD2);
            break;
            case 0x203:
                Chassis.RUD_Motor[Right_Rud].update(CAN1_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_RUD3);
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
        {}
				
				switch(CAN2_Rx.Header.ExtId & 0xFF)
        {
            case 11://
                Chassis.DRV_Motor[Front_Rud].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_DRV1);
            break;
						case 12://
                Chassis.DRV_Motor[Left_Rud].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_DRV2);
            break;
            case 13://
                Chassis.DRV_Motor[Right_Rud].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_DRV3);
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
