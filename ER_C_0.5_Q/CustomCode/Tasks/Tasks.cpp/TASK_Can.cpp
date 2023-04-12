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
        // --- 等待队列不为空
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);
				//
			
			  switch(CAN1_Rx.Header.StdId)
        {
					case 0x341:
						Chassis.CAN_Recv_RFLB(CAN1_Rx.Data);
						DevicesMonitor.Update(Frame_CHAS_AUSARTC_RFLB);
					break;

          case 0x342:
						Chassis.CAN_Recv_LFRB(CAN1_Rx.Data);
						DevicesMonitor.Update(Frame_CHAS_AUSARTC_LFRB);
					break;
				}
        switch(CAN1_Rx.Header.ExtId & 0xFF)
        {
//            case 11://
//                Chassis.DRV_Motor[RF_Drv].State_getInfo(CAN1_Rx.Header.ExtId>>8, CAN1_Rx.Data);
//								DevicesMonitor.Update(Frame_CHAS_DRV1);
//            break;
//						case 12://
//                Chassis.DRV_Motor[LF_Drv].State_getInfo(CAN1_Rx.Header.ExtId>>8, CAN1_Rx.Data);
//								DevicesMonitor.Update(Frame_CHAS_DRV2);
//            break;
//            case 13://
//                Chassis.DRV_Motor[RB_Drv].State_getInfo(CAN1_Rx.Header.ExtId>>8, CAN1_Rx.Data);
//								DevicesMonitor.Update(Frame_CHAS_DRV4);
//            break;
//            case 14://
//                Chassis.DRV_Motor[LB_Drv].State_getInfo(CAN1_Rx.Header.ExtId>>8, CAN1_Rx.Data);
//								DevicesMonitor.Update(Frame_CHAS_DRV3);
//            break;
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

				switch(CAN2_Rx.Header.ExtId & 0xFF)
        {
            case 11://
                Chassis.DRV_Motor[RF_Drv].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_DRV1);
            break;
						case 12://
                Chassis.DRV_Motor[LF_Drv].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_DRV2);
            break;
            case 13://
                Chassis.DRV_Motor[RB_Drv].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
								DevicesMonitor.Update(Frame_CHAS_DRV4);
            break;
            case 14://
                Chassis.DRV_Motor[LB_Drv].State_getInfo(CAN2_Rx.Header.ExtId>>8, CAN2_Rx.Data);
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