/**
 ------------------------------------------------------------------------------
 * @file    Communation.cpp
 * @author  Shake
 * @brief   数据通讯
 * @version V0.1
 * @date    2021-10-14
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "DEV_COMM.h"
/* Includes ------------------------------------------------------------------*/
#include "BSP_UART.h"


#include "System_DataPool.h"

/**
 * @brief      USART1(DR16) 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */
uint32_t AusartC_Recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
    Chassis.Receive_AusartC();
//    DevicesMonitor.Update(rame_CHAS_AUSARTC);
    return 0;
}
/**
 * @brief      CAN1 接收中断回调函数
 * @param[in]  CAN_RxMessage
 * @retval     None
 */
void User_CAN1_RxCpltCallback(CanRxMsg_t *CAN_RxMessage)
{
    if(Can1Recv_QueueHandle == NULL)
    {
        return;
    }
    xQueueSendToBackFromISR(Can1Recv_QueueHandle,CAN_RxMessage,0); //--- Ban on waiting.
    DevicesMonitor.Update(Frame_CAN1);
}


/**
 * @brief      CAN2 接收中断回调函数
 * @param[in]  CAN_RxMessage
 * @retval     None
 */
void User_CAN2_RxCpltCallback(CanRxMsg_t *CAN_RxMessage)
{
    if(Can2Recv_QueueHandle == NULL)
    {
        return;
    }
    xQueueSendToBackFromISR(Can2Recv_QueueHandle,CAN_RxMessage,0); //--- Ban on waiting.
    DevicesMonitor.Update(Frame_CAN2);
}


/**
 * @brief      USART3(Referee) 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */



uint32_t Vision_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
    return 0;
}
