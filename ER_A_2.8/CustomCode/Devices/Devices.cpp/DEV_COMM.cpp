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


#include "APP_DR16.h"

#include "System_DataPool.h"



/**
 * @brief      USART1(DR16) 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */
uint32_t DR16_Recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
    if(CTRL_DR16.start)
    {
        DR16.DataCapture((DR16_DataPack_Typedef*)Recv_Data);
        CTRL_DR16.LeverMode_Update();
        DevicesMonitor.Update(Frame_DR16);
    }
    return 0;
}

/**
 * @brief      USART(Posture) 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */

uint32_t Posture_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
    Auto.Posture.getMessage(Recv_Data);
    
    return 0;
}

/**
 * @brief      USART() 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */
uint32_t CusartA_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
	DevicesMonitor.Update(Frame_CHAS_CUSARTA);
    return 0;
}

/**
 * @brief      USART() 鎺ュ彈涓柇鍥炶皟鍑芥暟
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */
uint32_t Analog_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
    Auto.Analog.Receive(Recv_Data);
    DevicesMonitor.Update(Frame_CHAS_ANALOG);
    return 0;
}

uint32_t DRF1609H_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
    DRF1609H.Receive(Recv_Data);
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