#ifndef __DEV_COMM_H
#define __DEV_COMM_H

#include "include.h"
#include "System_DataPool.h"
#include "DEV_DR16.h"
#include "DEV_Motor.h"
#include "main.h"



/*------------------------------ System Handlers ------------------------------*/

	
	
/*------------------------------Function prototypes ---------------------------*/   
uint32_t DR16_Recv_Callback(uint8_t *buf, uint16_t len);
uint32_t BsqJnP2_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
uint32_t MX106_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
uint32_t Vision_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
void User_CAN1_RxCpltCallback(CanRxMsg_t *CAN_RxMessage);
void User_CAN2_RxCpltCallback(CanRxMsg_t *CAN_RxMessage);




#endif
