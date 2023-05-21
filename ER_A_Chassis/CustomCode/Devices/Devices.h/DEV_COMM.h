#ifndef __DEV_COMM_H
#define __DEV_COMM_H

#include "include.h"
#include "System_DataPool.h"
#include "DEV_DR16.h"
#include "DEV_Motor.h"
#include "main.h"



/*------------------------------ System Handlers ------------------------------*/

	
	
/*------------------------------Function prototypes ---------------------------*/   
uint32_t DR16_Recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen);
uint32_t Posture_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
uint32_t CusartA_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
uint32_t Analog_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
uint32_t DRF1609H_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
void User_CAN1_RxCpltCallback(CanRxMsg_t *CAN_RxMessage);
void User_CAN2_RxCpltCallback(CanRxMsg_t *CAN_RxMessage);




#endif
