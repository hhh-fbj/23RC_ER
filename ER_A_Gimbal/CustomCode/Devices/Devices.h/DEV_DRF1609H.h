#ifndef __DEV_DRF1609H_H
#define __DEV_DRF1609H_H

#include <stdint.h>

#define DRF_REC_SIZE 6
#define DRF_SEN_SIZE 9

#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
	uint8_t RID;
	uint8_t s_T; /*<! 帧头'S' */
	uint8_t SID;
	uint8_t GN;
	float Data;
	uint8_t e_T;	 /*<! 帧尾'E' */
}DRF_SEN_Pack_t;
#pragma pack()
typedef union 
{
	DRF_SEN_Pack_t Pack;
	uint8_t data[DRF_SEN_SIZE];
}DRF_SEN_SendMsg_u;

class DRF1609H_classdef
{
private:
    uint16_t Deal_Flag,Backup_Deal_Flag;
public: 
		DRF1609H_classdef();
    uint8_t DRF_REC_Data[DRF_REC_SIZE];
		DRF_SEN_SendMsg_u DRF_SEN;
    void Receive(uint8_t *data);
    void Send(void);
		void SetSend(void);
};

#endif