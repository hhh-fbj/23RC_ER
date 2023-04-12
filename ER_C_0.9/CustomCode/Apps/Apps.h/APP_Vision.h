#ifndef __APP_VISION_H
#define __APP_VISION_H

//#pragma anon_unions

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "ALGO_Kalman.h"
#include "APP_Devices.h"
#include "stm32f4xx_hal.h"
/* Private macros ------------------------------------------------------------*/
#define VISION_TX_SIZE    1
#define VISION_RX_SIZE    15


/* Exported types ------------------------------------------------------------*/

enum VisionMode_e
{
	Vision_Disable = 0,		// 视觉失能	
	Vision_Aim,
};


/**
 * @brief 设置结构体的对齐边界为1字节 让数据在内存储存中是连续的
 * @note  通过公用内存将数据同步 修改协议时修改Pack的数据与内存大小
 */
#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
	uint8_t start_Tag; /*<! 帧头'S' */
	float Yaw;
	float Depth;
	float X;
	uint8_t crc;
	uint8_t end_tag;	 /*<! 帧尾'E' */
} VisionRecv_Pack_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
//	uint8_t start_tag;		/*<! 帧头'S' */
	uint8_t detection;
//	uint8_t crc;			/*<! crc检验位 */
//	uint8_t end_tag;		/*<! 帧尾'E' */
}VisionSend_Pack_t;
#pragma pack()

typedef union 
{
	uint8_t data[VISION_TX_SIZE];
	VisionSend_Pack_t Pack;
}VisionSendMsg_u;

typedef union
{
	uint8_t data[VISION_RX_SIZE];
	VisionRecv_Pack_t Pack;
}VisionRecvMsg_u;





class Vision_classdef
{
private:
	uint8_t CRCBuf;
	uint8_t Reserve_Num;
	float Yaw_Reserve[20];
	float Depth_Reserve[20];
public:
	uint8_t aim;
  WorldTime_t TIME;
	uint16_t FPS;        /*<! 帧率 */
	float Use_Yaw,Use_Depth,Use_Flag;

	VisionSendMsg_u Send_Msg; /*<! 打包数据至小电脑 */
	VisionRecvMsg_u Recv_Msg; /*<! 从小电脑解包数据 */

	Vision_classdef();

	void RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
	void SendToPC(VisionSendMsg_u *pack2vision);
	void Aim_LockDetection(void);
};



#endif
