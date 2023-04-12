#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "DEV_BrtEncoder.h"
#include "DEV_Bldc.h"

#ifdef __cplusplus

//#pragma diag_suppress 550
#define AusartC_SIZE 21
/* Exported types ------------------------------------------------------------*/

#pragma pack(1)
typedef struct
{
	uint8_t start_tag;		/*<! 帧头'S' */
	uint16_t mode;			/*<! 视觉模式 */
	float speed[4];
	uint8_t crc;
	uint8_t end_tag;		/*<! 帧尾'E' */

}AusartCRecv_Pack_t;
#pragma pack()

typedef union 
{
	uint8_t data[AusartC_SIZE];
	AusartCRecv_Pack_t Pack;
}AusartCRecvMsg_u;

typedef union 
{
	float speed[2];
	uint8_t data[8];
}ReceiveSpeed__u;

//四轮
//右上 逆时针旋转
/* --- 底盘驱动电机 ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_Drv = 0,LF_Drv = 1,LB_Drv = 2,RB_Drv = 3
};


/*
    |(1)12      |(1)11
    |(2)13      |(3)14
*/
class Chassis_classdef
{
private:
    float time;
    float Tar_speed[4];
    uint8_t CRCBuf;
    unsigned char  Checksum_CRC8(uint8_t *buf,uint16_t len);
    float Three_Speed[4];

    ReceiveSpeed__u RS1;
    ReceiveSpeed__u RS2;
public:
    Chassis_classdef();
    AusartCRecvMsg_u Recv_Msg; 

    //电机驱动
    BldcDrive_VESC DRV_Motor[4] = {BldcDrive_VESC(11), BldcDrive_VESC(12), BldcDrive_VESC(13), BldcDrive_VESC(14)};/*<! 驱动轮 本杰明电调  */
	
    //PID
    IncrementPID DRV_PID[4]; 

    void Control();
    void Receive_AusartC();
    void CAN_Recv_RFLB(uint8_t can_rx_data[]);
    void CAN_Recv_LFRB(uint8_t can_rx_data[]);
};
#endif

#endif
