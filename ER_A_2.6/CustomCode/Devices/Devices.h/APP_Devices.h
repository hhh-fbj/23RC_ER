#ifndef __APP_DEVICES_H
#define __APP_DEVICES_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Macro Definitions ---------------------------------------------------------*/
#define DR16_MONITOR	(1<<0)  // DR16
#define CAN1_MONITOR	(1<<1)	// CAN1
#define CAN2_MONITOR	(1<<2)	// CAN2
#define CHAS_RUD1_MONITOR		(1<<3) // 驱动轮电机 2006  
#define CHAS_RUD2_MONITOR		(1<<4) // 驱动轮电机 2006  
#define CHAS_RUD3_MONITOR		(1<<5) // 驱动轮电机 2006  
#define CHAS_RUD4_MONITOR		(1<<6) // 驱动轮电机 2006  
#define CHAS_RUDEncider1_MONITOR	(1<<7) // 驱动轮编码器 
#define CHAS_RUDEncider2_MONITOR	(1<<8) // 驱动轮编码器  
#define CHAS_RUDEncider3_MONITOR	(1<<9) // 驱动轮编码器  
#define CHAS_RUDEncider4_MONITOR	(1<<10) // 驱动轮编码器  
#define CHAS_CUSARTA_MONITOR	(1<<11) // 
#define CHAS_POSTURE_MONITOR	(1<<12) // 
#define CHAS_ANALOG_MONITOR		(1<<13) // 


#define On_line   0
#define Off_line  1


#define CRITICAL_VAL_INIT \
{\
  	1,1,1,1,\
	1,1,1,1,\
	1,1,1,1,\
	1,1,\
};


/* Private type --------------------------------------------------------------*/
enum FrameType_e
{
  	Frame_DR16,
	Frame_CAN1,
	Frame_CAN2,
	Frame_CHAS_RUD1,
	Frame_CHAS_RUD2,
	Frame_CHAS_RUD3,
	Frame_CHAS_RUD4,
	Frame_CHAS_RUDEncider1,
	Frame_CHAS_RUDEncider2,
	Frame_CHAS_RUDEncider3,
	Frame_CHAS_RUDEncider4,
	Frame_CHAS_CUSARTA,
	Frame_CHAS_POSTURE,
	Frame_CHAS_ANALOG,
	FrameCount_NUM
};

typedef struct
{
	uint32_t Now;  //当前世界时间
	uint32_t Pre;  //上一次世界时间
}WorldTime_t;

/* Exported types ------------------------------------------------------------*/
class DevicesMonitor_classdef
{
private:
    static uint16_t Critical_Val[FrameCount_NUM];        /*<! 掉帧临界值 */


public:
	    static uint32_t Devices_Frame;                         /*<! 用于检测设备是否离线 */
    uint16_t FrameCounter[FrameCount_NUM] = {0};         /*<! 将设备帧率存进数组 */
    void Devices_Detec(void);                              /*<! 设备帧率检测 */
	void Update(FrameType_e device);
    uint8_t Frame_Detec(uint16_t counter, uint16_t len);   /*<! 帧率值检测 */
    uint8_t Get_State(uint32_t device);          /*<! 获取设备状态 */
	void Get_FPS(WorldTime_t *time, uint16_t *FPS);		   /*<! 获取帧率 */
	uint16_t FPS_Calc(uint16_t delta_time)				   /*<! 计算帧率 */
	const {return (1.0f / (double)(delta_time)) * 1000.0f;} // 别忘了先转换为浮点数，否则会有精度丢失
};

#endif
