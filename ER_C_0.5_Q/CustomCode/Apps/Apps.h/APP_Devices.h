#ifndef __APP_DEVICES_H
#define __APP_DEVICES_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Macro Definitions ---------------------------------------------------------*/
#define CAN1_MONITOR (1<<0)	
#define CAN2_MONITOR (1<<1)	
#define CHASSIS_AUSARTC_RFLB_MONITOR (1<<2)
#define CHASSIS_AUSARTC_LFRB_MONITOR (1<<3)
#define CHASSIS_DRV1_MONITOR (1<<4)	
#define CHASSIS_DRV2_MONITOR (1<<5)
#define CHASSIS_DRV3_MONITOR (1<<6)	
#define CHASSIS_DRV4_MONITOR (1<<7)	

#define On_line   0
#define Off_line  1


#define CRITICAL_VAL_INIT \
{\
  	1,1,1,1,\
	1,1,1,1,\
};



/* Private type --------------------------------------------------------------*/
enum FrameType_e
{
	Frame_CAN1 = 0,
	Frame_CAN2,
	Frame_CHAS_AUSARTC_RFLB,
	Frame_CHAS_AUSARTC_LFRB,
	Frame_CHAS_DRV1,
	Frame_CHAS_DRV2,
	Frame_CHAS_DRV3,
	Frame_CHAS_DRV4,
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
