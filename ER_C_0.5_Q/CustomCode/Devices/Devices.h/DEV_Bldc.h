#ifndef __DEV_BLDC_H
#define __DEV_BLDC_H

#include "include.h"
#include "BSP_CAN.h"

#define CAN_STATUS_MSGS_TO_STORE 10

#define F_VESC_ID  10
#define L_VESC_ID  11
#define R_VESC_ID  12

//-------下列为控制指令枚举-------//
typedef enum {
CAN_PACKET_SET_DUTY = 0,
CAN_PACKET_SET_CURRENT,
CAN_PACKET_SET_CURRENT_BRAKE,
CAN_PACKET_SET_RPM,
CAN_PACKET_SET_POS,
CAN_PACKET_FILL_RX_BUFFER,
CAN_PACKET_FILL_RX_BUFFER_LONG,
CAN_PACKET_PROCESS_RX_BUFFER,
CAN_PACKET_PROCESS_SHORT_BUFFER,
CAN_PACKET_STATUS,
CAN_PACKET_SET_CURRENT_REL,
CAN_PACKET_SET_CURRENT_BRAKE_REL,
CAN_PACKET_SET_CURRENT_HANDBRAKE,
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
CAN_PACKET_STATUS_2,
CAN_PACKET_STATUS_3,
CAN_PACKET_STATUS_4,
CAN_PACKET_PING,
CAN_PACKET_PONG,
CAN_PACKET_DETECT_APPLY_ALL_FOC,
CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
CAN_PACKET_CONF_CURRENT_LIMITS,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
CAN_PACKET_CONF_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_FOC_ERPMS,
CAN_PACKET_CONF_STORE_FOC_ERPMS,
CAN_PACKET_STATUS_5
} CAN_PACKET_ID;


//---------结构体---------//////
typedef struct 
{
	int id;
	uint32_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;


typedef struct 
{
	int id;
	int rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;



typedef struct 
{
	int id;
	uint32_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct 
{
	int id;
	uint32_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;


typedef struct
{
	int id;
	uint32_t rx_time;
	float v_in;
	int32_t tacho_value;
}can_status_msg_5;

typedef struct
{
	can_status_msg    status_msg;
	can_status_msg_2  status_msg_2;
	can_status_msg_3  status_msg_3;
	can_status_msg_4  status_msg_4;
	can_status_msg_5  status_msg_5;
} BJMs_t;


class BldcDrive_VESC
{
private://can_status_msg
	uint32_t ID;
	uint32_t rx_time;
//	float rpm;
	float current;
	float duty;
	float rpm;

	void can_transmit(CAN_HandleTypeDef* CANx,uint32_t id, uint8_t *data,uint8_t len);
	void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) ;
	int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
	int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) ;
public:
	int32_t falsh = 1;
	BldcDrive_VESC(uint8_t id) : ID(id){};
	void Set_current(CAN_HandleTypeDef* CAN_Num , float current);
	void Set_duty(CAN_HandleTypeDef* CAN_Num , float duty);
	void Set_rpm(CAN_HandleTypeDef* CAN_Num , float rpm);
	void State_getInfo(uint32_t cmd,uint8_t can_rx_data[]);

//	void setFalsh(float fal);
  	float getFalsh(void);
	float getRpm(void);
};

#endif /*_BLDC_CAN_H*/



