//本杰明电调
// 0x010  0x011 0x012
#include "DEV_Bldc.h"


// BJMs_t   BJM[3];
// uint8_t  id ;
// CAN_PACKET_ID  cmd ;
// can_status_msg *stat_tmp;
// can_status_msg_2 *stat_tmp_2;
// can_status_msg_3 *stat_tmp_3;
// can_status_msg_4 *stat_tmp_4;
// can_status_msg_5 *stat_tmp_5;

#define VESC_BASEID		0x10

/**
  * @brief  设置本杰明电调占空比
  * @param[in]  id 电机对应的ID(在上位机中设置)  duty 占空比的值
  * @retval None
  */
void BldcDrive_VESC::Set_duty(CAN_HandleTypeDef* CAN_Num, float duty) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);//数据类型转换
	can_transmit(CAN_Num, ID | ((uint32_t)CAN_PACKET_SET_DUTY << 8),buffer, send_index);//通过CAN发送
	//前半部分是被控制驱动的ID号，后半部分是枚举类型中的命令。
}


/**
  * @brief  设置本杰明电调电流
  * @param[in]  id 电机对应的ID(在上位机中设置)  current 电流的值
  * @retval None
  */
void BldcDrive_VESC::Set_current(CAN_HandleTypeDef* CAN_Num , float current) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current /* * 1000 */), &send_index);
	can_transmit(CAN_Num, ID | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}


/**
  * @brief  设置本杰明电调速度
  * @param[in]  id 电机对应的ID(在上位机中设置)  rpm 速度的值
  * @retval None
  */
void BldcDrive_VESC::Set_rpm(CAN_HandleTypeDef* CAN_Num , float rpm) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(rpm)  * falsh, &send_index);
	can_transmit(CAN_Num, ID | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

/**
  * @brief  本杰明电调接收函数
  * @param[in] 
  * @retval None
  */
int a,b,c,d,e,f,g;
void BldcDrive_VESC::State_getInfo(uint32_t cmd,uint8_t can_rx_data[])
{
	int ind;
	switch((CAN_PACKET_ID)cmd)//cmd判断		
	{ 		
		case CAN_PACKET_SET_DUTY:
			a++;
			ind = 0;
		break;

		case CAN_PACKET_SET_CURRENT:
			b++;
			ind = 0;

		break;

		case CAN_PACKET_SET_CURRENT_BRAKE:
			c++;
			ind = 0;
		break;

		case CAN_PACKET_SET_RPM:
			d++;
			ind = 0;
		break;

		case CAN_PACKET_SET_POS:
			e++;
			ind = 0;
		break;

		case CAN_PACKET_STATUS:
			f++;
			ind = 0;
			rx_time = 0;//rx_time 存取
			rpm = (float)buffer_get_int32(can_rx_data, &ind); //转速
			current = (float)buffer_get_int16(can_rx_data, &ind) / 10.0f; //电流
			duty = (float)buffer_get_int16(can_rx_data, &ind) / 1000.0f; //占空比
		break;
	
		default:
			g++;
		break;
	}
}

void BldcDrive_VESC::can_transmit(CAN_HandleTypeDef* CANx,uint32_t id, uint8_t *data,uint8_t len)  
{	  
	uint8_t Data[8];
	if(len > 8){
	   len = 8;
	}
	memcpy(Data, data, len);//从CANTxFrame中复制数据
	CANx_SendData_EXT(CANx, id, Data, len);
}

int16_t BldcDrive_VESC::buffer_get_int16(const uint8_t *buffer, int32_t *index) 
{
	int16_t res =	((uint16_t) buffer[*index]) << 8 | ((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}
int32_t BldcDrive_VESC::buffer_get_int32(const uint8_t *buffer, int32_t *index) 
{
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}
void BldcDrive_VESC::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//void BldcDrive_VESC::setFalsh(float fal)
//{
//	falsh = fal;
//}
float BldcDrive_VESC::getFalsh(void)
{
	return falsh;
}
float BldcDrive_VESC::getRpm(void)
{
	return rpm;
}