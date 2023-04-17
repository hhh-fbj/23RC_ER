//布瑞特编码器
#include "DEV_BrtEncoder.h"

void Encider_Brt::Init(void)
{
	encode_count = 0;
	lastAngle = 0;
	realAngle = 0;
	totolAngle = 0;
	
	falsh = 1;    //1则是正向     -1是反向
}

void Encider_Brt::getInfo(uint8_t can_rx_data[])
{
	/* ID确认 */
	int32_t encode_err = 0;
	switch(can_rx_data[2])//[指令 FUNC]
	{
		//编码器ID_是否设置成功
		case 0x01:
		if(can_rx_data[0] != 0x07 || can_rx_data[1] != ID){return;}
		realAngle = (uint32_t)(can_rx_data[5]<<16 | can_rx_data[4]<<8 |	can_rx_data[3]);
		UpData();
		break;

		//编码器ID_是否设置成功
		case 0x02: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x08){return;}
		if(can_rx_data[3] != 0x00 && ID==1){ID=2;}else{ID=3;}
		break;
		
		//CAN 通讯波特率_是否设置成功
		case 0x03: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Can_BaudRate==1){Can_BaudRate=2;}else{Can_BaudRate=3;}
		break;

		//编码器模式_是否设置成功
		case 0x04: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Mode==1){Mode=2;}else{Mode=3;}
		break;

		//编码器自动回传时间_是否设置成功
		case 0x05: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && AutoTime==1){AutoTime=2;}else{AutoTime=3;}
		break;

		//当前位置值为零点_是否设置成功
		case 0x06: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && ZeroSpot==1){ZeroSpot=2;}else{ZeroSpot=3;}
		break;

		//编码器值递增方向_是否设置成功
		case 0x07: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Direction==1){Direction=2;}else{Direction=3;}
		break;

		//编码器中点_是否设置成功
		case 0x0C: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Midpoint==1){Midpoint=2;}else{Midpoint=3;}
		break;

		//编码器当前位置值点_是否设置成功
		case 0x0D: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && CurrentLocation==1){CurrentLocation=2;}else{CurrentLocation=3;}
		break;

		//设置当前值为 5 圈值_是否设置成功
		case 0x0F: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && FiveCircle_Value==1){FiveCircle_Value=2;}else{FiveCircle_Value=3;}
		break;
	}
}


void Encider_Brt::UpData(void)
{
//	if(realAngle > 204800)
//	{
//		return;
//	}

	if((int32_t)(realAngle - lastAngle) > 2000000)
	{
		encode_count --;
	}
	else if((int32_t)(realAngle - lastAngle) < -2000000)
	{
		encode_count ++;
	}

	lastAngle = realAngle;
	totolAngle = realAngle + (encode_count * 32768*24);
	// if(encode_lastcount != encode_count)
	// {
	// 	falsh = - falsh;
	// 	encode_lastcount = encode_count;
	// }
}


	//void Encider_Brt::SetInstruction(CAN_HandleTypeDef *hcan, uint8_t func)
//{
//	uint8_t Data[8] = {0};
//	switch(func)
//	{
//		case 0x06:ZeroSpot = 1;break;
//		case 0x0C:Midpoint = 1;Data[3]=0x01;break;
//		case 0x0F:FiveCircle_Value = 1;Data[3]=0x01;break;
//	}

//	Data[0] = 0x04; 	 /* 数据长度 */
//	Data[1] = ID;  /* 编码器地址*/
//	Data[2] = func;  	 /* can指令 */

//	CANx_SendData(hcan, ID, Data, 8);
//}

void Encider_Brt::SetInstruction_U8(CAN_HandleTypeDef *hcan, uint8_t func, uint8_t data)
{
	uint8_t Data[8] = {0};
	switch(func)
	{
		case 0x02:ID_Flag=1;break;
		case 0x03:Can_BaudRate=1;break;
		case 0x04:Mode=1;break;
		case 0x07:Direction=1;break;
	}

	Data[0] = 0x04; 	 /* 数据长度 */
	Data[1] = ID;  /* 编码器地址*/
	Data[2] = func;  	 /* can指令 */
	Data[3] = data;
	CANx_SendData(hcan, ID, Data, 8);
}

void Encider_Brt::SetInstruction_U16(CAN_HandleTypeDef *hcan, uint8_t func, uint16_t data)
{
	uint8_t Data[8] = {0};
	if(func == 0x05)
	{
		AutoTime = 1;
		Data[0] = 0x05; 	 /* 数据长度 */
		Data[1] = ID;  /* 编码器地址*/
		Data[2] = func;  	 /* can指令 */
		Data[3] = data;
		Data[4] = data>>8;//?
		CANx_SendData(hcan, ID, Data, 8);
	}
}

//void Encider_Brt::SetInstruction(CAN_HandleTypeDef *hcan, uint8_t can_id, uint32_t data)
//{
//	uint8_t Data[8] = {0};
//	if(can_id == 0x0D)
//	{
//		CurrentLocation = 1;
//		Data[0] = 0x07; 	 /* 数据长度 */
//		Data[1] = ID;  /* 编码器地址*/
//		Data[2] = can_id;  	 /* can指令 */
//		Data[3] = data;//?
//		Data[4] = data>>8;//?
//		Data[5] = data>>16;//?
//		CANx_SendData(hcan, ID, Data, 8);
//	}
//}

int32_t Encider_Brt::getTotolAngle(void)
{
	return totolAngle;
}
void Encider_Brt::setFalsh(float fal)
{
	falsh = fal;
}
float Encider_Brt::getFalsh(void)
{
	return falsh;
}
//void Encider_Brt::setEncodeCount(int32_t Count)
//{
//	encode_count = Count;
//}
