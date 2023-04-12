//�����ر�����
#include "DEV_BrtEncoder.h"

void Encider_Brt::Init(void)
{
	encode_count = 0;
	lastAngle = 0;
	realAngle = 0;
	totolAngle = 0;
	
	falsh = 1;    //1��������     -1�Ƿ���
}

void Encider_Brt::getInfo(uint8_t can_rx_data[])
{
	/* IDȷ�� */
	int32_t encode_err = 0;
	switch(can_rx_data[2])//[ָ�� FUNC]
	{
		//������ID_�Ƿ����óɹ�
		case 0x01:
		if(can_rx_data[0] != 0x07 || can_rx_data[1] != ID){return;}
		if(ReadMode == 0x01)
		{
			ReadMode = 0xAA;
		}
		realAngle = (uint32_t)(can_rx_data[5]<<16 | can_rx_data[4]<<8 |	can_rx_data[3]);
		UpData();
		break;

		//������ID_�Ƿ����óɹ�
		case 0x02: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x08){return;}
		if(can_rx_data[3] != 0x00 && ID==1){ID=2;}else{ID=3;}
		break;
		
		//CAN ͨѶ������_�Ƿ����óɹ�
		case 0x03: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Can_BaudRate==1){Can_BaudRate=2;}else{Can_BaudRate=3;}
		break;

		//������ģʽ_�Ƿ����óɹ�
		case 0x04: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Mode==1){Mode=2;}else{Mode=3;}
		break;

		//�������Զ��ش�ʱ��_�Ƿ����óɹ�
		case 0x05: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && AutoTime==1){AutoTime=2;}else{AutoTime=3;}
		break;

		//��ǰλ��ֵΪ���_�Ƿ����óɹ�
		case 0x06: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && ZeroSpot==1){ZeroSpot=2;}else{ZeroSpot=3;}
		break;

		//������ֵ��������_�Ƿ����óɹ�
		case 0x07: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Direction==1){Direction=2;}else{Direction=3;}
		break;

		//�������е�_�Ƿ����óɹ�
		case 0x0C: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && Midpoint==1){Midpoint=2;}else{Midpoint=3;}
		break;

		//��������ǰλ��ֵ��_�Ƿ����óɹ�
		case 0x0D: 
		if(can_rx_data[0] != 0x04 || can_rx_data[1] != 0x01){return;}
		if(can_rx_data[3] != 0x00 && CurrentLocation==1){CurrentLocation=2;}else{CurrentLocation=3;}
		break;

		//���õ�ǰֵΪ 5 Ȧֵ_�Ƿ����óɹ�
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
	totolAngle = realAngle + (encode_count * 32768*24);//24ΪĿǰ�������ɻ���Ȧ��
	
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

//	Data[0] = 0x04; 	 /* ���ݳ��� */
//	Data[1] = ID;  /* ��������ַ*/
//	Data[2] = func;  	 /* canָ�� */

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

	Data[0] = 0x04; 	 /* ���ݳ��� */
	Data[1] = ID;  /* ��������ַ*/
	Data[2] = func;  	 /* canָ�� */
	Data[3] = data;
	CANx_SendData(hcan, ID, Data, 8);
}

void Encider_Brt::SetInstruction_U16(CAN_HandleTypeDef *hcan, uint8_t func, uint16_t data)
{
	uint8_t Data[8] = {0};
	if(func == 0x05)
	{
		AutoTime = 1;
		Data[0] = 0x05; 	 /* ���ݳ��� */
		Data[1] = ID;  /* ��������ַ*/
		Data[2] = func;  	 /* canָ�� */
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
//		Data[0] = 0x07; 	 /* ���ݳ��� */
//		Data[1] = ID;  /* ��������ַ*/
//		Data[2] = can_id;  	 /* canָ�� */
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
