#include "DEV_MX106.h"
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "System_DataPool.h"

//3595

uint8_t MX106_Data[MXDATASIZE];
void MX106_classdef::Ping(void)
{
  uint16_t crc;
  send_data[0] = 0XFF;
  send_data[1] = 0XFF;
  send_data[2] = 0XFD;
  send_data[3] = 0X00;
  send_data[4] = ID;
  send_data[5] = 0X03;
  send_data[6] = 0X00;
  send_data[7] = 0X01;
  crc = update_crc(0, send_data, 8);
  send_data[8] = (crc & 0xFF);
  send_data[9] = ((crc>>8) & 0xFF);
  send_data[10] = 0X00;
  send_data[11] = 0X00;
  send_data[12] = 0X00;
  send_data[13] = 0X00;
  send_data[14] = 0X00;
  send_data[15] = 0X00;

	switch(ID)
	{
		case 1:
			if(Tyg)
			{
				if((&huart7)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart7, send_data, 16);
				}
			}
			else
			{
				if((&huart6)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart6, send_data, 16);
				}
			}
		break;
			
		case 2:
			if((&huart7)->gState == HAL_UART_STATE_READY) 
			{
				HAL_UART_Transmit_DMA(&huart7, send_data, 16);
			}
		break;
	}
  State = 0X01;
}


uint8_t set_Torque_state;
uint16_t Recrc;
void MX106_classdef::Receive(uint8_t *data)
{
  switch(State)
  {
//	HAL_HalfDuplex_EnableReceiver(&huart6);
//	HAL_UART_Receive(&huart6, data2, 32, 100);
    case 0x01:
      State = data[8];
    break;

    case 0x03:
      State = data[8];
      if(State ==  0X00){Torque_Flag = set_Torque_state;}
      else{Torque_Flag = 99;}
    break;

    case 0x04:
      State = data[8];
      if(State ==  0X00){Error = 0;}
      else{Error = 1;}
    break;

    case 0x06:
      State = data[8];
      Recrc = update_crc(0, data, 13);
      if(data[13] == (uint8_t)(Recrc & 0xFF) && data[14] == (uint8_t)((Recrc>>8) & 0xFF))
      {
        if(State == 0)
        {
          Posotion = data[12]<<24 | data[11]<<16 | data[10]<<8 | data[9];
          Error = 0;
        }
      }
      else
      {
        Error = 1;
      }
    break;
    }
}

void MX106_classdef::Torque(uint8_t NON)
{
    uint16_t crc;
    send_data[0] = 0XFF;
    send_data[1] = 0XFF;
    send_data[2] = 0XFD;
    send_data[3] = 0X00;
    send_data[4] = ID;
    send_data[5] = 0X06;
    send_data[6] = 0X00;
    send_data[7] = 0X03;
    send_data[8] = 0X40;
    send_data[9] = 0X00;
    send_data[10] = NON;
    crc = update_crc(0, send_data, 11);
    send_data[11] = (crc & 0xFF);
    send_data[12] = ((crc>>8) & 0xFF);
    send_data[13] = 0X00;
    send_data[14] = 0X00;
    send_data[15] = 0X00;
	switch(ID)
	{
		case 1:
			if(Tyg)
			{
				if((&huart7)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart7, send_data, 16);
				}
			}
			else
			{
				if((&huart6)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart6, send_data, 16);
				}
			}
		break;
			
		case 2:
			if((&huart7)->gState == HAL_UART_STATE_READY) 
			{
				HAL_UART_Transmit_DMA(&huart7, send_data, 16);
			}
		break;
	}
    set_Torque_state = NON;
    State = 0X03;
}

void MX106_classdef::Position_Control(uint32_t position)
{
    uint16_t crc;
    send_data[0] = 0XFF;
    send_data[1] = 0XFF;
    send_data[2] = 0XFD;
    send_data[3] = 0X00;
    send_data[4] = ID;
    send_data[5] = 0X09;
    send_data[6] = 0X00;
    send_data[7] = 0X03;
    send_data[8] = 0X74;
    send_data[9] = 0X00;

    send_data[10] = (uint8_t)position;
    send_data[11] = (uint8_t)(position>>8);
    send_data[12] = (uint8_t)(position>>16);
    send_data[13] = (uint8_t)(position>>24);

    crc = update_crc(0, send_data, 14);
    send_data[14] = (crc & 0xFF);
    send_data[15] = ((crc>>8) & 0xFF);
		// HAL_UART_Transmit(&huart6, send_data, 16, 2000);
	switch(ID)
	{
		case 1:
			if(Tyg)
			{
				if((&huart7)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart7, send_data, 16);
				}
			}
			else
			{
				if((&huart6)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart6, send_data, 16);
				}
			}
		break;
			
		case 2:
			if((&huart7)->gState == HAL_UART_STATE_READY) 
			{
				HAL_UART_Transmit_DMA(&huart7, send_data, 16);
			}
		break;
	}
    State = 0X04;
}

void MX106_classdef::ExtendPosition_Control(uint32_t position)
{
    uint16_t crc;
    send_data[0] = 0XFF;
    send_data[1] = 0XFF;
    send_data[2] = 0XFD;
    send_data[3] = 0X00;
    send_data[4] = ID;
    send_data[5] = 0X09;
    send_data[6] = 0X00;
    send_data[7] = 0X03;
    send_data[8] = 0X74;
    send_data[9] = 0X00;

    send_data[10] = (uint8_t)position;
    send_data[11] = (uint8_t)(position>>8);
    send_data[12] = (uint8_t)(position>>16);
    send_data[13] = (uint8_t)(position>>24);

    crc = update_crc(0, send_data, 14);
    send_data[14] = (crc & 0xFF);
    send_data[15] = ((crc>>8) & 0xFF);
		// HAL_UART_Transmit(&huart6, send_data, 16, 2000);
	switch(ID)
	{
		case 1:
			if(Tyg)
			{
				if((&huart7)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart7, send_data, 16);
				}
			}
			else
			{
				if((&huart6)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart6, send_data, 16);
				}
			}
		break;
			
		case 2:
			if((&huart7)->gState == HAL_UART_STATE_READY) 
			{
				HAL_UART_Transmit_DMA(&huart7, send_data, 16);
			}
		break;
	}
		
		
    State = 0X04;
}

void MX106_classdef::ReadPosition()
{
    uint16_t crc;
    send_data[0] = 0XFF;
    send_data[1] = 0XFF;
    send_data[2] = 0XFD;
    send_data[3] = 0X00;
    send_data[4] = ID;
    send_data[5] = 0X07;
    send_data[6] = 0X00;
    send_data[7] = 0X02;

    send_data[8] = 0x84;
    send_data[9] = 0x00;
    send_data[10] = 0x04;
    send_data[11] = 0x00;

    crc = update_crc(0, send_data, 12);
    send_data[12] = (crc & 0xFF);
    send_data[13] = ((crc>>8) & 0xFF);
    send_data[14] = 0X00;
    send_data[15] = 0X00;
//		HAL_UART_Transmit(&huart6, send_data, 16, 2000);
	switch(ID)
	{
		case 1:
			if(Tyg)
			{
				if((&huart7)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart7, send_data, 16);
				}
			}
			else
			{
				if((&huart6)->gState == HAL_UART_STATE_READY) 
				{
					HAL_UART_Transmit_DMA(&huart6, send_data, 16);
				}
			}
		break;
			
		case 2:
			if((&huart7)->gState == HAL_UART_STATE_READY) 
			{
				HAL_UART_Transmit_DMA(&huart7, send_data, 16);
			}
		break;
	}
    State = 0X06;
}

void MX106_classdef::Read(){;}
void MX106_classdef::Write(){;}


//crc_accum 0
//*data_blk_ptr 数据包指针
/*
data_blk_size 数据包中不包括CRC的字节数
data_blk_size = 标头（3）+ 保留（1）+ 数据包ID（1）+ 数据包长度（2）+ 数据包长度 - CRC（2） = 3 + 1 + 1 + 2 + Pakcet长度 - 2 = 5 + 数据包长度;
数据包长度 = （LEN_H « 8 ） + LEN_L;
*/
unsigned short MX106_classdef::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}