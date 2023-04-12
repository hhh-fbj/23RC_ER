#ifndef __DEV_BRTENCODER_H
#define __DEV_BRTENCODER_H

#include "include.h"
#include "BSP_CAN.h"

#define ABS(x) (x)>0?(x):(-x)

#ifdef __cplusplus

class Encider_Brt
{
private:
  uint32_t realAngle;
  uint32_t lastAngle;
  int32_t totolAngle;

  
  int32_t encode_lastcount;			//上次的圈数

  float falsh; 		//写入falsh数据			1：没反			0：反了（0xff）

  uint8_t ID;

  uint8_t ID_Flag;//0x02
  uint8_t Can_BaudRate;//0x03
  uint8_t Mode;//0x04
  uint8_t AutoTime;//0x05
  uint8_t ZeroSpot;//0x06
  uint8_t Direction;//0x07
  uint8_t Midpoint;//0x0C
  uint8_t CurrentLocation;//0x0D
  uint8_t FiveCircle_Value;//0x0F
public:
	uint8_t ReadMode = 0x01;
	int32_t encode_count;					//这次的圈数
  Encider_Brt(uint8_t id): ID(id){};
	void Init(void);
  void getInfo(uint8_t can_rx_data[]);
  void UpData(void);
  int32_t getTotolAngle(void);
  void setFalsh(float fal);
  float getFalsh(void);
//  void setEncodeCount(int32_t Count);

//void SetInstruction(CAN_HandleTypeDef *hcan, uint8_t func);
	void SetInstruction_U8(CAN_HandleTypeDef *hcan, uint8_t func, uint8_t data);
  void SetInstruction_U16(CAN_HandleTypeDef *hcan, uint8_t func, uint16_t data);
//  void SetInstruction(CAN_HandleTypeDef *hcan, uint8_t func, uint32_t data);
};

#endif

#endif

