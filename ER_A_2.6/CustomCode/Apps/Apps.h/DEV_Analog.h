#ifndef __DEV_ANALONG_H
#define __DEV_ANALONG_H

#include "include.h"

#define LASER_NUM 18
#define REC_DATA_NUM (3+LASER_NUM*2+2)

//输入寄存器——AI 模拟量输入——可读
//PLC 地址30001-39999 协议地址0000H-FFFFH
//功能码04H
class Analog_Classdef
{
private:
	uint8_t ID;
  uint8_t flag;
public:
	Analog_Classdef(uint8_t id):ID(id){};
	float LaserRanging[LASER_NUM];
  uint8_t ReceiveData[REC_DATA_NUM];//3+2*8+2
  void ReadAll(void);
  void Receive(uint8_t *datass);
};


#endif