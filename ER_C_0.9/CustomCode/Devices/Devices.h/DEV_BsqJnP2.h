#ifndef __DEV_BSQJNP2_H
#define __DEV_BSQJNP2_H

#include "include.h"
#include "usart.h"

#define BSQJNP2_NUM 13

#ifdef __cplusplus


class BsqJnP2_Classdef
{
private:
  
  uint8_t ID = 1;
public:
	BsqJnP2_Classdef(){};
  //AD转换原码 int 
  //float
  //uint32_t
	uint8_t RecData[BSQJNP2_NUM];
	int Pressure[2];
  void Read_Float(void);
  void ZeroCal(void);
  void FullCal(uint32_t cor);
  void PointSet(uint32_t point);
  void IDSet(uint32_t id);
  void BaudSet(uint32_t rate);
  void ZeroClean(void);
	void Receive(uint8_t *data);
};

#endif

#endif

