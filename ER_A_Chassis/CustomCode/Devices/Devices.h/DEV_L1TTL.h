#ifndef __DEV_L1TTL_H
#define __DEV_L1TTL_H

#include <stdint.h>

#define DRF_REC_SIZE 20


class L1TTL_classdef
{
private:
	void L1TTL_Printf(const char *str, ...);
public: 
	uint8_t REC_REC_DATA[DRF_REC_SIZE];
	L1TTL_classdef();
	void Recevice(void);
	void control(void);
};

#endif