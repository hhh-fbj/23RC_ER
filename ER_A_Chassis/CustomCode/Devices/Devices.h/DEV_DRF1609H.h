#ifndef __DEV_DRF1609H_H
#define __DEV_DRF1609H_H

#include <stdint.h>

#define TJC4827X343_SIZE 5

class DRF1609H_classdef
{
private:
    uint16_t Deal_Flag,Backup_Deal_Flag;
public: 
	DRF1609H_classdef();
    uint8_t TJC4827X343_Data[TJC4827X343_SIZE];
    void Receive(uint8_t *data);
    void TJCPrintf(const char *str, ...);
    void communication(void);
};

#endif