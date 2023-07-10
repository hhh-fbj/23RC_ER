#ifndef __DEV_DRF1609H_H
#define __DEV_DRF1609H_H

#include <stdint.h>

#define DRF_REC_SIZE 19
#define DRF_SEN_SIZE 7


class DRF1609H_classdef
{
private:
    uint16_t Deal_Flag,Backup_Deal_Flag;
public: 
		DRF1609H_classdef();
    uint8_t DRF_REC_Data[DRF_REC_SIZE];
		uint8_t DRF_SEN_Data [DRF_SEN_SIZE];

		uint8_t de_time_flag;
		uint8_t de_time;

    void Receive(uint8_t *data);
		void DRT_Printf(const char *str, ...);
//    void Send(uint32_t idea);
		
		uint8_t DRF_UpFlag,DRF_shejijieshuFlag;
		uint8_t DRF_page0Flag,DRF_page1Flag;
		uint8_t Read_Flag;
};

#endif