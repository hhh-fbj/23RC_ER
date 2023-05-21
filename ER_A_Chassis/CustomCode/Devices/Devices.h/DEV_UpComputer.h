#ifndef __DEV_UPCOMPUTER_H
#define __DEV_UPCOMPUTER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
class UpperComputer_classdef
{
private:
    int8_t ANO_SendBuff[35] = {0};
    int32_t ANO_UserBuff[8] = {0};

    unsigned char Mini_Buffer[42] = {0}; // 数据发送缓冲区
public:
    UpperComputer_classdef();
    /* ANO 上位机*/
    void ANO_DataShow(float data1, float data2, float data3, float data4);
    void ANO_DataShow_F2(float data1, float data2, float data3, float data4);
    /* MiniBalance 上位机 */
    void Mini_Float2Byte(float *target, unsigned char *buf, unsigned char beg);
    void Mini_addData(float Data, unsigned char Channel);
    void Mini_Datashow(int ChannelAmount);
    unsigned char Mini_DataGenerate(unsigned char Channel_Number);

};

extern UpperComputer_classdef UpperComputer;


#endif
