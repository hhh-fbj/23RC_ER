#ifndef __DEV_BUZZER_H
#define __DEV_BUZZER_H

#include  <stdint.h>
#include  <string.h>

#define MAG_TIM htim4
#define MAG_CHANNEL TIM_CHANNEL_3

#define L1	262
#define L1U	277
#define L2	294
#define L2U	311
#define L3	330
#define L4	349
#define L4U	370
#define L5	392
#define L5U	415
#define L6	440
#define L6U	466
#define L7	494

#define M1	523
#define M1U	554
#define M2	587
#define M2U	622
#define M3	659
#define M4	698
#define M4U	740
#define M5	784
#define M5U	831
#define M6	880
#define M6U	932
#define M7	988

#define H1	1046
#define H1U	1109
#define H2	1175
#define H2U	1245
#define H3	1318
#define H4	1397
#define H4U	1480
#define H5	1568
#define H5U	1661
#define H6	1760
#define H6U	1865
#define H7	1976

typedef struct {
	uint16_t note;
	uint16_t time;
}MusicNote;

/* 蜂鸣器响铃类型 */
typedef enum
{
	Ring_Stop = 0,
	Ring_ContLong = -1,
	Ring_1time = 1,
	Ring_2times = 2,
	Ring_3times = 3,
	Ring_4times = 4,
	Ring_5times = 5,
	Ring_ContShort = 125
}RingType_e;


class Buzzer_Classdef
{
public:
    Buzzer_Classdef(){};
    ~Buzzer_Classdef(){};
			
	void Set_RingType(int8_t ringtype); //--- 设置响铃类型
	void Process();					 //--- 蜂鸣器处理
			
    void SuperMario_Music(void);
private:
		int8_t Ring_Type; //--- 响铃次数
    //马里奥乐谱
    MusicNote SuperMario[34] = {{M3, 100}, {0, 50}, {M3, 250}, {0, 50}, {M3, 100}, {0, 50}, {0, 150}, {M1, 100}, {0, 50}, {M3, 250}, {0, 50}, {M5, 250}, {0, 50}, {0, 300}, {L5, 250}, {0, 50}, {0, 300},/*{M1, 250}, {0, 50}*/};

    uint16_t notes_low[12] = {L1, L1U, L2, L2U, L3, L4, L4U, L5, L5U, L6, L6U, L7};
    uint16_t notes_mid[12] = {M1, M1U, M2, M2U, M3, M4, M4U, M5, M5U, M6, M6U, M7};
    uint16_t notes_high[12] = {H1, H1U, H2, H2U, H3, H4, H4U, H5, H5U, H6, H6U, H7};

    int8_t Init_TickTIME;
    int8_t Switch_Flag = false;

};

#endif