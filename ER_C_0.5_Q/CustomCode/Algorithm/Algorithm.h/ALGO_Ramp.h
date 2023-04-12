#ifndef __ALGO_RAMP_H_
#define __ALGO_RAMP_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

//#pragma diag_suppress 1035

#ifdef __cplusplus
extern "C"{
#endif
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float count;  //��ǰ�����ٶ�ֵ
	float rate;   //ÿһ�ε��ӵ��ٶ�ֵ
	float mincount;
	float maxcount;
}SpeedRamp_t;


void CountReset(SpeedRamp_t *SpeedRamp);
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp);
float RAMP_Output( float final, float now, float ramp );

#ifdef __cplusplus
}
#endif

#endif
