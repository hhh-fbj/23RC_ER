#ifndef __APP_DR16_H
#define __APP_DR16_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include <stdint.h>
/* Private type --------------------------------------------------------------*/
typedef struct
{
    float Target_Vx;
    float Target_Vy;
    float Target_Vw;

    float Target_Yaw;
    float Target_Pit;
}DR16Export_t;

typedef struct
{
    float Vx_RC;
    float Vy_RC;
    float Vz_RC;

    float Yaw_RC;
    float Pit_RC;
    float Yaw_PC;
    float Pit_PC;

}ExpVal_Coefficient_e;

/* Exported types ------------------------------------------------------------*/
class CTRL_DR16_classdef
{
private:
    ExpVal_Coefficient_e Coe; /*<! 数据系数 */

protected:
    
public:
    uint8_t start = 0;
    DR16Export_t Expt;         /*<! 对外输出数据 */

    LowPassFilter Yaw_LPF = LowPassFilter(0.1f); /*<! 低通滤波 */
    LowPassFilter Vw_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */

    CTRL_DR16_classdef();

    void LeverMode_Update();   //--- 拨杆模式更新

    void RCCtrl_Update();  //--- 遥控器模式
    void ExptData_Reset();  //--- 输出数据重置

    int8_t Data_Monitor();  //--- DR16数据监测

    /*<! 获取对外输出参数 */
    float Get_ExptVx(); 
    float Get_ExptVy();
    float Get_ExptVw();
    float Get_ExptYaw();
    float Get_ExptPit();
    uint8_t IsAnyOutput();
};

#endif
