#ifndef __APP_DR16_H
#define __APP_DR16_H

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include <stdint.h>
/* Private type --------------------------------------------------------------*/
typedef struct
{
	float L_X;
	float L_Y;
	float R_X;
	float R_Y;
	float D_W;
}DR16Export_t;

enum RCCtrl_UpdateMode_e
{
	RCCtrl_Update_DisableMode=0,
	RCCtrl_Update_GimbalMode=1,
	RCCtrl_Update_ShootMode=2,
	RCCtrl_Update_PullMode=3,
	RCCtrl_Update_AllShootMode=4,
	RCCtrl_Update_ClampMode=5,
	RCCtrl_Update_AutoMode=6,
};

/* Exported types ------------------------------------------------------------*/
class CTRL_DR16_classdef
{
private:
    bool LX_Zero;
    bool LY_Zero;
    bool RX_Zero;
    bool RY_Zero;
    bool DW_Zero;
		
		RCCtrl_UpdateMode_e Mode;
protected:
    float Yaw_Turn = 0.006;
public:
    uint8_t start;
    DR16Export_t Expt;         /*<! 对外输出数据 */
    

    LowPassFilter MouseX_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */
    LowPassFilter MouseY_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */
    LowPassFilter Yaw_LPF = LowPassFilter(0.1f); /*<! 低通滤波 */
    LowPassFilter Vw_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */


    CTRL_DR16_classdef();

    void LeverMode_Update();   //--- 拨杆模式更新

    void RCCtrl_Update();  //--- 遥控器模式
    void ExptData_Reset();  //--- 输出数据重置

    int8_t Data_Monitor();  //--- DR16数据监测

    /*<! 获取对外输出参数 */
    float Get_LX(); 
    float Get_LY();
    float Get_RX();
    float Get_RY();
    float Get_DW();
    uint8_t IsAnyOutput();
};

#endif