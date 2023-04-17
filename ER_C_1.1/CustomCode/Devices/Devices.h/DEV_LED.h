#ifndef __DEV_LED_H
#define __DEV_LED_H

#include <stdint.h>

#define BLE_CHANGE_TIME  1000  //--- 呼吸灯变换间隔
#define BLN_LENGHT   6  //--- 呼吸灯数组长度

class LED_classdef
{
private: 
    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha,red,green,blue;
    uint32_t BLN_Color[BLN_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
    uint32_t RGB;

public:
    LED_classdef();

    void Init();
    void BLN_Ctrl();    /*<! C板呼吸灯控制 */
    void BLN_Display(uint32_t rgb); /*<! C板呼吸灯展示 */
};

#endif
