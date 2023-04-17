#include "DEV_LED.h"
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "System_DataPool.h"



LED_classdef::LED_classdef()
{
    
}

void LED_classdef::Init()
{
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}


//蓝 -> 绿(灭) -> 红 -> 蓝(灭) -> 绿 -> 红(灭) -> 蓝
void LED_classdef::BLN_Ctrl()
{

    for(uint8_t i = 0 ; i < BLN_LENGHT ; i++)
    {
        alpha = (BLN_Color[i] & 0xFF000000) >> 24;
        red = ((BLN_Color[i] & 0x00FF0000) >> 16);
        green = ((BLN_Color[i] & 0x0000FF00) >> 8);
        blue = ((BLN_Color[i] & 0x000000FF) >> 0);

        delta_alpha = (float)((BLN_Color[i + 1] & 0xFF000000) >> 24) - (float)((BLN_Color[i] & 0xFF000000) >> 24);
        delta_red = (float)((BLN_Color[i + 1] & 0x00FF0000) >> 16) - (float)((BLN_Color[i] & 0x00FF0000) >> 16);
        delta_green = (float)((BLN_Color[i + 1] & 0x0000FF00) >> 8) - (float)((BLN_Color[i] & 0x0000FF00) >> 8);
        delta_blue = (float)((BLN_Color[i + 1] & 0x000000FF) >> 0) - (float)((BLN_Color[i] & 0x000000FF) >> 0);

        delta_alpha /= BLE_CHANGE_TIME;
        delta_red /= BLE_CHANGE_TIME;
        delta_green /= BLE_CHANGE_TIME;
        delta_blue /= BLE_CHANGE_TIME;

        for(uint16_t j = 0 ; j < BLE_CHANGE_TIME ; j++)
        {

            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            RGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
            BLN_Display(RGB);

            osDelay(1);
        }
    }

}

void LED_classdef::BLN_Display(uint32_t rgb)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (rgb & 0xFF000000) >> 24;
    red = ((rgb & 0x00FF0000) >> 16) * alpha;
    green = ((rgb & 0x0000FF00) >> 8) * alpha;
    blue = ((rgb & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
