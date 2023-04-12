#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

#define LOW_BATTER_VOLTAGE   21.9f

#ifdef  __cplusplus
extern "C"{
#endif
extern void init_vrefint_reciprocal(void);
extern float get_temprate(void);
extern float get_battery_voltage(void);
extern uint8_t get_hardware_version(void);
#ifdef  __cplusplus
}
#endif
#endif
