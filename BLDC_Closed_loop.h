#ifndef __BLDC_CLOSED_LOOP
#define __BLDC_CLOSED_LOOP

#include <stdint.h>

extern void PWMControlEngineInit();
extern int16_t PWMControlEngine( int16_t current_control_value, int16_t set_point, int16_t process_value );

#endif
