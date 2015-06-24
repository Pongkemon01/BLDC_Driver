#ifndef __COMM_LOOP_NEW_H
#define __COMM_LOOP_NEW_H

#include "BLDC.h"

/* Prototypes of functions provided by comm_loop */
extern inline uint16_t avg_comm_time(void);
extern void CommLoop_Setup(void);
extern uint8_t CommLoop_Align(void); /* Excite */
extern void CommLoop_Start(void); /* Ramp up */
extern void CommLoop_Stop(void);

#endif
