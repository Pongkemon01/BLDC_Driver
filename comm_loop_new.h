#ifndef __COMM_LOOP_NEW_H
#define __COMM_LOOP_NEW_H

#define ALIGN_PWM_START		205U
#define ALIGN_PWM_STEP		51U
#define ALIGN_STEPS			6

#define comm_time	( UNEG( TMR1_comm_time.word ) )

extern doublebyte TMR1_comm_time;

/* Prototypes of functions provided by comm_loop */
extern void CommLoop_Setup(void);
extern uint8_t CommLoop_Align(void); /* Excite */
extern void CommLoop_Start(void); /* Ramp up */
extern void CommLoop_Stop(void);

#endif