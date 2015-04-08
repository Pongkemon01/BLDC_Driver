#ifndef __COMM_LOOP_NEW_H
#define __COMM_LOOP_NEW_H

/* Alignment constants */
#define ALIGN_PWM_START		205U
#define ALIGN_PWM_STEP		51U
#define ALIGN_STEPS			6

/* History length for speed averaging */
#define AVG_BACKLOG_LEN		3	/* Length is 2^(AVG_BACKLOG_LEN) */
#define AVG_BACKLOG_SIZE	( 1 << AVG_BACKLOG_LEN )
#define AVG_BACKLOG_MASK	( AVG_BACKLOG_SIZE - 1 )

#define comm_time()		( UNEG( TMR1_comm_time.word ) )
#define avg_comm_time()	( AvgCommTime() )

extern doublebyte TMR1_comm_time;

/* Prototypes of functions provided by comm_loop */
extern uint16_t AvgCommTime(void);
extern void CommLoop_Setup(void);
extern uint8_t CommLoop_Align(void); /* Excite */
extern void CommLoop_Start(void); /* Ramp up */
extern void CommLoop_Stop(void);

#endif
