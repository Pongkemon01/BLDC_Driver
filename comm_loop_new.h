#ifndef __COMM_LOOP_NEW_H
#define __COMM_LOOP_NEW_H

#include "BLDC.h"

/* ERROR_SCALE - Feedback scaling factor (proportional part)
   The error is the difference between the expected zero cross time and the
   actual zero cross time. The error is scaled down before it is accumulated
   into the commutation time. 2 raised to ERROR_SCALE (2^ERROR_SCALE) is 
   the scaling division factor. If the scaling factor is too large then
   the motor response will be slow. If the scaling factor is too small then
   the motor may become unstable.

   If the motor frequently misses lock after the startup sequence then
   this number is probably too small. If the motor loses lock at high speed
   or during acceleration then this number is probably too large.

   The raw error is divided by 2 to the power of ERROR_SCALE before accumulating.
   Example: If the raw error is 96 and ERROR_SCALE is 3 then
   the error correction that is accumulated is reduced to 96/2^3 or 12.
*/
#define ERROR_SCALE			4

/* INTE_SCALE and DIFF_SCALE - Feedback scaling factors (integral and differential parts)
   These factors similarly operate as ERROR_SCALE. However, each of them
   is used on separate terms from the ERROR_SCALE. INTE_SCALE is used with
   the summation of the previous error values while DIFF_SCALE is used with
   the difference between current error and the last one.
   
   If any of them is too small, the motor may be unstable. Tuning them requires
   a several trial-and-error matching. For simplicity, we should start with the
   value of 16 and adjust ERROR_SCALE until the motor becomes stable.
   Then, we decrease INTE_SCALE slowly until the motor runs smoothly in low
   to medium speed. Finally, we decrease DIFF_SCALE to make the motor runs
   smoothly in high speed.
   
   As the error is stored in signed 16-bit variable. Setting any of INTE_SCALE
   and DIFF_SCALE to the value greater than 15 will be considered as disabling
   the corresponding term.
   
   Normally, their value should be about twice of ERROR_SCALE.
*/
#define INTE_SCALE			16
#define DIFF_SCALE			16

#define MIN_TMR1_VALUE		( 0xFFFF - (TMR1_FREQUENCY / ((ABSOLUTE_MIN_RPM * COMM_PER_REV) / SEC_PER_MIN)) )

/* History length for speed averaging */
#define AVG_BACKLOG_LEN		3	/* Length is 2^(AVG_BACKLOG_LEN) */
#define AVG_BACKLOG_SIZE	( 1 << AVG_BACKLOG_LEN )
#define AVG_BACKLOG_MASK	( AVG_BACKLOG_SIZE - 1 )

#define comm_time()		( UNEG( TMR1_comm_time.word ) )
#define avg_comm_time()	( AvgCommTime() )

extern doublebyte TMR1_comm_time;
extern uint16_t pwm_current;

/* Prototypes of functions provided by comm_loop */
extern uint16_t AvgCommTime(void);
extern void CommLoop_Setup(void);
extern uint8_t CommLoop_Align(void); /* Excite */
extern void CommLoop_Start(void); /* Ramp up */
extern void CommLoop_Stop(void);

#endif
