#ifndef __COMM_LOOP_AN1305_H
#define __COMM_LOOP_AN1305_H

/* ERROR_SCALE - Feedback scaling factor (aka., proportional factor)
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
#define ERROR_SCALE                 4

#define comm_time()	( UNEG( TMR1_comm_time.word ) )

extern doublebyte TMR1_comm_time;

/* Prototypes of functions provided by comm_loop */
extern void CommLoop_Setup(void);
extern uint8_t CommLoop_Align(void); /* Excite */
extern void CommLoop_Start(void); /* Ramp up */
extern void CommLoop_Stop(void);

#endif