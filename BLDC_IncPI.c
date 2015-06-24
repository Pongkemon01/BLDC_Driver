/* BLDC_DiffPI.c */
/* Implementation of different-mode PI controller for BLDC speed controller */
#include "BLDC_Closed_loop.h"

/* Number of bits to shift in scaling. Thus, the scaling factor of 8 makes
the actual value to 2^14 = 16384 */
#define SCALING_FACTOR 14

/* K_P and K_I here are effective values instead of theoretical ones because
the theoretical values are usually floating-point numbers. We tried to avoid
floating-point arithmetic as it consumes a lot of resources. Therefore, we
use scaled integer instead. They are derived as:
	- Let K_P and K_I are theoretical values of Kp and Ki respectively.
	- Let T_S is the control loop interval (sampling period). In this system,
	  it is 10ms
	- Let T_I is the recovery time constant (in second). This value will be
	  calculated in tuning process.
	  
	Then:
	EFF_K_P = K_P * ( 2 ^ SCALING_FACTOR )
	EFF_K_I = ( K_P * T_S / T_I ) * ( 2 ^ SCALING_FACTOR )
	
	Both EFF_K_P and EFF_K_I are rounded to be integers.
	
The EFF_K_P and EFF_K_I are pre-calculated by hand during tuning process. */
//#define EFF_K_P		1474
//#define EFF_K_I		3
#define EFF_K_P		1474
#define EFF_K_I		5

/* Maximum number */
#define MAX_PWM_CHG		1000
#define MAX_INT         INT16_MAX
#define MAX_LONG        INT32_MAX
#define MAX_I_TERM      (MAX_LONG >> 1)

/* Variables */
static int32_t prev_err;

/************************************************************************
*                                                                       *
*      Function: 	     PWMControlEngineInit                           *
*                                                                       *
*      Description:      Initialize the closed-loop control engine.     *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void PWMControlEngineInit()
{
	prev_err = 0;
}

/************************************************************************
*                                                                       *
*      Function: 	     PWMControlEngine					            *
*                                                                       *
*      Description:      Implementation of the different-mode PI        *
*                        controller.                                    *
*                                                                       *
*      Parameters:       current_control_value : Current process value  *
*						 set_point : Desired speed (absolute value)     *
*                        process_value : current BLDC speed             *
*      Return value:     Next control value used as the next pwm value. *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

int16_t PWMControlEngine( int16_t current_control_value, int16_t set_point, int16_t process_value )
{
	int32_t err;
	int16_t dpwm;
	
	err = ( int32_t )( set_point - process_value );
	dpwm = ( int16_t )( ( ( EFF_K_P * ( err - prev_err ) ) + ( EFF_K_I * err ) ) >> SCALING_FACTOR );
	prev_err = err;
	
	/* Limit the range of dpwm */
	if( dpwm > MAX_PWM_CHG )
		dpwm = MAX_PWM_CHG;
	else if( dpwm < ( -MAX_PWM_CHG ) )
		dpwm = -MAX_PWM_CHG;
	
	return( current_control_value + dpwm );
}
 
/*---------------------------------------------------------------------------*/
/*===========================================================================*/
