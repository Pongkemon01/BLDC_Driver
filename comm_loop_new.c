/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Driver             *
*                                                                       *
*     Author               : Akrappong Patchararungruang                *
*                                                                       *
*     Date                 : 2015/02/26                                 *
*     SharePoint Version   : 1.0                                        *
*                                                                       *
*     Other Files Required :                                            *
*     Tools Used: Compiler : xc8                                        *
*                 Assembler:                                            *
*                 Linker   :                                            *
*                                                                       *
*************************************************************************
*
*************************************************************************
*
* Change History:
* Author               Date        Comment
* Akrapong            2015.02.26  V 1.0 - Original code
* Akrapong            2014.02.28  V 1.1 - Add some undocumented control commands
*
*************************************************************************
*/

/******************* System includes and definition *********************/
#include <xc.h>
#include "BLDC.h"
#include "comm_loop_new.h"

uint16_t startup_rpm = UNEG(COMM_TIME_INIT);

/* Average backlog */
#define AVG_BACKLOG_SIZE	( 1 << ( AVG_BACKLOG_LEN - 1 ) )
#define AVG_BACKLOG_MASK	( AVG_BACKLOG_SIZE - 1 )

uint16_t avg_backlog[AVG_BACKLOG_SIZE];
uint8_t avg_backlog_index;

/* Variables for ISR */
enum isr_state_t{zero_detect,commutate} isr_state;
uint8_t zc_count;

/* Zero-crossing timing state */
doublebyte TMR1_comm_time;
doublebyte zc;
doublebyte prev_zc;
doublebyte comm_after_zc;

/* Timer counters for startup */
uint8_t TMR0_excite_timer;
uint16_t current_pwm;

/* Sub-state */
uint8_t excite_events;

/*---------------------------------------------------------------------------*/

/* Prototype */

/************************************************************************
*                                                                       *
*                          I N T E R R U P T                            *
*                                                                       *
*************************************************************************/
/************************************************************************
 * Motor drive is controlled by interrupts. There are two types of interrupts:
 * Timer1 overflow and BEMF comparator.
 *
 * Timer1 interrupts:
 *    Commutation interrupt - Motor drive is changed to the next commutation
 *    phase. Every motor commutation is triggered by a Timer1 interrupt.
 *    Commutation switching occurs first followed by either zero-crossing setup
 *    or commutation time correction. When the current commutation cycle
 *    includes zero-crossing detection then inductive spike blanking is
 *    performed followed by comparator setup to catch the zero-crossing event.
 *    When the current commutation cycle does not include zero-crossing
 *    detection then the new commutation time is computed using the
 *    zero-crossing information captured during the previous commutation cycle.                                                                 //
 * Comparator interrupt:
 *    Zero Cross interrupt - This signals when the BEMF voltage on the floating
 *    motor terminal has crossed the motor supply voltage midpoint. Timer1 is
 *    read and the measured time is saved to be used to compute and correct
 *    commutation. Commutation correction computations are performed in the
 *    following commutation cycle when there is more free time.
 *
 *************************************************************************/
static void interrupt
interrupt_handler(void)
{
static uint8_t ctemp; /* Dummy variable used to clear some registers */
static doublebyte temp_comm_time;
static uint16_t expected_zc, avg_zc;
static int16_t zc_error, temp1, temp2;

	switch (isr_state)
	{
		case zero_detect:
			/* disable comparator interrupts */
			CxIE = 0;
			/* This state can be entered by either a Timer1 interrupt
			   or a comparator interrupt.
			    - If it's a comparator interrupt then zero cross has been
			      detected and needs to be processed.
			    - If it's a Timer1 interrupt then the zero cross was missed
			      so we need to skip zero cross processing and fall through
			      to commutation.
			*/

			/* Save previous zc time */
			prev_zc.word = zc.word;

			if(CxIF)
			{
				/* Interrupt is from the comparator */
				CxIF = 0;
				
				/* Capture zero-crossing time. Repeat timer read if rollover
				corrupted the capture */
				do
				{
					zc.bytes.high = TMR1H;
					zc.bytes.low = TMR1L;
				}while (zc.bytes.high != TMR1H);
				
				/* Average zc time using EMA(3) */
				if( zc.word <= TMR1_comm_time.word )
					zc.word = TMR1_comm_time.word;

				if(BLDC_State == COMMUTE)
				{
					/* During normal commutation, we adjust timer to commutate
					 one-half commutation period after zero-cross is detected
					 this provides immediate correction for early or late
					 zero-crossing and assures alignment of commutation with
					 the motor position. */
					TMR1ON = 0;
					TMR1H = comm_after_zc.bytes.high;
					TMR1L = comm_after_zc.bytes.low;
					TMR1ON = 1;
				}

				/* If we can detect contiguous BEMF in RAMPUP, we can go to
				COMMUTE state */
				if( BLDC_State == RAMPUP )
				{
					zc_count--;
					if( zc_count == 0 )
					{
						/* Turn on LED OK when entering COMMUTE */
						LED_OK = 1;
						BLDC_State = COMMUTE;
						CCP1AS = ECCP1AS_INIT; /* Activate current limit */
					}
				}

				/* Reset stall timer as we have detected BEMF */
				TMR0_stall_timer = TIMEBASE_STALL_COUNT;

				isr_state = commutate;
				break;
			}
			else
			{
				/* Interrupt is from timer1
				If execution reaches this point then a Timer1 interrupt
				occurred before zero-crossing was detected. This should only
				happen during deceleration and acceleration when searching for
				zero cross in forced commutation. */
				
				/* Corrective calculation */

				zc_count = EXPECT_ZC_COUNT;
				/* adjust the zero-crossing time to catch up with the motor */
				if(BLDC_State == COMMUTE)
				{
					/* Assume that zero-crossing at the end of commutation */
					zc.word = 0xFFFF;
				}
				/* No "break" statement here. We follow through
				"commutate" state to process timer1 interrupt. */
			}

		case commutate:
			/* Only Timer1 interrupt is handled in this state.
			   Commutation occurs at every Timer1 overflow event */
			if(TMR1IF)
			{
				TMR1IF = 0;
				/* This service commutates the motor and either waits for
				 * a blanking period or computes the next commutation.
				 * If bemf_flag indicates that zero cross detection is to be
				 * performed in the the upcomming period then blanking is
				 * performed. Blanking holds off the input to the comparator
				 * to allow the commutation switching transients to settle.
				 * A relatively clean BEMF signal will then be available for
				 * zero cross detection. In this version of the software
				 * blanking is performed dynamically. We wait a minimum
				 * blanking period to give the drivers a chance to settle then
				 * we read the comparator output until it is low thereby
				 * ensuring that the flyback currents have settled.
				 * Then, the comparator will be setup for zero cross detection.
				 */
				Commutate();
				if(bemf_flag ^ ReverseDirection)
				{
					/* Adjust Timer1 preset to account for minimum blanking time */
					temp_comm_time.word = TMR1_comm_time.word + BLANKING_COUNT;

					/* Wait a mimimum blanking time to allow drivers to settle */
					while(TMR1L < BLANKING_COUNT);

					/* Setup Timer1 for the next commutation event */
					TMR1ON = 0;
					TMR1H = temp_comm_time.bytes.high;
					TMR1L += temp_comm_time.bytes.low;
					if(CARRY)TMR1H++;
					TMR1ON = 1;

					/* Wait for flyback currents to settle before setting up
					comparator for interrupts. If Timer1 overflows while waiting
					then flyback voltage and zero cross were both missed,
					in which case we need to commutate and try again. */
					while(CxOUT)
					{
						if (TMR1IF)
						{
							break;
						}
					}
					if(!TMR1IF)
					{
						ctemp = CMxCON0; /* reading register clears unused flops */
						CxIF = 0;
						CxIE = 1;
						isr_state = zero_detect;
					}
				}
				else
				{
					/* Compute corrected commutation time from present
					   commutation time and zero-crossing event time.
					   Zero-crossing error is the difference between
					   when ZC should have happened and when it actually
					   happened. Negative error shortens the commutation time
					   and positive error lengthens the commutation time.

					   ZCE(n) = ZC(n)-(CT(n)/2)
					   CT(n+1) = CT(n) + ZCE(n)*Error_Gain
					   where:
					     CT is commutation time
					     ZC is zero-crossing event time
					     ZCE is zero-crossing error
					     (n) denotes present cycle
					     (n+1) denotes next cycle

					   Timer1 is a 16-bit timer that counts up. The time to
					   overflow is the negative of the count in Timer1.
					   Signed integers are considered negative when the most
					   significant bit is 1. However, all instances of Timer1
					   counts are negative, even when the most significant bit
					   is zero. Therefore, it is easiest to think of Timer1 as
					   a 17-bit counter where the most significant
					   (unimplemented) bit is permantly fixed to 1.
					   The variables TMR1_comm_time and expected_zc are
					   signed 16-bit integers. When TMR1_comm_time is shifted
					   right to divide-by-2 then the 17th bit extension is lost
					   if the 16th bit of Timer1 is zero. We compensate for
					   the lost extension by forcing the extension on the
					   TMR1_comm_time right shift.
					*/

					expected_zc = UDIV2( TMR1_comm_time.word ); /* CT(n)/2 */
					avg_zc = UDIV2( zc.word ) + UDIV2( prev_zc.word );

					zc_error = (int16_t)(avg_zc - expected_zc); /* ZCE(n) = ZC(n)-(CT(n)/2) */
					/* absolute value */
					if( zc_error < 0 )
						temp1 = -zc_error;
					else
						temp1 = zc_error;
					temp2 = (int16_t)(UNEG( expected_zc ));
					temp2 >>= 1;
					/* stop forced commutation if zero cross detected within
					middle half of commutation period */
					/* Note by Pong: we can adjust how narrow of the zero cross
					range. For example, ((-expected_zc.word)>>2) for within
					the middle quater. */
					if (temp1 < temp2)
					{
						if( BLDC_State == RAMPUP )
						{
							zc_count--;
							if( zc_count == 0 )
							{
								/* Turn on LED OK when entering COMMUTE */
								LED_OK = 1;
								BLDC_State = COMMUTE;
								CCP1AS = ECCP1AS_INIT; /* Activate current limit */
							}
						}

						/* Reset stall timer as we have detected BEMF */
						TMR0_stall_timer = TIMEBASE_STALL_COUNT;
					}
					else
						zc_count = EXPECT_ZC_COUNT;

					/* Note by Pong: If we want to change start-up procedure--
					for example, make different adjustment to TMR1_comm_time--
					just add an if-else structure here. For example:
					if( BLDC_State == RAMPUP )
					{
						Start-up procedure
					}
					else
					{
						commutation procedure
					}
					*/
					/* -CT(n+1) = -CT(n) - ZCE(n)*Error_Gain */
					TMR1_comm_time.word -= (uint16_t)(zc_error>>ERROR_SCALE);
					
					/* Keep current TMR1_comm_time in backlog */
					avg_backlog[avg_backlog_index] = UNEG(TMR1_comm_time.word);
					avg_backlog_index++;
					avg_backlog_index &= AVG_BACKLOG_MASK;
										
					/* Limit zc to TMR1_comm_time */
					if( zc.word < TMR1_comm_time.word )
						zc.word = TMR1_comm_time.word;
						
					temp_comm_time.word = TMR1_comm_time.word + FIXED_ADVANCE_COUNT;

					/* setup for commutation */
					TMR1ON = 0;
					TMR1H = temp_comm_time.bytes.high;
					TMR1L += temp_comm_time.bytes.low;
					if(CARRY)TMR1H++;
					TMR1ON = 1;

					isr_state = commutate;
					/* setup for commutation time after zero cross event
					   half the commutation time is adjusted for motor advance
					   timing expected_zc is negative time to commutation event
					   ADVANCE_COUNT is positive time to advance. Adding
					   positive number to negative time shortens the negative time
					*/
					comm_after_zc.word = expected_zc + ADVANCE_COUNT;
				}
			}
			break;

		default:
			/* We should not be here!! So we break the motor to restart it */
			BLDC_State = RAMPDOWN;
			break;
	}/* end switch */
}/* end ISR */
/*---------------------------------------------------------------------------*/
/*===========================================================================*/

/************************************************************************
*                                                                       *
*                          Helper Functions                             *
*                                                                       *
*************************************************************************/

/************************************************************************
*                                                                       *
*      Function:       AvgCommTime                                      *
*                                                                       *
*      Description:    Average commutation time.                        *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

uint16_t AvgCommTime(void)
{
	uint32_t comm_sum;
	uint8_t c;
	
	comm_sum = 0;
	for( c = 0; c < AVG_BACKLOG_SIZE; c++ )
		comm_sum += (uint32_t)( avg_backlog[c] );
	return( (uint16_t)( comm_sum >> AVG_BACKLOG_LEN ) );
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       CommLoop_Setup                                   *
*                                                                       *
*      Description:    Initializing BLDC for alignment process.         *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void CommLoop_Setup()
{
	uint8_t c;
	
	TMR0_excite_timer = TIMEBASE_EXCITE_COUNT;

	/* Clear previous speed backlog */
	for( c = 0 ; c < AVG_BACKLOG_SIZE ; c++ )
		avg_backlog[c] = 0;
	avg_backlog_index = 0;

	/* Setup Sub-states */
	excite_events = ALIGN_STEPS;

	/* Start the first move */
	/* startup duty cycle; It is already normalized to MAX_DUTY_CYCLE */
	current_pwm = ALIGN_PWM_START;
	SetCCPVal( ALIGN_PWM_START );
	CCP1CON = CCP1CON_INIT;           /* PWM on */
	
	/* HIN = U; LIN = V and W */
	PSTRCON = MODULATE_V | MODULATE_W;
	DRIVE_U = 1;
	DRIVE_V = 0;
	DRIVE_W = 0;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       CommLoop_Align                                   *
*                                                                       *
*      Description:    Align BLDC rotor to a predefined position.       *
*                                                                       *
*      Parameters:                                                      *
*      Return value:   0 - BLDC does not finish alignment yet.          *
*                      1 - BLDC finished alignment and ready for the    *
*                          next state.                                  *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

uint8_t CommLoop_Align(void) /* Excitation phase */
{
	/* The excitation timer determines how long to dwell at each slow
	 start commutation point. */
	--TMR0_excite_timer;
	if( TMR0_excite_timer == 0 )
	{
		/* when slow start is complete change to the startup timer
		   which determines how long to ramp-up at fixed commutations
		   the ramp-up is terminated early when the first zero cross
		   is detected. */
		--excite_events;
		if( excite_events == 0 )
		{
			zc_count = EXPECT_ZC_COUNT;

			/* Initial RPM */
			TMR1_comm_time.word = startup_rpm;

			/* Setup commutation timing */
			zc.word = startup_rpm;	/* zc cannot go lower than TMR1_comm_time */
			comm_after_zc.word = UDIV2( zc.word );

			TMR1H = TMR1_comm_time.bytes.high;
			TMR1L = TMR1_comm_time.bytes.low;
			isr_state = commutate;
			/* Setup commutation phase for start up */
			/* HIN = none */
			DRIVE_U = 0;
			DRIVE_V = 0;
			DRIVE_W = 0;
			if( ReverseDirection )
			{
				/* LIN = V */
				PSTRCON = MODULATE_V;
				comm_state = 6;
			}
			else
			{
				/* LIN = W */
				PSTRCON = MODULATE_W;
				comm_state = 3;
			}
			Commutate();	/* Make the first move */
			TMR1ON = 1;
			TMR1IE = 1;
			PEIE=1;
			GIE=1;
			return(1);
		}
		else
		{
			/* reset the dwell timer for the next step */
			TMR0_excite_timer = TIMEBASE_EXCITE_COUNT;
			current_pwm += ALIGN_PWM_STEP;
			SetCCPVal( current_pwm );
		}
	}
	return(0);
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       CommLoop_Start                                   *
*                                                                       *
*      Description:    Speed-up BLDC (ramp-up) until it reach the state *
*                      that closed-loop commutation could be performed. *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void CommLoop_Start() /* Ramp up */
{
	/* AN1305 does nothing in ramp up */
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       CommLoop_Stop                                    *
*                                                                       *
*      Description:    Disable all closed-loop commutation control.     *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void CommLoop_Stop()
{
	TMR1ON = 0;
	TMR1IE = 0;
}
/*---------------------------------------------------------------------------*/
