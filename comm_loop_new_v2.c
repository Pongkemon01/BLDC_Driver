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
#include "comm_loop_new_v2.h"

/* Variables for ISR */
enum isr_state_t{zero_detect,commutate} isr_state;
uint8_t zc_count;
uint16_t avg_zc;

/* Zero-crossing timing state */
doublebyte zc;
doublebyte comm_after_zc;
uint16_t TMR1_comm_time;

/* Timer counters for startup */
uint8_t TMR0_excite_timer;

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
				
				/* zc time was scaled by 0x8000 (the 15th bit is 1) so we
				scale it back */
				zc.word &= 0x7FFFU;
				
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
					
					/* We finished timely critical task so we do something to
					prepare for next commutation state. This should reduce
					timely critical requirement for the next step. */
					
					/* Zero crossing occurred too early. We cannot control
					the motor any more */
					if( zc.word < MIN_ZC_TIME )
					{
						LED_OK = 0;
						LED_ERROR = 1;
						BLDC_State = RAMPDOWN;
					}
					/* Exponential average */
					avg_zc = ( avg_zc >> 1 ) + ( ( zc.word ) >> 1 );
					
					/* Zero crossing should occur in the middle of commutation
					period. Therefore, commutation time is twice of zero-crossing
					time. */
					TMR1_comm_time = ( avg_zc ) << 1; /* avg_zc is positive time */
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
						LED_ERROR = 0;
						DACCON1 = DAC_RUNNING_CURRENT;
						BLDC_State = COMMUTE;
					}
				}

				isr_state = commutate;
			}
			else
			{
				/* Interrupt is from timer1
				If execution reaches this point then a Timer1 interrupt
				occurred before zero-crossing was detected. This should only
				happen during acceleration in startup. However, it should not
				occur in normal commutation. */
				
				if(BLDC_State == COMMUTE)
				{
					/* During COMMUTE state, we should not be here
					because we set the TIMER to the maximum allowable interval.
					If we ever reach here, we have got troubles */
					LED_OK = 0;
					LED_ERROR = 1;
					BLDC_State = RAMPDOWN;
				}
			}
			break;

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
					/* Zero crossing should occurred within a half range of 
					Timer1 counting. Otherwise, we cannot manage commutation
					time, which is twice of the zero-crossing time. Therefore, 
					we start searching for zero-crossing event by setting
					Timer1 to start at 0x8000 (half way of full-range counting) */
					
					/* Adjust Timer1 preset to account for minimum blanking time */
					temp_comm_time.word = 0x8000U + BLANKING_COUNT;

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
					/* Compute corrected commutation time from zero-crossing
					   event time. Zero-crossing time was recorded when Timer1
					   had started counting up from 0. Therefore, the recorded
					   Timer1 value can be considered as a positive time value,
					   which is opposite from our Timer1 assumption below.

					   Timer1 is a 16-bit timer that counts up. The time to
					   overflow is the negative of the count in Timer1.
					   Signed integers are considered negative when the most
					   significant bit is 1. However, all instances of Timer1
					   counts are negative, even when the most significant bit
					   is zero. Therefore, it is easiest to think of Timer1 as
					   a 17-bit counter where the most significant
					   (unimplemented) bit is permanently fixed to 1.
					*/

					/* TMR1_comm_time was prepared in zero-detect state. */
					
					/* Negative the time to conform with Timer1 */
					temp_comm_time.word = UNEG( TMR1_comm_time ) + FIXED_ADVANCE_COUNT;

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
					comm_after_zc.word = UNEG( avg_zc ) + ADVANCE_COUNT;
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

inline uint16_t avg_comm_time(void)
{
	return( TMR1_comm_time );
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

	/* Clear previous commutation time */
	TMR1_comm_time = 0;
	
	/* Set start-up current limit */
	DACCON1 = DAC_STARTING_CURRENT;

	/* Start the first move */
	/* startup duty cycle; It is already normalized to MAX_DUTY_CYCLE */
	SetCCPVal( STARTUP_DUTY_CYCLE );
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
doublebyte comm_time;

	/* The excitation timer determines how long to dwell at each slow
	 start commutation point. */
	--TMR0_excite_timer;
	if( TMR0_excite_timer == 0 )
	{
		zc_count = EXPECT_ZC_COUNT;

		/* Initial RPM */
		comm_time.word = UNEG( COMM_TIME_INIT );

		/* Setup commutation timing */
		zc.word = UDIV2( comm_time.word );	/* zc cannot go lower than TMR1_comm_time */
		comm_after_zc.word = zc.word;
		avg_zc = 0;

		TMR1H = comm_time.bytes.high;
		TMR1L = comm_time.bytes.low;
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
			comm_state = 5;
		}
		else
		{
			/* LIN = W */
			PSTRCON = MODULATE_W;
			comm_state = 2;
		}
		Commutate();	/* Make the first move */
		TMR1ON = 1;
		TMR1IE = 1;
		PEIE=1;
		GIE=1;
		return(1);
	}
	else
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
