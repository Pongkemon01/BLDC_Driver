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

/* #pragma config statements should precede project file includes. */

/* CONFIG1 */
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable (All VCAP pin functionality is disabled)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

/******************* Project includes and definition *********************/
#include "BLDC.h"
#include "comm_loop_new_v2.h"

/******************************************************************************
* variable definitions. Use static-global to save time to create auto var.    *
* BY DEFAULT, ALL GLOBAL VARIABLES ARE INITIALIZED TO 0 BY THE START-UP CODE. *
******************************************************************************/

/* Variables for commutation state */
uint8_t comm_state;
bit bemf_flag;
bit ReverseDirection;
/*---------------------------------------------------------------------------*/

enum BLDC_State_t BLDC_State;

/* Timer for each state */
uint8_t TMR0_rampup_timer;
uint8_t TMR0_rampdown_timer;
uint8_t TMR0_break_timer;
uint8_t TMR0_cooldn_timer;
uint8_t TMR0_overcurrent_timer;
uint8_t TMR0_oc_monostable_timer;

/*---------------------------------------------------------------------------*/

/* Variable for SPI communication driver functions */
enum SPI_State_t{
	IDLE=0,	/* Waiting for incoming command */
	PARAM11, /* Waiting for incoming MSB of the parameter */
	PARAM12, /* Waiting for incoming LSB of the parameter */
	PARAM21, /* Waiting for host to get the MSB of return value */
	PARAM22  /* Waiting for host to get the LSB of return value */
}SPI_State;
doublebyte RetParam, InpParam;
uint8_t NewSPICommand;
uint8_t NewSPIUndocCommand;
int16_t desired_speed = 0;	/* Speed is in electrical rpm */
uint16_t desired_pwm = 0; /* PWM value in 10-bit format (used in PWM mode) */
/*---------------------------------------------------------------------------*/

/* Variable for main and miscellaneous functions */
uint8_t timebase_10ms;
uint8_t oc_restart_count;

uint16_t pwm_current;

enum BLDC_Mode_t{
	SPEED_MODE=0,	/* BLDC operates in speed-control mode */
	PWM_MODE		/* BLDC operates in pwm-control mode */
}BLDC_Mode;
/*---------------------------------------------------------------------------*/

void Commutate(void);

/* Include the PWM control engine */
#include "BLDC_Closed_loop.h"

/************************************************************************
*                                                                       *
*      Function:       Commutate                                        *
*                                                                       *
*      Description:    driver commutation                               *
*                                                                       *
*      PSTRCON is used to steer PWM to one side of bridge and           *
*      DRIVE_PORT is used to drive other side. Comparator input         *
*      is steered to undriven phase of motor.                           *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*************************************************************************/
/*
*    3|4|5|0|1|2|3|4|5|0|1|2|
*     | | |_|_| | | | |_|_| |
*  U _|_|/| | |\|_|_|/| | |\|
*    _| | | | |_|_| | | | |_|
*  V  |\|_|_|/| | |\|_|_|/| |
*     |_|_| | | | |_|_| | | |
*  W /| | |\|_|_|/| | |\|_|_|
*
* State  Low   High  Comparator
*   0     V     U       -W
*   1     W     U        V
*   2     W     V       -U
*   3     U     V        W
*   4     U     W       -V
*   5     V     W        U
*/

void Commutate(void)
{
	/* Low side modulation and BEMF is sensed when falling */
	switch(comm_state) {
	case 0:
		/* HIN = U; LIN = V; SENSE = W */
		PSTRCON = MODULATE_V;
		DRIVE_U = 1;
		DRIVE_V = 0;
		DRIVE_W = 0;
		COMPARATOR = SENSE_W_FALLING;
		bemf_flag = 1;
		break;
	case 1:
		/* HIN = U; LIN = W; SENSE = V */
		PSTRCON = MODULATE_W;
		DRIVE_U = 1;
		DRIVE_V = 0;
		DRIVE_W = 0;
		COMPARATOR = SENSE_V_RISING;
		bemf_flag = 0;
		break;
	case 2:
		/* HIN = V; LIN = W; SENSE = U */
		PSTRCON = MODULATE_W;
		DRIVE_U = 0;
		DRIVE_V = 1;
		DRIVE_W = 0;
		COMPARATOR = SENSE_U_FALLING;
		bemf_flag = 1;
		break;
	case 3:
		/* HIN = V; LIN = U; SENSE = W */
		PSTRCON = MODULATE_U;
		DRIVE_U = 0;
		DRIVE_V = 1;
		DRIVE_W = 0;
		COMPARATOR = SENSE_W_RISING;
		bemf_flag = 0;
		break;
	case 4:
		/* HIN = W; LIN = U; SENSE = V */
		PSTRCON = MODULATE_U;
		DRIVE_U = 0;
		DRIVE_V = 0;
		DRIVE_W = 1;
		COMPARATOR = SENSE_V_FALLING;
		bemf_flag = 1;
		break;
	case 5:
		/* HIN = W; LIN = V; SENSE = U */
		PSTRCON = MODULATE_V;
		DRIVE_U = 0;
		DRIVE_V = 0;
		DRIVE_W = 1;
		COMPARATOR = SENSE_U_RISING;
		bemf_flag = 0;
		break;
	default:
		/* Error!! we should not be here. So, reset all state. */
		PSTRCON = 0;	/* Modulate off */
		DRIVE_U = 0;	/* Fixed drive off */
		DRIVE_V = 0;
		DRIVE_W = 0;
		COMPARATOR = 0x00;
		bemf_flag = 0;
		comm_state = 0;
		break;
	}
	/* setup for next commutation event */
	if( ReverseDirection )
	{
		comm_state--;
		if( comm_state > 5)	/* Wrap around */
			comm_state = 5;
	}
	else
	{
		comm_state++;
		if( comm_state > 5)
			comm_state = 0;
	}
} /* end Commutate */
/*---------------------------------------------------------------------------*/
/*===========================================================================*/

/************************************************************************
 * Operation cycle of a BLDC motor consists of 8 sequencial states.
 * Immediately after start-up, the BLDC motor is in STOP state.
 * When a running is initiated, the motor follow the following state steps
 * until reaching COMMUTE state.
 * - SETUP  : Setting up parameters for running the motor
 * - EXCITE : Activate motor coils twice with different polarity.
 *            This step holds the rotor at a known position.
 * - RAMPUP : Speed-up the motor to the minimum speed that BEMF can be detected.
 *            Running motor in this state is open-looped. The BEMF should
 *            be detected within a specified time period. Otherwise,
 *            the motor is considered as stalled and must perform
 *            stop procedure.
 * - COMMUTE : BEMF can be detected. The motor runs with closed-loop
 *            speed control algorithm.
 *
 * When we want to stop the motor or reverse its direction, we must follow
 * the stop procedure, which transits the motor through the following states
 * until it reach STOP state.
 * - RAMPDOWN : Turn-off all motor drivers and let the motor runs freely
 *            to reduce its speed.
 * - BREAK    : Turn-on all high-side MOSFETs while turn-off all low-side ones.
 *              This action results in BEMF break to completely stop the motor.
 * - COOLDN   : Turn-of all motor drivers again and wait for a while
 *              to deplete all residual currents.
 * - STOP     : The motor is ready for the next run.
 *
 ************************************************************************/

/************************************************************************
*                                                                       *
*      Function:       SetCCPVal (utility function)                     *
*                                                                       *
*      Description:    set 10-bit PWM duty cycle in CCPR register by    *
*                      the 16-bit parameter value                       *
*                                                                       *
*      Parameters:     16-bit PWM duty cycle                            *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void SetCCPVal( uint16_t Duty )
{
	 CCPR1L = (Duty >> 2);
	 DC1B0  = Duty & 0x01;
	 DC1B1  = (Duty >> 1) & 0x01;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       GetCCPVal  (utility function)                    *
*                                                                       *
*      Description:    get 10-bit PWM duty cycle from CCPR register     *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

uint16_t GetCCPVal( void )
{
	uint16_t ccpVal;

	ccpVal = ( (uint16_t)CCPR1L ) << 2;
	if( DC1B1 == 1 )
		ccpVal |= 2;
	if( DC1B0 == 1 )
		ccpVal |= 1;

	return( ccpVal );
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Setup                                       *
*                                                                       *
*      Description:     initialize PWM and driver registers & variables *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  Drivers are initialized when BLDC is going to start or resume.       *
*                                                                       *
*************************************************************************/

void BLDC_Setup( void )
{
	if( BLDC_State != SETUP ) return;

	/* Setup TimeBase parameters */
	TMR0_rampup_timer = TIMEBASE_RAMPUP_COUNT;
	TMR0_rampdown_timer = TIMEBASE_RAMPDOWN_COUNT;
	TMR0_break_timer = TIMEBASE_BREAK_COUNT;
	TMR0_cooldn_timer = TIMEBASE_COOLDN_COUNT;
	
	/* Setup commutation scheme */
	CommLoop_Setup();

	BLDC_State = EXCITE;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Excite                                 *
*                                                                       *
*      Description:  Dwell at two successive commutations before        *
*                   starting the spin up                                *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  This is called every main loop cycle but the TMR0_slow_start_flag    *
*  limits execution to every 10 ms as defined by the TimeBaseManager()  *
*  routine.                                                             *
*                                                                       *
*************************************************************************/

void BLDC_Excite( void )
{
	if( BLDC_State != EXCITE ) return;

	if( ( CommLoop_Align() ) == 1)
		BLDC_State = RAMPUP;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Rampup                                      *
*                                                                       *
*      Description:    Wait and check whether BEMF can be detected.     *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*  Zero cross must be detected within the middle 25% of the             *
*  commutation period before the startup timer expires otherwise        *
*  the motor will be shut down.                                         *
*                                                                       *
*  Startup time is determined by the constant TIMEBASE_RAMPUP_COUNT     *
*  which is defined in BLDC.h. Maximum startup time is 2.55 seconds.    *
*  This routine is called every 10ms during RAMPUP state.               *
*                                                                       *
*************************************************************************/

void BLDC_Rampup( void )
{
	if( BLDC_State != RAMPUP ) return;

	CommLoop_Start();	/* Ramp-up procedure */
	--TMR0_rampup_timer;
	if( TMR0_rampup_timer == 0 )
	{
		/* Commutation interrupt will change BLDC state to COMMUTE,
		if BEMF is detected. If rotation is not stable by the time
		the startup is complete then force a full reinitialization
		by jumping to RAMPDOWN state. */
		if( BLDC_State != COMMUTE )
		{
			LED_ERROR = 1;	/* Stall */
			BLDC_State = RAMPDOWN;
		}
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Commute                                     *
*                                                                       *
*      Description:     check motor stall and re-start                  *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void BLDC_Commute( void )
{
	if( BLDC_State != COMMUTE ) return;

	/* Our algorithm detects motor stall within interrupt routine.
	Therefore, we do not perform any operation here */
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Rampdown                                    *
*                                                                       *
*      Description:    Let the BLDC spins freely before break.          *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void BLDC_Rampdown( void )
{
	if( BLDC_State != RAMPDOWN ) return;

	LED_OVERCURRENT = 0;	/* Motor off, no more over-current */

	/* Turn off all PWM and High-side drivers. */
	PSTRCON = MODULATE_OFF;
	CCP1CON = 0;	/* PWM off */
	DRIVE_U = 0;
	DRIVE_V = 0;
	DRIVE_W = 0;

	/* Diable all control */
	CommLoop_Stop();
	PEIE = 0;
	GIE = 0;

	/* Turn-off green LED */
	LED_OK = 0;

	/* Update timer and check for time-out */
	--TMR0_rampdown_timer;
	if( TMR0_rampdown_timer == 0 )
	{
		BLDC_State = BREAK;
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Break                                       *
*                                                                       *
*      Description:    Break the BLDC by activating all High-side.      *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void BLDC_Break( void )
{
	if( BLDC_State != BREAK ) return;

	/* Turn off all PWM. Turn on High-side drivers. */
	PSTRCON = MODULATE_OFF;
	CCP1CON = 0;	/* PWM off */
	DRIVE_U = 1;
	DRIVE_V = 1;
	DRIVE_W = 1;

	/* Update timer and check for time-out */
	--TMR0_break_timer;
	if( TMR0_break_timer == 0 )
	{
		BLDC_State = COOLDN;
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       BLDC_Cooldown                                    *
*                                                                       *
*      Description:    Switch-off all MOSFET and wait for a while.      *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void BLDC_Cooldown( void )
{
	if( BLDC_State != COOLDN ) return;

	/* Turn off all PWM and High-side driver. */
	PSTRCON = MODULATE_OFF;
	CCP1CON = 0;	/* PWM off */
	DRIVE_U = 0;
	DRIVE_V = 0;
	DRIVE_W = 0;

	/* Update timer and check for time-out */
	--TMR0_cooldn_timer;
	if( TMR0_cooldn_timer == 0 )
	{
		BLDC_State = STOP;
	}
}
/*---------------------------------------------------------------------------*/

/***********************************************************************
* Main function of BLDC state. The state machine updates and transits
* every 10ms. (controlled by Timer0 from the main loop).
*************************************************************************/

void BLDC_Machine( void )
{
	switch( BLDC_State )
	{
		case STOP:
			/* Just clear over-current status */
			LED_OVERCURRENT = 0;
			break;
		case SETUP:
			BLDC_Setup();
			break;
		case EXCITE:
			BLDC_Excite();
			break;
		case RAMPUP:
			BLDC_Rampup();
			break;
		case COMMUTE:
			BLDC_Commute();
			break;
		case RAMPDOWN:
			BLDC_Rampdown();
			break;
		case BREAK:
			BLDC_Break();
			break;
		case COOLDN:
			BLDC_Cooldown();
			break;
		default :
			/* Error!!! we should not be here */
			BLDC_State = RAMPDOWN; /* Reset state to RAMPDOWN for safely stop*/
			break;
	}
}
/*---------------------------------------------------------------------------*/
/*===========================================================================*/

/************************************************************************
 * SPI driver.
 * It manage SPI state
 ************************************************************************/

/************************************************************************
*                                                                       *
*      Function: 	     ReadCurrentRPM (Utility function)              *
*                                                                       *
*      Description:      Convert average commutation time from          *
*                        TMR1_comm_time history log back to             *
*                        electrical RPM.                                *
*                                                                       *
*      Parameters:                                                      *
*      Return value:      RPM value in uint16_t                         *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

uint16_t ReadCurrentRPM( void )
{
	return( (uint16_t)( ( (uint32_t)COMM_TIME_TO_RPM_FACTOR ) / ( avg_comm_time() ) ) );
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     PackStatus (Utility function)                  *
*                                                                       *
*      Description:      Pack the driver status indicated by LEDs into  *
*                        the format that required by SPI driver.        *
*                        ( 0ect0 0000 )                                 *
*                                                                       *
*      Parameters:                                                      *
*      Return value:      Status value in uint8_t                       *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

uint8_t PackStatus( void )
{
	uint8_t Status;

	Status = 0;
	if( LED_OK == 1 )
		Status |= LED_OK_MASK;
	if( LED_ERROR == 1 )
		Status |= LED_ERROR_MASK;
	if( LED_OVERCURRENT == 1 )
		Status |= LED_OVERCURRENT_MASK;
	if( LED_OVERTEMP == 1 )
		Status |= LED_OVERTEMP_MASK;

	return( Status );
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     ProcessSPIParam                                *
*                                                                       *
*      Description:      Interpret and process the command from SPI.    *
*                        The commands in consideration are those that   *
*                        has extra parameter value apart from the       *
*                        command byte such as setting BLDC speed.       *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

/***************************************************

SPI Commands. They have the maximum size of 3 bytes

Set max current: 111c cccc
   where c is 5-bit data use to set DAC output voltage. The DAC produces
   voltage in the range 0V to 1.984V, which is used to compare with the output
   from curent sensing circuit. However, this firmware cap the setting to the
   range 2 - 28 (1.2A - 17A)

   return: kect0 0000 0ect0 0000 0ect0 0000
   where k = commutation flag
		 e = Error flag
		 c = Over-current flag
		 t = Over-temperature flag

Set electrical RPM: 1100 0000 pppp pppp pppp pppp
   where p is 16-bit 2'complement RPM value (-32768 - 32767)
   return: kect0 0000 kect0 0000 kect0 0000

Get current electrical RPM: 1010 0000 0000 0000 0000 0000
	return kect0 0000 rrrr rrrr rrrr rrrr
	where r is sign-extended of p (see above)

No operation (used to read status): 1000 0000
	return kect0 0000

Hidden commands (undocumented):

Set operation mode to speed mode: 1000 0100
	return kect 0000

Set operation mode to PWM mode: 1000 1000
	return kect 0000

Set PWM value (used only in PWM mode): 1000 1100 dddd dddd
	where d is percent of PWM duty cycle (0 - 100)
	return kect 0000 kect 0000

Get PWM value : 1001 0000 0000 0000 0000 0000
	return kect0 0000 0000 00dd dddd dddd
	
Reverse BLDC rotation direction : 1001 0100
	return kect 0000
	(Operates only in PWM mode)

****************************************************/

void ProcessSPIParam( void )
{
	/* Currently, only SetRPM and SetPWM commands need action */
	switch( NewSPICommand )
	{
		case SPI_SET_SPEED:
			LED_OVERCURRENT = 0;
			desired_speed = ( int16_t )( InpParam.word );
			/* Cap with the boundary */
			if(desired_speed > 0)
			{
				if( desired_speed > MAX_RPM )
					desired_speed = MAX_RPM;
				else if( desired_speed < MIN_RPM )
					desired_speed = MIN_RPM;
			}
			else if( desired_speed != 0 )
			{
				if( desired_speed < -(MAX_RPM) )
					desired_speed = -(MAX_RPM);
				else if( desired_speed > -(MIN_RPM) )
					desired_speed = -(MIN_RPM);
			}
			oc_restart_count = MAX_OVERCURRENT_RST;
			break;
			
		case SPI_NOP:
			LED_OVERCURRENT = 0;
			if( NewSPIUndocCommand == SPI_UNDOC_SET_PWM )
			{
				/* Parameter should be in percent (0 - 100) */
				if( InpParam.word > 100 )
					InpParam.word = 100;	/* Cap the maximum value */
				desired_pwm = (uint16_t)( ( ( (uint32_t)InpParam.word ) * MAX_DUTY_CYCLE ) / 100L );
			}
			break;

		default:
			/* Do nothing */
			break;
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     SPIManager                                     *
*                                                                       *
*      Description:      Manage SPI state and return requested data to  *
*                        the host.                                      *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void SPIManager( void )
{
/* SPI Status flags
Data ready to read: SSPIF (in PIR)
Buffer overflow: SSPOV (in SSPCON1)

Each SSPIF event means data to server was transmitted
and new data from server is ready to be read in SSPBUF (must read it)
then new data to server should be place in the SSPBUF for next SSPIF.
*/

	volatile uint8_t dummy; /* Prevent optimization to remove this variable */
	
	if( SSPOV == 1 )
	{
		/* SPI Overflow!!! The state must be reset to IDLE */
		SPI_State = IDLE;
		dummy = SSPBUF;
		SSPOV = 0;
		SSPIF = 0;
		
		return;
	}

	/* Retrieve commands from 8-bit SPI module */
	if( SSPIF == 0)
	{
		if( ( SPI_State == IDLE ) && ( SS_PIN == 1 ) ) /* No incoming data */
			SSPBUF = PackStatus();

		return;
	}

	SSPIF = 0;	/* Clear status */
	switch( SPI_State )
	{
		case IDLE:	/* A byte is received in IDLE. It must be a command code */
			NewSPICommand = SSPBUF;
			/* Extract embedded parameter */
			InpParam.bytes.high = NewSPICommand & SPI_PARAM_MASK;
			NewSPICommand &= SPI_COMMAND_MASK;
			switch( NewSPICommand )
			{
				case SPI_NOP:
					/* Check for extra control commands (undocumented) */
					if( InpParam.bytes.high != 0 )
					{
						NewSPIUndocCommand = InpParam.bytes.high & SPI_UNDOC_MASK;
						switch( NewSPIUndocCommand )
						{
							case SPI_UNDOC_SPEED_M:
								BLDC_Mode = SPEED_MODE;
								if( BLDC_State == COMMUTE )
									BLDC_State = RAMPDOWN;
								SSPBUF = PackStatus();
								break;
							case SPI_UNDOC_PWM_M:
								BLDC_Mode = PWM_MODE;
                                desired_pwm = pwm_current;
								//if( BLDC_State == COMMUTE )
								//	BLDC_State = RAMPDOWN;
								SSPBUF = PackStatus();
								break;
							case SPI_UNDOC_GET_PWM:
								RetParam.word = GetCCPVal();
								SSPBUF = RetParam.bytes.high;
								SPI_State = PARAM21;
								break;
							case SPI_UNDOC_SET_PWM:
								SSPBUF = PackStatus();
								InpParam.bytes.high &= SPI_UNDOC_PARAM_MASK;
								SPI_State = PARAM12;
								break;
							case SPI_UNDOC_REVERSE:
								if( BLDC_Mode == PWM_MODE )
								{
									ReverseDirection = ~ReverseDirection;
									if( BLDC_State == SETUP ||
										BLDC_State == EXCITE ||
										BLDC_State == RAMPUP ||
										BLDC_State == COMMUTE )
									{
										BLDC_State = RAMPDOWN;
									}
								}
								break;
							default:
								SSPBUF = PackStatus();
								break;
						}									
					}
					else
						SSPBUF = PackStatus();
					break;
				case SPI_SET_SPEED:
					SSPBUF = PackStatus();
					SPI_State = PARAM11;
					break;
				case SPI_GET_SPEED:
					RetParam.word = (uint16_t)(ReadCurrentRPM());
					if( ReverseDirection == 1 ) /* Reverse direction? then negative rpm */
						RetParam.word = UNEG( RetParam.word );
					SSPBUF = RetParam.bytes.high;
					SPI_State = PARAM21;
					break;
				case SPI_SET_CURRENT:
					SSPBUF = PackStatus();
					if( InpParam.bytes.high < 2 )
						InpParam.bytes.high = 2;
					if( InpParam.bytes.high > 28 )
						InpParam.bytes.high = 28;
					DACCON1 = InpParam.bytes.high;	/* Set current value */
					break;
				default:
					/* Unknown. Do nothing */
					break;
			}
			break;
		case PARAM11:
			/* Read the MSB of incoming parameter */
			InpParam.bytes.high = SSPBUF;
			SSPBUF = PackStatus();
			SPI_State = PARAM12;
			break;
		case PARAM12:
			/* Read the LSB of incoming parameter and process the command */
			InpParam.bytes.low = SSPBUF;
			SSPBUF = PackStatus();
			SPI_State = IDLE;
			ProcessSPIParam();
			break;
		case PARAM21:
			/* Clear out dummy data from host and prepare LSB of outgoing */
			dummy = SSPBUF;
			SSPBUF = RetParam.bytes.low;
			SPI_State = PARAM22;
			break;
		case PARAM22:
			/* Clear out dummy data from host */
			dummy = SSPBUF;
			SSPBUF = PackStatus();
			SPI_State = IDLE;
			break;
		default:
			/* Clear out dummy data from host */
			dummy = SSPBUF;
			SSPBUF = PackStatus();
			SPI_State = IDLE;	/* Error!!! Reset the state */
			break;
	}
}
/*---------------------------------------------------------------------------*/
/*===========================================================================*/

/************************************************************************
 * Miscellaneous and main functions.
 ************************************************************************/

/************************************************************************
*                                                                       *
*      Function:       TimeBaseManager                                  *
*                                                                       *
*      Description:    manage time base timer TIMER0                    *
*                                                                       *
*      Parameters:                                                      *
*      Return value:   1 every 10ms. otherwise 0                        *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  The TimeBaseManager() is called in the main loop.                    *
*  The function use polling based to check Timer0.                      *
*  The TimeBaseManager() base time resolution is 10mS set by the        *
*  constant TIMEBASE_LOAD_10ms. All other timers inherit this           *
*  resolution.                                                          *
*                                                                       *
*************************************************************************/

uint8_t TimeBaseManager( void )
{
	if(T0IF)
	{
		T0IF = 0;
		TMR0 += TIMEBASE_MANAGER_RELOAD_COUNT;

		if(timebase_10ms != 0)
		{
			timebase_10ms--;
			return 0;								// wait 10ms
		}

		timebase_10ms = TIMEBASE_LOAD_10ms;
		return 1;
	}

	return 0;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function:       InitSystem                                       *
*                                                                       *
*      Description:    initialize system registers and variables        *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void InitSystem( void )
{
	/* disable interrupts */
	PEIE=0;
	GIE=0;

	/* Setup internal oscillator frequency */
	OSCCON = OSCCON_INIT;

	/* set all ports tristate */
	TRISA = 0xFF;
	TRISB = 0xFF;
	TRISC = 0xFF;

	/* initialize ports */
	PORTA=PORTA_INIT;
	TRISA=TRISA_INIT;

	PORTB=PORTB_INIT;
	TRISB=TRISB_INIT;

	PORTC=PORTC_INIT;
	TRISC=TRISC_INIT;

	/* initialize Analog pins */
	ANSELA=ANSELA_INIT;
	ANSELB=ANSELB_INIT;

	/* FVR, DAC, and Temperature sensors */
	FVRCON = FVRCON_INIT;
	DACCON0 = DACCON0_INIT;
	DACCON1 = DAC_STARTING_CURRENT;

	/* ADC */
	ADCON0 = ADCON0_INIT;
	ADCON1 = ADCON1_INIT;

	/* Disable PWM */
	CCP1CON = 0;

	/* PWM auto-shutdown and auto-restart */
	PWM1CON = PWM1CON_INIT;
	CCP1AS = ECCP1AS_INIT; /* Activate current limit */

	/* COMPARATORS */
	CMxCON0 = CMxCON0_INIT;
	CMxCON1 = CMxCON1_INIT;

	CMyCON0 = CMyCON0_INIT;	/* Prepare for current sense */
	CMyCON1 = CMyCON1_INIT;

	/* DRIVER init */
	PR2 = PWM_PERIOD;
	T2CON = T2CON_INIT;

	/* Initial BLDC state and drive port bits */
	BLDC_State = STOP;
	DRIVE_U = 0;
	DRIVE_V = 0;
	DRIVE_W = 0;
	MODULATE_U_BIT = 0;
	MODULATE_V_BIT = 0;
	MODULATE_W_BIT = 0;

	/* TIMER1 */
	T1CON = T1CON_INIT;
	
	/* SPI */
	SSPCON1 = SSPCON1_INIT;
	SSPIF = 0;

	/* TIMER0 and related startup variables */
	TMR0 = 0;
	T0IF = 0;
	OPTION_REG = OPTION_REG_INIT;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     PwmManager                                   *
*                                                                       *
*      Description:    Perform a control loop to adjust PWM according   *
*                      to desired BLDC speed                            *
*                      The PI algorithm used in this function is        *
*                      increment mode PI algorithm using the equation:  *
*                                                                       *
*                      Delta u = (Kp * (E(k) - E(k-1))) + (Ki * E(k))   *
*                                                                       *
*                      Delta u will be added to current PWM value to    *
*                      to adjust PWM period. The equation yields delta  *
*                      instead of the actual PWM value like conventional*
*                      PI algorithm.                                    *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
inline void PwmManager(void)
{
	//if( BLDC_Mode != PWM_MODE ) return; /* Operate only in PWM mode */

	if( BLDC_State == COMMUTE )
	{

		if( desired_pwm < REQUEST_OFF )
		{
			desired_pwm = 0;
			BLDC_State = RAMPDOWN;
			return;
		}

		if( desired_pwm > MAX_PRAC_DUTY_CYCLE )
			desired_pwm = MAX_PRAC_DUTY_CYCLE;
			
		if( pwm_current != desired_pwm )
		{
			if( desired_pwm > pwm_current )
			{
				if( ( desired_pwm - pwm_current ) > PWM_ROC )
					pwm_current += PWM_ROC;
				else
					pwm_current = desired_pwm;
			}
			else
			{
				if( ( pwm_current - desired_pwm ) > PWM_ROC )
					pwm_current -= PWM_ROC;
				else
					pwm_current = desired_pwm;
			}
			SetCCPVal( pwm_current );
		}
	}
	else
	{
		/* BLDC is not in COMMUTE state, do not start PWM control */
		pwm_current = STARTUP_DUTY_CYCLE;

		if( ( desired_pwm >= STARTUP_DUTY_CYCLE ) && ( BLDC_State == STOP ) )
			BLDC_State = SETUP;	/* Start motor if a desired speed is set */
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     SpeedManager                                   *
*                                                                       *
*      Description:    Perform a control loop to adjust PWM according   *
*                      to desired BLDC speed                            *
*                      The PI algorithm used in this function is        *
*                      increment mode PI algorithm using the equation:  *
*                                                                       *
*                      Delta u = (Kp * (E(k) - E(k-1))) + (Ki * E(k))   *
*                                                                       *
*                      Delta u will be added to current PWM value to    *
*                      to adjust PWM period. The equation yields delta  *
*                      instead of the actual PWM value like conventional*
*                      PI algorithm.                                    *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
inline void SpeedManager(void)
{
	int16_t temp_pwm, avg_speed;
	
	if( BLDC_Mode != SPEED_MODE ) return; /* Operate only in speed mode */

	if( BLDC_State == COMMUTE )
	{
		/* If request to stop the motor or change rotation direction,
		then stop the motor first. */
		if( ( desired_speed == 0 )
		 || ( ( desired_speed < 0 ) && ( ReverseDirection == 0 ) )
		 || ( ( desired_speed > 0 ) && ( ReverseDirection == 1 ) ) )
		{
			BLDC_State = RAMPDOWN;
			return;
		}
		
		avg_speed = (int16_t)( ReadCurrentRPM() );
		/* Calculate PWM change using a control engine */
		/* If temp_pwm is a negative number, then an error occurs */
		if( desired_speed > 0)
			temp_pwm = PWMControlEngine( ( int16_t )pwm_current, desired_speed, avg_speed );
		else
			temp_pwm = PWMControlEngine( ( int16_t )pwm_current, ( -desired_speed ), avg_speed );

		/* stop when speed requested is below the minimum speed threshold */
		if( temp_pwm < REQUEST_OFF )
			temp_pwm = 0;
		desired_pwm = (uint16_t)temp_pwm;
	}
	else
	{
		/* BLDC is not in COMMUTE state, do not start PWM closed-loop control*/
		PWMControlEngineInit();

		if( ( desired_speed >= MIN_RPM ) && ( BLDC_State == STOP ) )
		{
			desired_pwm = STARTUP_DUTY_CYCLE;
			if( desired_speed < 0 )
				ReverseDirection = 1;
			else
				ReverseDirection = 0;
		}
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     CheckOverCurrent                               *
*                                                                       *
*      Description:    Check for over-current status, automatically     *
*                      performed by PWM auto-shutdown mechanism.        *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
inline void CheckOverCurrent( void )
{
	/* Check for over-current. Over-current status will be cleared by
	monostable operation in Main */
	if( ( BLDC_State == COMMUTE ) && ( OC_STAT == 1 ) )
	{
		TMR0_oc_monostable_timer = TIMEBASE_OC_DELAY_COUNT;
		LED_OVERCURRENT = 1;
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     CheckOverTemp                                  *
*                                                                       *
*      Description:    Check for over-temperatur status using on-chip   *
*                      PIC temperature sensor.                          *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
inline void CheckOverTemp( void )
{
	doublebyte Temperature;

	/* Check for over-temperature */
	Temperature.bytes.high = ADRESH;
	Temperature.bytes.low = ADRESL;

	GO = 1; /* Start next temperature reading */

	if( Temperature.word >= OC_ADCOUT )
		LED_OVERTEMP = 1;
	else
		LED_OVERTEMP = 0;
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*                              M A I N                                  *
*                                                                       *
*************************************************************************/
void main( void )
{
	uint8_t	control_timer;
    uint16_t i = 0;
    
	timebase_10ms = TIMEBASE_LOAD_10ms;
	control_timer = TIMEBASE_CONTROL_ITER_COUNT;
	ReverseDirection = 0;
	desired_speed = 0;
	desired_pwm = 0;
	SPI_State = IDLE;
	BLDC_State = STOP;
	BLDC_Mode = SPEED_MODE;
	oc_restart_count = MAX_OVERCURRENT_RST;
	TMR0_oc_monostable_timer = 0;
	TMR0_overcurrent_timer = TIMEBASE_OVERCURRENT_COUNT;

	InitSystem();

	LED_OK = 0;
	GO = 1;	/* Start first temperature read-out */

	/* Code section for debug */
	/*SetCCPVal( STARTUP_DUTY_CYCLE );
	CCP1CON = CCP1CON_INIT;
	
	DRIVE_U = 0;
	DRIVE_V = 0;
	DRIVE_W = 0;
	PSTRCON = 0;*/

	/* To debug, uncomment only 1 line from the following lines. 
	ONLY 1 LINE could be uncomment at a time */	
	
	//PSTRCON = MODULATE_U;
	//PSTRCON = MODULATE_V;
	//PSTRCON = MODULATE_W;
	//DRIVE_U = 1;
	//DRIVE_V = 1;
	//DRIVE_W = 1;

    BLDC_Mode = PWM_MODE;
    /* End of debugging code */

	/* Main loop */
	while(1) /* Infinite Loop */
	{
		CLRWDT();
		CheckOverCurrent();
		SPIManager();

	    if( TimeBaseManager() == 1 ) /* Return 1 every 10ms */
		{
			/* This block is executed every 10ms */
			BLDC_Machine();
			CheckOverTemp();
			
			/* Over-current protection */
			if( TMR0_oc_monostable_timer != 0 )
				TMR0_oc_monostable_timer--;

			/* After decrement, if monostable is out, 
			clear over-current status */
			if( TMR0_oc_monostable_timer == 0 )
			{
				TMR0_overcurrent_timer = TIMEBASE_OVERCURRENT_COUNT;
				LED_OVERCURRENT = 0;
			}
			else
			{
				/* Otherwise, count the timeout */
	            if( TMR0_overcurrent_timer != 0 )
		            TMR0_overcurrent_timer--;
				
				/* If time is up, shutdown/restart BLDC. The counter for
				restarting the BLDC will be reset when a new speed-set
				command is received from SPI */
				if( TMR0_overcurrent_timer == 0 && BLDC_State == COMMUTE )
	        	{
					TMR0_overcurrent_timer = TIMEBASE_OVERCURRENT_COUNT;
		        	if( oc_restart_count != 0 )
		        		oc_restart_count--;
		        	else	
		    	    	desired_speed = 0;
	        		BLDC_State = RAMPDOWN;
	        	}
			}

			control_timer--;
			if( control_timer == 0 )
			{
				/* High-level control algorithm should be performed in longer
				interval than 10ms. Otherwise, the motor is unstable in
				high speed */
				control_timer = TIMEBASE_CONTROL_ITER_COUNT;
				if( BLDC_Mode == SPEED_MODE )
				{
					/* Perform speed controller to adjust PWM value */
					SpeedManager();
				}
			}

			/* Perform low-level control */			
			PwmManager();

			/* DEBUG */
			if( i < 65530 )
				i++;
            if(i == 400)
            {
				//ReverseDirection = 1;
				//BLDC_State = SETUP;
				//desired_speed = 4000;
				desired_pwm = STARTUP_DUTY_CYCLE; /* Max = 635, startup = 127 */
                //desired_pwm = 330;  // Max for V2 (6.7A)
                //desired_pwm = 250;
			}
			if( i == 800 )
			{
                desired_pwm = 290;
				//desired_speed = 6000;
			}
			if( i == 1200 )
			{
				desired_pwm = 150;
				//desired_speed = 3500;
			}
			/*if(i == 1000)
			{
				desired_speed = 3000;
			}

			if(i == 1500)
			{
				desired_speed = 6000;
			}*/
            /* End of DEBUG */

		}
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                         E N D   M A I N                               *
*************************************************************************/
