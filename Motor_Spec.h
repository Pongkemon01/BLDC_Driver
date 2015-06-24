// File: Motor_Spec.h
//
// Motor parameters specific for VideoRay Thruster.
//
///////////////////////////////////////////////////////////////////////////////
// Parameters that should be adjusted to match the motor being used:
//
// START_RPM - This determines the open-loop commutation time at startup. A good
//             starting point is 1/10th the expected motor speed at the startup voltage.
//             (THIS VALUE IS IN ELECTRICAL RPM NOT THE MECHANICAL COUNTERPART).
//
// STARTUP_DRIVE_PCT = determines the startup duty cycle in percent which determines 
//             the startup voltage. 
//             Voltage = AppliedVoltage * Duty_cycle_count/MAX_DUTY_CYCLE.
//             Example: Applied voltage = 80V, STARTUP_DUTY_CYCLE = 0x10, therefore:
//             Startup voltage = 80 * 16/125 = 10.24 volts.
//
// MAX_PRAC_DUTY_PCT = determine the maximum percent of allowable pwm duty-cycle.
// As the charge-pump circuits for high-side MOSFETs require a certain amount 
// of PWM off time to recharge capacitors. The maximum PWM duty-cycle should not
// be greater than this value.
//
//
// BLANKING_COUNT_us - The number of microseconds to hold off from detecting zero cross
//            immediately after a commutation event. This allows the flyback currents
//            to die out so the back EMF can be accurately measured. Flyback current
//            causes the large spike in the back EMF voltage immediately after releasing
//            the motor phase from active drive current.
//
// STALL_COUNT_us - This is the shortest expected commutation time. When the motor stalls
//             a false zero cross is detected immediately after every commutation. This causes 
//             the zero cross tracking algorithm to repeatedly decrease the commutation time
//             at each commutation event until the commutation becomes shorter than the motor's 
//             top rated speed. The control loop recognizes this and reverts back to the 
//             startup procedure when it happens.
//
///////////////////////////////////////////////////////////////////////////////
//

#ifndef __MOTOR_SPEC

/************************************************************************
* Motor Definitions                                                               
*************************************************************************/
/* start speed in electrical RPM. However, since timer register is a
 * 16-bit register counting at 1MHz, counting for 1 round can manage
 * the speed at the minimum rate of 160RPM(elec) */
#define ABSOLUTE_MIN_RPM		160
//#define START_RPM               700
#define START_RPM               1000

// percentage of speed request below which the motor will turn off
#define LOW_OFF_REQUEST_PCT       16L

/* MAX_PRAC_DUTY_PCT = determine the maximum percent of allowable pwm duty-cycle.
   As the charge-pump circuits for high-side MOSFETs require a certain amount 
   of PWM off time to recharge capacitors. The maximum PWM duty-cycle should not
   be greater than this value.
*/
#define MAX_PRAC_DUTY_PCT		80L

// STARTUP_DRIVE_PCT = determines initial CCPR1L duty cycle from speed table for motor startup
// NOTE: A CCPR1L number that seems to work best for all applications is 13%
//       Larger numbers tend to make the control algorithm jump past the 
//       ideal commutation rate during startup.
//       Smaller numbers usually do not produce enough torque to start the motor.
// There are three startup values. The one to use will be determined by measuring
// the motor supply voltage before startup.

// Try lower percent such as 13 - 35 to smooth startup speed
#define STARTUP_DRIVE_PCT        20L

// MAX_STARTUP_EVENTS - The startup algorithm starts by single stepping the motor
//             this number of times. This is done to position the motor in a known
//             alignment before higher speed commutation is attempted. At least two
//             steps are needed because at the first step the motor may be prepositioned 
//             in a state where it cannot swing left or right because it's in the middle 
//             of that boundary. The second step assures that the motor will respond.
//
#define MAX_STARTUP_EVENTS          2

// # of phases in the motor (Usually 3 here)
#define NUM_PHASES             3L

// # of commutations in one electrical revolution (Always 6 for most BLDC)
#define COMM_PER_REV			( NUM_PHASES * 2 )

// 60 seconds in one minute
#define SEC_PER_MIN            60L

// startup commutations per minute
#define START_COMM_PER_MIN    (START_RPM*COMM_PER_REV)

// commutations per second at beginning of startup ramp
#define START_COMM_PER_SEC    (START_COMM_PER_MIN/SEC_PER_MIN)

// Timer1 counts per commutation at beginning of startup
#define TMR1_START_COUNT      (TMR1_COUNTS_PER_SEC/START_COMM_PER_SEC)

// initial starup commutation period
#define COMM_TIME_INIT			(TMR1_START_COUNT)

/* Commutation time to RPM factor. Divide this constant with the commutation
time with this value will give the RPM.
	Commutation per second = Timer1 count per sec / Commutation time.
	Commutation per min = Commutation per sec * 60
	RPM = Commutation per min / commutation per rev
*/
#define COMM_TIME_TO_RPM_FACTOR	( ( TMR1_COUNTS_PER_SEC * SEC_PER_MIN ) / COMM_PER_REV )

/* Number of contiguous zero crossing to be considered as stable */
#define EXPECT_ZC_COUNT                 3

/* Number of restarting times if the motor stops working */
#define MAX_OVERCURRENT_RST			2

// blanking count in microseconds
#define BLANKING_COUNT_us		      100U
#define BLANKING_COUNT              (BLANKING_COUNT_us * TMR1_COUNTS_PER_us)

// Number of microseconds to commutate early after zero cross
// this number is subtracted from half the expected commutation time to set commutation event after zero cross
#define ADVANCE_TIMING_us           7L
// Commutation happens in two stages
// Stage 1 is zero cross detection: Commutation is forced 1/2 commutation period after the Z-C event.
// Stage 2 is fixed commutation: Commutation timer is set at beginning for full commutation period.
// Stage 1 takes 12 us to detect Z-C and restuff the commutation timer. FIXED_ADVANCE_us adjusts for that difference.
//#define FIXED_ADVANCE_TIMING_us     ADVANCE_TIMING_us - 0L
#define FIXED_ADVANCE_TIMING_us     15L

#define ADVANCE_COUNT               (ADVANCE_TIMING_us*TMR1_COUNTS_PER_us)
#define FIXED_ADVANCE_COUNT         (FIXED_ADVANCE_TIMING_us*TMR1_COUNTS_PER_us)

/* Define minimum zero-crossing time. If zero-crossing event occurs with
the time less than this value. Our driver cannot perform stable control */
#if ADCANCE_COUNT > FIXED_ADVANCE_COUNT
  #define MIN_ZC_TIME	( ADVANCE_COUNT << 1 )
#else
  #define MIN_ZC_TIME	( FIXED_ADVANCE_COUNT << 1 )
#endif

/* PWM_ROC the maximum amount of PWM change in a control interval.
   Too high value may cause the motor unstable. Lower value makes the motor
   to reach desired speed slower.
*/
#define PWM_ROC			6

////////////////////////////////////////////////////////////////////////////////////////
//// On-Off limits

// +1 forces PWM to 100% duty cycle and adjust to 10 bits
#define MAX_DUTY_CYCLE			((PWM_PERIOD+1)<<2)
#define MAX_PRAC_DUTY_CYCLE		((MAX_PRAC_DUTY_PCT*MAX_DUTY_CYCLE)/100L)
#define STARTUP_DUTY_CYCLE		((STARTUP_DRIVE_PCT*MAX_DUTY_CYCLE)/100L)

#define REQUEST_OFF				((LOW_OFF_REQUEST_PCT*MAX_DUTY_CYCLE)/100L)

/* Spec for VideoRay Thruster */
#define MIN_RPM                 2500
#define MAX_RPM                 8000


#endif /* __MOTOR_SPEC */