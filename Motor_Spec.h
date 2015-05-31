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
// ERROR_SCALE - Feedback scaling factor.
//             The error is the difference between the expected zero cross time and the
//             actual zero cross time. The error is scaled down before it is accumulated into
//             into the commutation time. 2 raised to ERROR_SCALE (2^ERROR_SCALE) is the scaling
//             division factor. If the scaling factor is too large then the motor response will
//             be slow. If the scaling factor is too small then the motor may become unstable.
//             If the motor frequently misses lock after the startup sequence then this number is 
//             probably too small. If the motor loses lock at high speed or during acceleratioin
//             then this number is probably too large.
//
///////////////////////////////////////////////////////////////////////////////
//

/************************************************************************
* Motor Definitions                                                               
*************************************************************************/
/* start speed in electrical RPM. However, since timer register is a
 * 16-bit register counting at 1MHz, counting for 1 round can manage
 * the speed at the minimum rate of 160RPM(elec) */
#define START_RPM               300L

// STARTUP_DRIVE_PCT = determines initial CCPR1L duty cycle from speed table for motor startup
// NOTE: A CCPR1L number that seems to work best for all applications is 13%
//       Larger numbers tend to make the control algorithm jump past the 
//       ideal commutation rate during startup.
//       Smaller numbers usually do not produce enough torque to start the motor.
// There are three startup values. The one to use will be determined by measuring
// the motor supply voltage before startup.
#define STARTUP_DRIVE_PCT        45L

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

// +1 forces PWM to 100% duty cycle and adjust to 10 bits
#define MAX_DUTY_CYCLE        ((PWM_PERIOD+1)<<2)

#define STARTUP_DUTY_CYCLE        ((STARTUP_DRIVE_PCT*MAX_DUTY_CYCLE)/100L)

// number of slow commutations between warmup and startup                                          
#define EXCITE_STEPS                  2

#define EXPECT_ZC_COUNT                 3

#define MAX_OVERCURRENT_RST			2

// blanking count in microseconds
#define BLANKING_COUNT_us		      100U
#define BLANKING_COUNT              (BLANKING_COUNT_us * TMR1_COUNTS_PER_us)

// stall commutation time in microseconds
#define  STALL_COUNT_us             100UL

// # of Timer1 counts below which a stall condition is detected
#ifndef MICROSECONDS_PER_SECOND
    #define MICROSECONDS_PER_SECOND     1000000UL
#endif
#define MIN_COMM_TIME               ((STALL_COUNT_us * TMR1_COUNTS_PER_SEC)/MICROSECONDS_PER_SECOND)

// The raw error is divided by 2 to the power of ERROR_SCALE before accumulating.
// Example: If the raw error is 96 and ERROR_SCALE is 3 then the error correction that is accumulated
// is reduced to 96/2^3 or 12.
#define ERROR_SCALE                 3

// Number of microseconds to commutate early after zero cross
// this number is subtracted from half the expected commutation time to set commutation event after zero cross
#define ADVANCE_TIMING_us           0L
// Commutation happens in two stages
// Stage 1 is zero cross detection: Commutation is forced 1/2 commutation period after the Z-C event.
// Stage 2 is fixed commutation: Commutation timer is set at beginning for full commutation period.
// Stage 1 takes 12 us to detect Z-C and restuff the commutation timer. FIXED_ADVANCE_us adjusts for that difference.
#define FIXED_ADVANCE_TIMING_us     ADVANCE_TIMING_us - 0L

#define ADVANCE_COUNT               (ADVANCE_TIMING_us*TMR1_COUNTS_PER_us)
#define FIXED_ADVANCE_COUNT         (FIXED_ADVANCE_TIMING_us*TMR1_COUNTS_PER_us)

////////////////////////////////////////////////////////////////////////////////////////
//// On-Off limits

// percentage of speed request below which the motor will turn off
#define LOW_OFF_REQUEST_PCT         12L

#define REQUEST_OFF					((LOW_OFF_REQUEST_PCT*MAX_DUTY_CYCLE)/100L)

/* Spec for VideoRay Thruster */
#define MIN_RPM                 3000
#define MAX_RPM                 8000
