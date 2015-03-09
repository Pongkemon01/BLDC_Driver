/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Driver             *
*                                                                       *
*     Author               : Akrappong Patchararungruang                *
*                                                                       *
*     Filename             : BLDC.h                                     *
*     File Description     : Hardware constant for enhanced mid-range   *
*                            PIC16F1936/PIC16F1938                      *
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
*
*************************************************************************
*/

#ifndef BLDC_H
#define BLDC_H

#include <stdint.h>

/******************************************************
 * SYSTEM OSCILLATOR
 ******************************************************/
#define  FOSC_32_MHZ
#define  HIGHSPEED_PWM

// Primary oscillator
#define  FOSC                    32000000L
#define  OSCCON_INIT             0b11110000
// Timer0: 1:64 prescale
#define OPTION_REG_INIT          0b10000101
#define TIMER0_PRESCALE          64L
// Timer1: prescaler 1:8 = 1 us period @32MHz
#define  T1CON_INIT              0b00110000
#define  TMR1_PRESCALE           8L
#ifdef  HIGHSPEED_PWM
    //Timer2: on, 1:1 prescale, 1:1 postscale, 0.125 us period @32MHz
    #define TIMER2_PRESCALE          1L
    #define T2CON_INIT               0b00000100
#else
    //Timer2: on, 1:4 prescale, 1:1 postscale, 0.5 us period @32MHz
    #define TIMER2_PRESCALE          4L
    #define T2CON_INIT               0b00000101
#endif

#define  SYSTEM_FREQUENCY        (FOSC/4L)

/******************************************************
 * Watch Dog Timer
 ******************************************************/
/* Prescale = 1:4096 resulting in WDT period of approximately 132 ms */
#define  SWDTCON_INIT            0b00001110

/******************************************************
 * TIMER0 based
 * Timer0 is used for the TimebaseManager() routine which
 * determines how often each service, other than motor control, 
 * is performed.
 * Services include: Supply monitor, Speed request monitor, warmup time, 
 *                   slow start step interval, and stall check
 ******************************************************/
#ifndef T0IF
#define T0IF TMR0IF
#endif

#define MILLISECONDS_PER_SEC              1000L

// number of milliseconds in each timebase count
// (i.e. resolution of TimebaseManager() in milliseconds)
#define TIMEBASE_MS_PER_COUNT             10

// number of milliseconds in each timebase manager tick clock
#define TIMEBASE_MS_PER_TICK              2L
#define TIMEBASE_LOAD_10ms                (TIMEBASE_MS_PER_COUNT/TIMEBASE_MS_PER_TICK)

// timebase reload value for TimebaseManager() interrupt period: 
#define TIMEBASE_MANAGER_PERIOD_COUNT     ((TIMEBASE_MS_PER_TICK*SYSTEM_FREQUENCY)/(TIMER0_PRESCALE*MILLISECONDS_PER_SEC))

#define TIMEBASE_MANAGER_RELOAD_COUNT     (~TIMEBASE_MANAGER_PERIOD_COUNT)

#if (TIMEBASE_MANAGER_PERIOD_COUNT > 0x00FF)
   #error "TIMER0_PRESCALE is too small. Timebase reload count exceeds 0xFF." 
#endif

#if (TIMEBASE_MANAGER_PERIOD_COUNT < 0x80)
   #warning "TIMER0_PRESCALE can be decreased to improve timebase resolution"
#endif

/******************************************************
 * TimebaseManager times
 ******************************************************/

// all TIMEBASE times must be expressed in even multiples of TIMEBASE_MS_PER_COUNT

// TIMEBASE_STARTUP_ms = number of milliseconds allowed to achieve zero-cross lock
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_EXCITE_ms           30

// TIMEBASE_RAMPUP_ms = number of milliseconds allowed to achieve zero-cross lock
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_RAMPUP_ms           2000

// TIMEBASE_RAMPDOWN_ms = number of milliseconds allowed BLDC to spin freely before stop
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_RAMPDOWN_ms          100

// TIMEBASE_BREAK_ms = number of milliseconds to break BLDC.
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_BREAK_ms				100

// TIMEBASE_COOLDN_ms = number of milliseconds to cooldown the driver before completely idle
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_COOLDN_ms            100

// time for continuous unstable condition to cause stall
// TIMER0_STALL_ms = number of milliseconds in out-of-lock condition to recognize stall
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMER0_STALL_ms               2500
                                          
// number of milliseconds between each motor stall condition check
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_STALLCHECK_ms        500

/******************************************************
 * TIMEBASE times converted to counts
 ******************************************************/
#define TIMEBASE_EXCITE_COUNT		(TIMEBASE_EXCITE_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_RAMPUP_COUNT		(TIMEBASE_RAMPUP_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_RAMPDOWN_COUNT		(TIMEBASE_RAMPDOWN_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_BREAK_COUNT		(TIMEBASE_BREAK_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_COOLDN_COUNT		(TIMEBASE_COOLDN_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_STALL_COUNT		(TIMER0_STALL_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_STALLCHECK_COUNT	(TIMEBASE_STALLCHECK_ms/TIMEBASE_MS_PER_COUNT)

/******************************************************
 * TIMER1 based
 ******************************************************/
#define TMR1_FREQUENCY              (SYSTEM_FREQUENCY/TMR1_PRESCALE)
#define MICROSECONDS_PER_SECOND     1000000UL
#define TMR1_COUNTS_PER_us          (TMR1_FREQUENCY/MICROSECONDS_PER_SECOND)
#define TMR1_COUNTS_PER_SEC         TMR1_FREQUENCY

#define TIMER1_HIGH_GATE_COUNT      0xff

// overhead adjusts for the time lost when reloading the timer
#define TIMER1_OVERHEAD             (12/TIMER1_PRESCALE)

// timer1 load values based on blanking count
#define TIMER1_HIGH_BLANKING_COUNT   (0xFF - (BLANKING_COUNT_us/256L)) 
#define TIMER1_LOW_BLANKING_COUNT   ((0xFFFF - BLANKING_COUNT_us) & 0xFF)

// dwell time allowing flyback currents to settle before BEMF voltage check
#define BEMF_DELAY_COUNT            (BEMF_ACQUISITION_us * TMR1_COUNTS_PER_us)
#define ACQUISITION_DELAY_COUNT     (ACQUISITION_TIME_us * TMR1_COUNTS_PER_us)

/******************************************************
 * PWM and TIMER2
 ******************************************************/

// PWM mode
#define CCP1CON_INIT           0b00001100

#define TIMER2_FREQUENCY       (SYSTEM_FREQUENCY/TIMER2_PRESCALE)

#ifdef  HIGHSPEED_PWM
    // 1/20KHz = 50µS PWM period @ 0.5 us Timer2 period
    #define PWM_FREQ            31250L
#else
// 1/16KHz = 62.5µS PWM period @ 0.5 us Timer2 period
//#define PWM_FREQUENCY          16000L
// 1/20KHz = 50µS PWM period @ 0.5 us Timer2 period
#define PWM_FREQ               20000L
#endif

#define PWM_PERIOD             (((TIMER2_FREQUENCY/PWM_FREQ)-1L)&0xFF)   

 // Stall event trigger
#define MAX_TMR1_PRESET        (~MIN_COMM_TIME)
/*-----------------------------------------------------------*/

/* ECCP */

/* auto shutdown on C2OUT (Over current), drive output pins to 0 */
#define ECCP1AS_INIT         0b00100000

/* auto restart */
#define PWM1CON_INIT         0b10000000

/* Over-current indicator */
#define OC_STAT			CCP1ASE

/******************************************************
 * I/O Definition
 ******************************************************/
/* Device PIC16F1936/PIC16F1938 TQFP Pin assignments:
 * 
 * Pin #   PORT ID I/O Use  Name    Description
 *------   ------- --- ---  ------- --------------------------
 *  27       RA0    I  AN0  C12IN0- BEMF A 
 *  28       RA1    I  AN1  C12IN1- BEMF B 
 *   1       RA2    O       RA2     (unused) 
 *   2       RA3    I  AN3  C1IN+   BEMF Reference
 *   3       RA4    O       C1OUT   ZC Test point
 *   4       RA5    I       SS      SPI Slave Select
 *   5       RA6    O       RA6     LED2 (Red)
 *   6       RA7    O       RA7     LED1 (Green)
 *  18       RB0    O       RB0/INT (unused)
 *  19       RB1    I  AN10 C12IN3- Current sense
 *  20       RB2    O       P1B     Low Side Drive B 
 *  21       RB3    I  AN9  C12IN2- BEMF C
 *  22       RB4    O       P1D     Low Side Drive C
 *  23       RB5    O       RB5     High Side Drive A
 *  24       RB6    O       PGC     ICSP Clock
 *  25       RB7    O       PGD     ICSP Data
 *   8       RC0    O       RC0     LED3 (Orange)
 *   9       RC1    O       RC1     LED4 (Yellow)
 *  10       RC2    O       P1A     Low Side Drive A  
 *  11       RC3    I       SCK     SPI Clock
 *  12       RC4    I       SDI     SPI Data Input
 *  13       RC5    O       SDO     SPI Data Output
 *  14       RC6    O       RC6     High Side Drive B
 *  15       RC7    O       RC7     High Side Drive C
 *  26       RE3    I       MCLR    MCLR (Operates without any config)
 *
 */
/*-----------------------------------------------------------*/

// PORTA (PORT) 
#define   TRISA_INIT       0b00101011
#define   PORTA_INIT       0b00000000
#define   TRISA_ERROR      0b11111111
#define   PORTA_ERROR      0b11111111
/*-----------------------------------------------------------*/

// PORTB (PORT)
#define   TRISB_INIT       0b00001010
#define   PORTB_INIT       0b00000000
#define   TRISB_ERROR      0b11111111
#define   PORTB_ERROR      0b11111111
/*-----------------------------------------------------------*/

// PORTC (PORT)
#define   TRISC_INIT       0b00011000
#define   PORTC_INIT       0b00000000
#define   TRISC_ERROR      0b11111111
#define   PORTC_ERROR      0b11111111
/*-----------------------------------------------------------*/

/******************************************************
 * COMPARATORS
 ******************************************************/
 
/* redefine comparator definitions to ease selection between C1 and C2
 CMx is used for BEMF sense
 CMy is used for current sense
*/
#define CMxCON0      CM1CON0
#define CMxCON1      CM1CON1
#define CxIE         C1IE
#define CxIF         C1IF
#define CxOUT        MC1OUT
#define COMPARATOR   CMxCON1

#define CMyCON0      CM2CON0
#define CMyCON1      CM2CON1
#define CyIE         C2IE
#define CyIF         C2IF
#define CyOUT        MC2OUT
/*-----------------------------------------------------------*/

/*
 Comparator bit definitions 
*/
// CMxCON0
#define CxON            0b10000000
#define CxOE            0b00100000
#define CxINV           0b00010000
#define CxFAST          0b00000100
#define CxHYST          0b00000010
#define CxSYNC          0b00000001

// CMxCON1
#define CxINTP          0b10000000
#define CxINTN          0b01000000
#define CxPIN           0b00000000
#define CxCDAC          0b00010000  
#define CxFVR           0b00100000
#define CxGND           0b00110000
#define CxIN0           0b00000000
#define CxIN1           0b00000001
#define CxIN2           0b00000010
#define CxIN3           0b00000011
/*-----------------------------------------------------------*/

/* Comparator initializations */

/* BEMF comparator initialization */
#define  CMxCON0_INIT         CxON | CxOE | CxFAST
#define  CMxCON1_INIT         SENSE_V_RISING

/* Overcurrent sense comparator initialization */
#define CMyCON0_INIT    CxON | CxFAST | CxINV 
#define CMyCON1_INIT	CxINTP | CxCDAC | CxIN3     

/*-----------------------------------------------------------*/
/******************************************************
 * FVR, DAC, and Temperature sensor
 ******************************************************/
#define FVRCON_INIT		0b11111000	/* Out 2.048V to DAC. Also turn-on temperature sensor */
#define DACCON0_INIT	0b11001000	/* Use FVR as ref+ and GND as ref- */
#define DAC_5A                  8
#define DAC_12A			20			/* DAC value for 12A */
#define DAC_14A			23			/* DAC value for 14A */
#define DAC_DEFAULT_CURRENT	DAC_14A

/******************************************************
 * ADC
 ******************************************************/

#define ANSELA_INIT          0b00001011
#define ANSELB_INIT          0b00001010

#define ADCON0_INIT		0b01110101	/* Use temperature sensor as input */
#define ADCON1_INIT		0b10100000	/* Fosc/32 = 1us switching step, vdd-gnd as ref. */

#define OC_ADCOUT		597	/* Set to about 65 degree celcius */

/*-----------------------------------------------------------*/

/******************************************************
 * Driver and comparator sense states
 ******************************************************/

/*
 Phase               A        B        C
 PWM Side Drive   P1A/RC2  P1B/RB2  P1D/RB4
 Fixed Side Drive   RB5      RC6      RC7
 BEMF Sense       C12IN0-  C12IN1-  C12IN2-
*/

#ifndef PSTRCON
#define PSTRCON PSTR1CON
#endif

// PORT control of drive pins
#define DRIVE_A              LATB5
#define DRIVE_B              LATC6
#define DRIVE_C              LATC7
#define DRIVE_U              DRIVE_A
#define DRIVE_V              DRIVE_B
#define DRIVE_W              DRIVE_C

// PWM Drive steering
#define MODULATE_U_BIT       LATC2
#define MODULATE_V_BIT       LATB2
#define MODULATE_W_BIT       LATB4

#define MODULATE_A           0b00000001
#define MODULATE_B           0b00000010
#define MODULATE_C           0b00001000
#define MODULATE_U           MODULATE_A
#define MODULATE_V           MODULATE_B
#define MODULATE_W           MODULATE_C
#define MODULATE_OFF         0

#define SENSE_A_RISING       CxINTP | CxPIN | CxIN0
#define SENSE_B_RISING       CxINTP | CxPIN | CxIN1
#define SENSE_C_RISING       CxINTP | CxPIN | CxIN2
#define SENSE_U_RISING       SENSE_A_RISING
#define SENSE_V_RISING       SENSE_B_RISING
#define SENSE_W_RISING       SENSE_C_RISING

#define SENSE_A_FALLING      CxINTN | CxPIN | CxIN0
#define SENSE_B_FALLING      CxINTN | CxPIN | CxIN1
#define SENSE_C_FALLING      CxINTN | CxPIN | CxIN2
#define SENSE_U_FALLING      SENSE_A_FALLING
#define SENSE_V_FALLING      SENSE_B_FALLING
#define SENSE_W_FALLING      SENSE_C_FALLING

#define SENSE_A               SENSE_A_RISING
#define SENSE_B               SENSE_B_RISING
#define SENSE_C               SENSE_C_RISING
#define SENSE_U               SENSE_A_RISING
#define SENSE_V               SENSE_B_RISING
#define SENSE_W               SENSE_C_RISING

/*-----------------------------------------------------------*/

/******************************************************
 * SPI
 ******************************************************/
/* SSP slave + SS enabled, Clock idle = high */
#define SSPCON1_INIT		0b00110100
#define SS_PIN				RA5

/* SPI Command code */
#define SPI_COMMAND_MASK	0b11100000U
#define SPI_PARAM_MASK		0b00011111U

#define SPI_SET_CURRENT		0b11100000U
#define SPI_SET_SPEED		0b11000000U
#define SPI_GET_SPEED		0b10100000U
#define SPI_NOP				0b10000000U

/* Undocumented commands */
#define SPI_UNDOC_MASK		0b00011100U
#define SPI_UNDOC_PARAM_MASK 0b00000011U
#define SPI_UNDOC_SPEED_M	0b00000100U
#define SPI_UNDOC_PWM_M		0b00001000U
#define SPI_UNDOC_SET_PWM	0b00001100U
#define SPI_UNDOC_GET_PWM	0b00010000U
#define SPI_UNDOC_REVERSE	0b00010100U
/*-----------------------------------------------------------*/

/******************************************************
 * Status LEDs
 * Green - POR or BLDC is running
 * Red - BLDC stalls
 ******************************************************/
#define LED_OK                LATA7
#define LED_ERROR             LATA6
#define LED_OVERCURRENT       LATC0
#define LED_OVERTEMP          LATC1

/* Mask for status bits in SPI data */
#define LED_ERROR_MASK			0b01000000
#define LED_OVERCURRENT_MASK	0b00100000
#define LED_OVERTEMP_MASK		0b00010000
#define LED_OK_MASK				0b10000000


/******************************************************
 * BLDC Motor definitions
 ******************************************************/
#include "Motor_Spec.h"

#endif /* BLDC_H */