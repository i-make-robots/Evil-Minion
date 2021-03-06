#ifndef CONFIG_H
#define CONFIG_H
//--------------------------------------------------------
// Evil Minion 5 Axis robot firmware
// dan@marginallyclever.com 
// 2015 September 3
// see http://evilminion.info/ for more information.
//--------------------------------------------------------

#include <Arduino.h>

#define INVERT_C  // flip C linear actuator direction
#define INVERT_D  // flip D linear actuator direction

// MISC
#define PIN_LED   (13)

// MOTORS & ACTUATORS
#define PIN_A_DIR (46)
#define PIN_A_STE (48)
#define PIN_A_ENA (50)

#define PIN_B_DIR (29)
#define PIN_B_STE (31)
#define PIN_B_ENA (33)

#define PIN_C_PWM (3)
#ifdef INVERT_C
#define PIN_C_INA (2)
#define PIN_C_INB (4)
#else
#define PIN_C_INA (4)
#define PIN_C_INB (2)
#endif

#define PIN_D_PWM (9)
#ifdef INVERT_D
#define PIN_D_INA (8)
#define PIN_D_INB (10)
#else
#define PIN_D_INA (10)
#define PIN_D_INB (8)
#endif

#define PIN_E_DIR (35)
#define PIN_E_STE (37)
#define PIN_E_ENA (39)


// SENSORS
#define PIN_SENSOR_CLK (20)

#define PIN_SENSOR_SDOUT_A (22)
#define PIN_SENSOR_SDOUT_B (23)
#define PIN_SENSOR_SDOUT_C (24)  //ok
#define PIN_SENSOR_SDOUT_D (25)
#define PIN_SENSOR_SDOUT_E (26)  //ok

#define PIN_SENSOR_A_CSEL (14)
#define PIN_SENSOR_B_CSEL (15)
#define PIN_SENSOR_C_CSEL (16) //ok
#define PIN_SENSOR_D_CSEL (17) //ok
#define PIN_SENSOR_E_CSEL (21) //ok



// SENSORS
#define SENSOR_PARITY_TEST_ON
#define SENSOR_TOTAL_BITS    (18)  // 18 bits of data
#define SENSOR_ANGLE_BITS    (12)
#define SENSOR_ERROR_BITS    (5)
#define SENSOR_PARITY_BITS   (1)
#define SENSOR_STATUS_BITS   (SENSOR_ERROR_BITS+SENSOR_PARITY_BITS)
#define SENSOR_ANGLE_MASK    (0b111111111111000000)
#define SENSOR_ERROR_MASK    (0b000000000000111110)
#define SENSOR_PARITY_MASK   (0b000000000000000001)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)(1<<SENSOR_ANGLE_BITS))


// ATC
//#define PIN_ATC_GND        // ATC pin 1
#define PIN_ATC_RX     (7)   // ATC pin 2
#define PIN_ATC_TX     (6)   // ATC pin 3
#define PIN_ATC_EN_5V  (12)  // ATC pin 4
#define PIN_ATC_EN_12V (11)  // ATC pin 5


// PID tuning
#define PID_EPSILON (0.001)
#define PID_KP (0.25)
#define PID_KI (0.05)
#define PID_KD (0.0001)


// Comms
#define BAUD_RATE (57600)
#define MAX_BUF   (64)


// Misc
#define NUM_AXIES (5)
#define NUM_TOOLS            (6)
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead
#define MM_PER_SEGMENT       (10)  // Arcs are split into many line segments.  How long are the segments?


// Stepper math
#define MOTOR_STEPS_PER_TURN   (400.0)
#define MOTOR_MICROSTEPPING    (16.0)
#define TOTAL_STEPS_PER_TURN   (MOTOR_STEPS_PER_TURN*MOTOR_MICROSTEPPING)
#define WRIST_GEAR_RATIO       (48.0/8.0)  // large gear # teeth / small gear # teeth
#define ANCHOR_GEAR_RATIO      (80.0/8.0)  // large gear # teeth / small gear # teeth
#define WRIST_STEPS_PER_TURN   (TOTAL_STEPS_PER_TURN*WRIST_GEAR_RATIO)
#define ANCHOR_STEPS_PER_TURN  (TOTAL_STEPS_PER_TURN*ANCHOR_GEAR_RATIO)

#define MAX_FEEDRATE         (30000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (1000)
#define DEFAULT_FEEDRATE     (8500.0)
#define DEFAULT_ACCELERATION (250)


// Actuator math
// The minimum PWM signal to send that will move an actuator.  0-255, inclusive.
#define ACTUATOR_C_MIN_PWM  (15.0)
#define ACTUATOR_D_MIN_PWM  (25.0)
#define ACTUATOR_C_MAX_PWM  (255.0)
#define ACTUATOR_D_MAX_PWM  (255.0)

#define COMPLIANCE_DEFAULT_EPSILON (0.1)  // get within this many degrees of destination before stopping

#define DEFAULT_IS_ABSOLUTE_ON (1)  // set 1 for on, 0 for off



//------------------------------------------------------------------------------
// PHYSICAL LIMITS & SOFTWARE LIMITS
//------------------------------------------------------------------------------
// The physical limits are used to calibrate the machine.

#define ANGLE_B_MAX (360-72.90)
#define ANGLE_B_MIN (72.9)
#define ANGLE_C_MAX (160.31)
#define ANGLE_C_MIN (50.57)
#define ANGLE_D_MAX (173.6)
#define ANGLE_D_MIN (87.85)
#define ANGLE_E_MAX (180+165)
#define ANGLE_E_MIN (180-165)
/*
// The software limits may (and probably should) be less than the physical limits.
#define ANGLE_A_MAX (270)
#define ANGLE_A_MIN (90)
#define ANGLE_B_MAX (280)
#define ANGLE_B_MIN (80)
#define ANGLE_C_MAX (160.31)
#define ANGLE_C_MIN (50.57)
#define ANGLE_D_MAX (173.6)
#define ANGLE_D_MIN (87.85)
#define ANGLE_E_MAX (180+165)
#define ANGLE_E_MIN (180-165)
*/


//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define EEPROM_VERSION   1                         // Increment EEPROM_VERSION when adding new variables

#define ADDR_VERSION     0                         // address of the version number (one byte)
#define ADDR_GUID        (ADDR_VERSION+1)          // address of the UUID (long - 4 bytes)
// sensor adjustments
#define ADDR_ADJ_A (ADDR_GUID+4)
#define ADDR_ADJ_B (ADDR_ADJ_A+4)
#define ADDR_ADJ_C (ADDR_ADJ_B+4)
#define ADDR_ADJ_D (ADDR_ADJ_C+4)
#define ADDR_ADJ_E (ADDR_ADJ_D+4)



//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------
// for timer interrupt control
#define CLOCK_FREQ            (16000000L)
#define MAX_COUNTER           (65536L)
// time passed with no instruction?  Make sure PC knows we are waiting.
#define TIMEOUT_OK            (1000)

// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START



//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern long robot_uid;
extern float destination[NUM_AXIES]; 
extern char move_active[NUM_AXIES];
extern float compliance_time[NUM_AXIES];
extern char continuous_reporting;
extern char compliant_mode;


#endif // CONFIG_H
