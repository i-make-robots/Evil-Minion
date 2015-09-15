// Arm5 firmware



#include "config.h"
#include "pid.h"
#include "sensor.h"
#include "motor.h"



// movement control
float destination[NUM_AXIES]; 
char move_active[NUM_AXIES];

// timing
long m0, m1;

// misc
long robot_uid=0;


char continuous_reporting;
char compliant_mode;
float compliance_limit;



void setup() {
  // STEPPER MOTORS & LINEAR ACTUATORS
  // A
  pinMode(PIN_A_DIR,OUTPUT);
  pinMode(PIN_A_STE,OUTPUT);
  pinMode(PIN_A_ENA,OUTPUT);
  // B
  pinMode(PIN_B_DIR,OUTPUT);
  pinMode(PIN_B_STE,OUTPUT);
  pinMode(PIN_B_ENA,OUTPUT);
  // C  
  pinMode(PIN_C_INA,OUTPUT);
  pinMode(PIN_C_PWM,OUTPUT);
  pinMode(PIN_C_INB,OUTPUT);
  // D
  pinMode(PIN_D_INA,OUTPUT);
  pinMode(PIN_D_PWM,OUTPUT);
  pinMode(PIN_D_INB,OUTPUT);
  // E
  pinMode(PIN_E_DIR,OUTPUT);
  pinMode(PIN_E_STE,OUTPUT);
  pinMode(PIN_E_ENA,OUTPUT);

  motor_all_stop();
  
  
  // SENSORS
  pinMode(PIN_SENSOR_CLK,OUTPUT);
  pinMode(PIN_SENSOR_SDOUT_E,INPUT);
  pinMode(PIN_SENSOR_SDOUT_D,INPUT);

  pinMode(PIN_SENSOR_A_CSEL,OUTPUT);  // A
  pinMode(PIN_SENSOR_B_CSEL,OUTPUT);  // B
  pinMode(PIN_SENSOR_C_CSEL,OUTPUT);  // C
  pinMode(PIN_SENSOR_D_CSEL,OUTPUT);  // D
  pinMode(PIN_SENSOR_E_CSEL,OUTPUT);  // E

  
  comms_setup();
  loadConfig();
  setup_sensors();
  tick_sensors();
  
 
  
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    // PIDs
    PID_init(pid[i]);
    move_active[i]=0;
    sensors_expected[i] = sensors_raw[i];
  } 
  
  compliant_mode = 0;
  continuous_reporting = 0;
  compliance_limit = COMPLIANCE_DEFAULT_EPSILON;
  
  help();
  ready();
  
  m0 = millis();
}


void test_piston_C(int dir,int pwm) {
  digitalWrite(PIN_C_INA,(dir>0) ? HIGH : LOW);
  digitalWrite(PIN_C_INB,(dir>0) ? LOW : HIGH);
  analogWrite(PIN_C_PWM,pwm);
}


void test_piston_D(int dir,int pwm) {
  digitalWrite(PIN_D_INA,(dir>0) ? HIGH : LOW);
  digitalWrite(PIN_D_INB,(dir>0) ? LOW : HIGH);
  analogWrite(PIN_D_PWM,pwm);
}


void test_piston() {
  int dir, pwm;
  
  for(pwm=0;pwm<255;++pwm) {
    Serial.println(pwm);
    //test_piston_D(1,pwm);
    //test_piston_D(-1,pwm);
    test_piston_C(1,pwm);
    //test_piston_C(-1,pwm);
    delay(100);
  }
}


void loop() {
  tick_comms();
  tick_sensors();
  
  if(continuous_reporting==1) {
    where();
  }
  
  m1 = millis();
  float dt = ((m1-m0)*0.001);
  if(dt >= 0.01) {
    m0 = m1;
    
    char can_comply=comply();
    if( is_target_set() ) {
      if( can_comply ) {
        // move motors
        tick_motors(dt);
      } else {
        // collision!
        Serial.println(F("Collision"));
        motor_all_stop();
      }
    } else {
      // no target set
      if( can_comply==0 ) {
        // pushed
        Serial.println(F("Pushed"));
        respond_to_push();
      }
    }
  }
}


void respond_to_push() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    
  }
}


char is_target_set() {
  int i;
  
  for(i=0;i<NUM_AXIES;++i) {
    if( move_active[i]!=0 ) return 1;
  }
  return 0;
}


/**
 * Compliance test.
 */
char comply() {
  // if compliance test is off, allow movement
  if(compliant_mode==0) return 1;
  
  int i;
  float ds;
  char can_comply=1;
  
  for(i=0;i<NUM_AXIES;++i) {
    // if expected sensor too far from actual sensor then interference
    ds = sensors_expected[i] - sensors_raw[i];
    
    if(fabs(ds) > compliance_limit) {
      Serial.print(i);
      Serial.print('=');
      Serial.println(ds);
      can_comply=0;
    }
  }
  
  if(can_comply==0) Serial.println(F("Can't comply"));
  
  return can_comply;
}


