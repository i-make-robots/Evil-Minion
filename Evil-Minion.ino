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
  comms_setup();
  loadConfig();
  motor_setup();
  setup_sensors();
  
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    // PIDs
    PID_init(pid[i]);
    move_active[i]=0;
  } 
  
  compliant_mode = 0;
  continuous_reporting = 0;
  compliance_limit = COMPLIANCE_DEFAULT_EPSILON;
  
  help();
  ready();
  
  m0 = millis();
}


void loop() {
  tick_comms();
  tick_sensors();
  
  // post a position update every 50ms.  No need to overdo it...
  m1 = millis();
  float dt = ((m1-m0)*0.001);
  if(dt >= 0.05) {
    m0 = m1;
    if(continuous_reporting==1) {
      where();
    }
  }

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


void respond_to_push() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {}
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
    ds = sensors_expected[i] - sensors_filtered[i];
    
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


