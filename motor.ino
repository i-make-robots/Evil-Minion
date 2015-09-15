#include "motor.h"
#include "sensor.h"
#include <Servo.h>



char *motor_letters = "ABCDE";
PIDobject pid[NUM_AXIES];

// @TEST Servo stuff
Servo toolServo;
int servo_angle;



void servo_setup() {
  toolServo.attach(PIN_ATC_TX);
  servo_angle=120;
  toolServo.write(servo_angle);
}


void servo_move(int angle) {
  if(angle<120) angle=120;
  if(angle>170) angle=170;
  servo_angle = angle;
  toolServo.write(angle);
}


void test_stepper(int dir,int ste,int ena,int wait) {
  int i;
  
  // Activate stepper
  //digitalWrite(ena,LOW);
  // set forward
  digitalWrite(dir,HIGH);
  for(i=0;i<400;++i) {
    digitalWrite(ste,HIGH);
    digitalWrite(ste,LOW);
    delay(wait);
  }
  // set backward
  digitalWrite(dir,LOW);
  for(i=0;i<400;++i) {
    digitalWrite(ste,HIGH);
    digitalWrite(ste,LOW);
    delay(wait);
  }
  // Disable stepper
  //digitalWrite(ena,HIGH);
}


void test_dual_steppers(int dir1,int ste1,int ena1,
                         int dir2,int ste2,int ena2,int wait) {
  int i;
  
  // Activate stepper
  //digitalWrite(ena,LOW);
  // set forward
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);
  for(i=0;i<400;++i) {
    digitalWrite(ste1,HIGH);
    digitalWrite(ste1,LOW);
    digitalWrite(ste2,HIGH);
    digitalWrite(ste2,LOW);
    delay(wait);
  }
  // set backward
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);
  for(i=0;i<400;++i) {
    digitalWrite(ste1,HIGH);
    digitalWrite(ste1,LOW);
    digitalWrite(ste2,HIGH);
    digitalWrite(ste2,LOW);
    delay(wait);
  }
  // Disable stepper
  //digitalWrite(ena,HIGH);
}


void test_actuator(int ina,int pwm,int inb,int wait) {
  int i;
  
  // out
  Serial.println("out");
  digitalWrite(ina,HIGH);
  digitalWrite(inb,LOW);
  for(i=0;i<255;++i) {
    Serial.println(i);
    analogWrite(pwm,i);
    delay(wait);
  }
  for(i=0;i<255;++i) {
    analogWrite(pwm,255-i);
    delay(wait);
  }

  // in
  Serial.println("in");
  digitalWrite(ina,LOW);
  digitalWrite(inb,HIGH);
  for(i=0;i<255;++i) {
    Serial.println(i);
    analogWrite(pwm,i);
    delay(wait);
  }
  for(i=0;i<255;++i) {
    analogWrite(pwm,255-i);
    delay(wait);
  }
  digitalWrite(inb,LOW);
  digitalWrite(ina,LOW);
}



void test_motors() {
  test_actuator(PIN_D_INA,PIN_D_PWM,PIN_D_INB, 4);
  test_actuator(PIN_C_INA,PIN_C_PWM,PIN_C_INB, 4);
  
  //test_stepper(PIN_E_DIR,PIN_E_STE,PIN_E_ENA, 5);  // E
  //test_stepper(PIN_B_DIR,PIN_B_STE,PIN_B_ENA, 5);  // B
  //test_stepper(PIN_A_DIR,PIN_A_STE,PIN_A_ENA, 5);  // A
  
  //test_dual_steppers(PIN_B_DIR,PIN_B_STE,PIN_B_ENA,
  //                   PIN_A_DIR,PIN_A_STE,PIN_A_ENA,5);
}


void test_servo_wave() {
  toolServo.write(20);
  delay(1000);
  toolServo.write(160); 
  delay(1000);
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


// turn on power to the motors (make them immobile)
void motor_enable() {
  Serial.println(F("NOT IMPLEMENTED"));
  while(1);
}


// turn off power to the motors (make them move freely)
void motor_disable() {
  Serial.println(F("NOT IMPLEMENTED"));
  while(1);
}


void motor_step_A(int dir,float pid_adjust) {
  digitalWrite(PIN_A_DIR, (dir>0) ? HIGH : LOW);
  while(pid_adjust>0) {
    pid_adjust--;
    digitalWrite(PIN_A_STE,HIGH);
    digitalWrite(PIN_A_STE,LOW);
  }
}


void motor_step_B(int dir,float pid_adjust) {
  digitalWrite(PIN_B_DIR, (dir>0) ? HIGH : LOW);
  while(pid_adjust>0) {
    pid_adjust--;
    digitalWrite(PIN_B_STE,HIGH);
    digitalWrite(PIN_B_STE,LOW);
  }
}


void motor_step_E(int dir,float pid_adjust) {
  digitalWrite(PIN_E_DIR, (dir>0) ? HIGH : LOW);
  while(pid_adjust>0) {
    pid_adjust--;
    digitalWrite(PIN_E_STE,HIGH);
    digitalWrite(PIN_E_STE,LOW);
  }
}


void motor_move(int motor_index,float pid_adjust) {
  float v;
  float dir = (pid_adjust>0) ? 1.0f : -1.0f;
  pid_adjust *= dir;
  
  switch(motor_index) {
    case 0:  // a
      motor_step_A(-dir,pid_adjust);
      motor_step_B(-dir,pid_adjust);
      sensors_expected[0] += dir*pid_adjust / WRIST_STEPS_PER_TURN;
      if(sensors_expected[0] == destination[0]) break;
    break;
    case 1:  // b
      motor_step_A(dir,pid_adjust);
      motor_step_B(-dir,pid_adjust);
      sensors_expected[1] += dir*pid_adjust / WRIST_STEPS_PER_TURN;
      if(sensors_expected[1] == destination[1]) break;
    break;
    case 2:  // c
      digitalWrite(PIN_C_INA,(dir>0) ? HIGH : LOW);
      digitalWrite(PIN_C_INB,(dir>0) ? LOW : HIGH);
      v = fabs(pid_adjust) * (255.0-ACTUATOR_C_MIN_PWM) + ACTUATOR_C_MIN_PWM;
      v = max(0,min(255,v));
      analogWrite(PIN_C_PWM,v);
      sensors_expected[2] += dir * SENSOR_ANGLE_PER_BIT;
    break;
    case 3:  // d
      digitalWrite(PIN_D_INA,(dir>0) ? HIGH : LOW);
      digitalWrite(PIN_D_INB,(dir>0) ? LOW : HIGH);
      v = fabs(pid_adjust) * (255.0-ACTUATOR_D_MIN_PWM) + ACTUATOR_D_MIN_PWM;
      v = max(0,min(255,v));
      analogWrite(PIN_D_PWM,v);
      sensors_expected[3] += dir * SENSOR_ANGLE_PER_BIT;
    break;
    case 4:  // e
      motor_step_E(dir,pid_adjust);
      sensors_expected[4] += dir*pid_adjust / ANCHOR_STEPS_PER_TURN;
      if(sensors_expected[4] == destination[4]) break;
    break;
  }
}


void motor_all_stop() {
  analogWrite(PIN_C_PWM,0);
  analogWrite(PIN_D_PWM,0);
  digitalWrite(PIN_C_INA,LOW);
  digitalWrite(PIN_C_INB,LOW);
  digitalWrite(PIN_D_INA,LOW);
  digitalWrite(PIN_D_INB,LOW);
}




void tick_motors(float dt) {
  int i;
  float v;
  
  for(i=0;i<NUM_AXIES;++i) {
    if( move_active[i]==0 ) continue;
    
//    Serial.print(i);
//    Serial.print(":");
    
    v = PID_step(pid[i], destination[i],sensors_raw[i],dt);
/*
    Serial.print("D=");
    Serial.print(destination[i]);
    Serial.print(",RAW=");
    Serial.print(sensors_raw[i]);
    Serial.print(",V=");
    Serial.print(v);
//*/
    motor_move(i,v);
//    Serial.print("\n");
    if(fabs(destination[i]-sensors_raw[i]) <= SENSOR_ANGLE_PER_BIT ) {
      //Serial.print("Stopping ");
      //Serial.println(i);
      sensors_expected[i] = sensors_raw[i];
      move_active[i]=0;
      if(i==2) {
        digitalWrite(PIN_C_INA,LOW);
        digitalWrite(PIN_C_INB,LOW);
      } else if(i==3) {
        digitalWrite(PIN_D_INA,LOW);
        digitalWrite(PIN_D_INB,LOW);
      }
    }
  }
}
