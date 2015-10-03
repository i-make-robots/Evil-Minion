//--------------------------------------------------------
// Evil Minion 5 Axis robot firmware
// dan@marginallyclever.com 
// 2015 September 3
// see http://evilminion.info/ for more information.
//--------------------------------------------------------
#include "pid.h"





void PID_init(PIDobject &o) {
  o.integral=0;
  o.last_error=0;
}


float PID_step(PIDobject &o, float wanted,float actual,float dt) {
  float error = wanted - actual;

  if( fabs(error) > PID_EPSILON ) {
    o.integral += error * dt;
  }
  
  float derivative = ( error - o.last_error ) / dt;
  o.last_error = error;

//  Serial.print("P=");
//  Serial.print(error);
//  Serial.print("I=");
//  Serial.print(o.integral);
//  Serial.print("D=");
//  Serial.println(derivative);

  return PID_KP * error + PID_KI * o.integral + PID_KD * derivative;
}
