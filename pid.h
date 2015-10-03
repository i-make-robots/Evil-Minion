#ifndef PID_H
#define PID_H
//--------------------------------------------------------
// Evil Minion 5 Axis robot firmware
// dan@marginallyclever.com 
// 2015 September 3
// see http://evilminion.info/ for more information.
//--------------------------------------------------------



#include "config.h"



struct PIDobject {
  float integral;
  float last_error;
};



void PID_init(PIDobject &o);
float PID_step(PIDobject &o, float wanted,float actual,float dt);



#endif  // PID_H
