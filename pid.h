#ifndef PID_H
#define PID_H



#include "config.h"



struct PIDobject {
  float integral;
  float last_error;
};



void PID_init(PIDobject &o);
float PID_step(PIDobject &o, float wanted,float actual,float dt);



#endif  // PID_H
