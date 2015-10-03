#ifndef MOTOR_H
#define MOTOR_H
//--------------------------------------------------------
// Evil Minion 5 Axis robot firmware
// dan@marginallyclever.com 
// 2015 September 3
// see http://evilminion.info/ for more information.
// GPL CC-BY-SA-NC
//--------------------------------------------------------



#include "config.h"


extern char *motor_letters;
extern PIDobject pid[NUM_AXIES];

extern int servo_angle;


void motor_setup();
void servo_move(int angle);

void test_stepper(int dir,int ste,int ena,int wait);
void test_dual_steppers(int dir1,int ste1,int ena1,
                         int dir2,int ste2,int ena2,int wait);
void test_actuator(int ina,int pwm,int inb,int wait);
void test_motors();

void motor_enable();
void motor_disable();
void motor_move(int motor_index,float pid_adjust);
void motor_all_stop();

void tick_motors();


#endif  // MOTOR_H
