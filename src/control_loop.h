#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>

void control_timer_start(void);
void control_timer_stop(void);
void control_loop_init(void);
void kalman_timer_start(void);
void kalman_timer_stop(void);
void control_set_speed(float);

void aim_timer_start(void);
void aim_timer_stop(void);
void aim_loop_init(void);

void servo_timer_start(void);
void servo_timer_stop(void);
void servo_loop_init(void);

#endif
