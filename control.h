#ifndef CONTROL_H
#define CONTROL_H

#include "_pid.h"

extern pid_t pid_x;
extern pid_t pid_y;
extern pid_t pid_r;

void control_update_setpoint_vx(float x);
void control_update_setpoint_vy(float y);
void control_update_setpoint_omega(float r);
void control_init(void);

#endif // CONTROL_H
