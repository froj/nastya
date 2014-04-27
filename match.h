#ifndef MATCH_H
#define MATCH_H

#include <pid.h>

extern struct pid_filter pos_x_pid;
extern struct pid_filter pos_y_pid;
extern struct pid_filter theta_pid;

void position_control_init();
void ready_for_match(void);
int goto_position(float dest_x, float dest_y, float lookat_x, float lookat_y);
void match_set_disable_position_control(bool dis_pos_ctl);
void match_set_red(void);
void match_set_yellow(void);

#endif // MATCH_H
