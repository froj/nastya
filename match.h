#ifndef MATCH_H
#define MATCH_H

#include <pid.h>

extern struct pid_filter pos_x_pid;
extern struct pid_filter pos_y_pid;
extern struct pid_filter theta_pid;

void ready_for_match(void);

#endif // MATCH_H
