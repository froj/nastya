#ifndef PID_H
#define PID_H

struct pid {
	// configuration
	float kp;
	float ki;
	float kd;
	float delta_t;
	float integration_bound;
	// state
	float error_integral;
	float prev_error;
};


void pid_init(struct pid *pid, float freq);
void pid_reset(struct pid *pid);
float pid_control(struct pid *pid, float error);

#endif
