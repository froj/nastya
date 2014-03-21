#include "pid.h"

void pid_init(struct pid *pid, float freq)
{
	pid->kp = 0.1;
	pid->ki = 0;
	pid->kd = 0;
	pid->integration_bound = 0;
	pid->delta_t = 1/freq;
	pid_reset(pid);
}

void pid_reset(struct pid *pid)
{
	pid->error_integral = 0;
	pid->prev_error = 0;
}

float pid_control(struct pid *pid, float error)
{
	float p = error;
	pid->error_integral += error * pid->delta_t;
	if (pid->integration_bound != 0) {
		if (pid->error_integral > pid->integration_bound)
			pid->error_integral = pid->integration_bound;
		else if (pid->error_integral < -pid->integration_bound)
			pid->error_integral = -pid->integration_bound;
	}
	float i = pid->error_integral;
	float d = 0;
	d = (error - pid->prev_error) / pid->delta_t;
	pid->prev_error = error;
	return -1*(pid->kp*p + pid->ki*i + pid->kd*d);
}
