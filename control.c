#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <ucos_ii.h>
#include <robot_base_mixer/robot_base_mixer.h>
#include <cvra_dc.h>
#include "tasks.h"
#include "cvra_param_robot.h"
#include "_pid.h"

#include "control.h"

#define CONTROL_FREQ 80 // [Hz]

OS_STK control_task_stk[CONTROL_TASK_STACKSIZE];

static pid_t pid_x;
static pid_t pid_y;
static pid_t pid_r;
static volatile float setpoint_speed_x;
static volatile float setpoint_speed_y;
static volatile float setpoint_omega;


float wheel_cmd[3];

void control_update_setpoint_vx(float x)
{
    setpoint_speed_x = x;
}

void control_update_setpoint_vy(float y)
{
    setpoint_speed_y = y;
}

void control_update_setpoint_omega(float r)
{
    setpoint_omega = r;
}


        float y_x, y_y, y_r;


void control_task(void *arg)
{
    float prev_enc[3] = {
        -cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE),
        -cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE), 
        -cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE)};
    while (42) {
        float enc[3];
        float encdiff[3];
//        float y_x, y_y, y_r;
        float u_x, u_y, u_r;
        //float wheel_cmd[3];
        enc[0] = -cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE);
        enc[1] = -cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE);
        enc[2] = -cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE);
        encdiff[0] = (enc[0] - prev_enc[0])/ROBOT_ENCODER_RESOLUTION*2*M_PI*CONTROL_FREQ;
        encdiff[1] = (enc[1] - prev_enc[1])/ROBOT_ENCODER_RESOLUTION*2*M_PI*CONTROL_FREQ;
        encdiff[2] = (enc[2] - prev_enc[2])/ROBOT_ENCODER_RESOLUTION*2*M_PI*CONTROL_FREQ;
        holonomic_base_mixer_wheels_to_robot(encdiff, &y_x, &y_y, &y_r);
        u_x = pid_control(&pid_x, y_x - setpoint_speed_x);
        u_y = pid_control(&pid_y, y_y - setpoint_speed_y);
        u_r = pid_control(&pid_r, y_r - setpoint_omega);
        holonomic_base_mixer_robot_to_wheels(u_x, u_y, u_r, wheel_cmd);
        cvra_dc_set_pwm0(HEXMOTORCONTROLLER_BASE, wheel_cmd[0]*DC_PWM_MAX_VALUE);
        cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, -wheel_cmd[1]*DC_PWM_MAX_VALUE);
        cvra_dc_set_pwm2(HEXMOTORCONTROLLER_BASE, -wheel_cmd[2]*DC_PWM_MAX_VALUE);
        prev_enc[0] = enc[0];
        prev_enc[1] = enc[1];
        prev_enc[2] = enc[2];
        //OSTimeDlyHMSM(0, 0, 0, 1000/CONTROL_FREQ);
        OSTimeDly(OS_TICKS_PER_SEC/CONTROL_FREQ);
    }
}


void control_init(void)
{
    setpoint_speed_x = 0;
    setpoint_speed_y = 0;
    setpoint_omega = 0;
    pid_init(&pid_x, CONTROL_FREQ);
    pid_init(&pid_y, CONTROL_FREQ);
    pid_init(&pid_r, CONTROL_FREQ);
    pid_x.kp = 0.07;
    pid_x.ki = 0.01;
    pid_x.kd = 0;
    pid_y.kp = 0.01;
    pid_y.ki = 0;
    pid_y.kd = 0;
    pid_r.kp = 0.01;
    pid_r.ki = 0;
    pid_r.kd = 0;
    OSTaskCreateExt(control_task,
                    NULL,
                    &control_task_stk[CONTROL_TASK_STACKSIZE-1],
                    CONTROL_TASK_PRIORITY,
                    CONTROL_TASK_PRIORITY,
                    &control_task_stk[0],
                    CONTROL_TASK_STACKSIZE,
                    NULL, NULL);
}
