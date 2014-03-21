#include <stdint.h>
#include <stddef.h>
#include <ucos_ii.h>
#include <robot_base_mixer/robot_base_mixer.h>
#include <cvra_dc.h>
#include "tasks.h"
#include "_pid.h"

#include "control.h"

#define CONTROL_FREQ 500 // [Hz]

OS_STK control_task_stk[CONTROL_TASK_STACKSIZE];

static pid_t pid_x;
static pid_t pid_y;
static pid_t pid_r;
static float setpoint_speed_x;
static float setpoint_speed_y;
static float setpoint_omega;

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


void control_task(void *arg)
{
    float prev_enc[3] = {0, 0, 0};
    while (42) {
        float enc[3];
        float encdiff[3];
        float y_x, y_y, y_r;
        float u_x, u_y, u_r;
        float wheel_cmd[3];
        enc[0] = cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE); // TODO
        enc[0] = cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE);
        enc[0] = cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE);
        encdiff[0] = enc[0] - prev_enc[0];
        encdiff[1] = enc[1] - prev_enc[1];
        encdiff[2] = enc[2] - prev_enc[2];
        holonomic_base_mixer_wheels_to_robot(encdiff, &y_x, &y_y, &y_r);
        u_x = pid_control(&pid_x, y_x - setpoint_speed_x);
        u_y = pid_control(&pid_y, y_y - setpoint_speed_y);
        u_r = pid_control(&pid_r, y_r - setpoint_omega);
        holonomic_base_mixer_robot_to_wheels(u_x, u_y, u_r, wheel_cmd);
        cvra_dc_set_pwm0(HEXMOTORCONTROLLER_BASE, wheel_cmd[0]*DC_PWM_MAX_VALUE);
        cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, wheel_cmd[1]*DC_PWM_MAX_VALUE);
        cvra_dc_set_pwm2(HEXMOTORCONTROLLER_BASE, wheel_cmd[2]*DC_PWM_MAX_VALUE);
        prev_enc[0] = enc[0];
        prev_enc[1] = enc[1];
        prev_enc[2] = enc[2];
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
    OSTaskCreateExt(control_task,
                    NULL,
                    &control_task_stk[CONTROL_TASK_STACKSIZE-1],
                    CONTROL_TASK_PRIORITY,
                    CONTROL_TASK_PRIORITY,
                    &control_task_stk[0],
                    CONTROL_TASK_STACKSIZE,
                    NULL, NULL);
}
