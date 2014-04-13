#include <stdio.h>
#include <stdint.h>
#include <robot_base_mixer/robot_base_mixer.h>
#include <math.h>
#include <ucos_ii.h>
#include <uptime.h>
#include "hardware.h"

#include "tasks.h"
#include "plot_task.h"

#include "position_integration.h"


#define POSITON_INTEGRATION_FREQ 1000 // [Hz]

OS_STK position_integration_stk[POSITION_INTEGRATION_TASK_STACKSIZE];

static double pos_x;
static double pos_y;
static double theta;
static double vel_x;
static double vel_y;
static double omega;

void get_position(float *x, float *y)
{
    *x = pos_x;
    *y = pos_y;
}

float get_heading(void)
{
    return theta;
}

void get_velocity(float *x, float *y)
{
    *x = vel_x;
    *y = vel_y;
}

float get_omega(void)
{
    return omega;
}

void position_integration_task(void *pdata)
{
    printf("position integration task started\n");
    timestamp_t last_iteration = uptime_get();
    static int32_t prev_enc[3];
    prev_enc[0] = hw_get_wheel_0_encoder();
    prev_enc[1] = hw_get_wheel_1_encoder();
    prev_enc[2] = hw_get_wheel_2_encoder();
    while (1) {
        OSTimeDly(OS_TICKS_PER_SEC/POSITON_INTEGRATION_FREQ);
        static int32_t enc[3], delta_enc[3];
        static float delta_wheel[3];
        timestamp_t now = uptime_get();
        enc[0] = hw_get_wheel_0_encoder();
        enc[1] = hw_get_wheel_1_encoder();
        enc[2] = hw_get_wheel_2_encoder();
        delta_enc[0] = enc[0] - prev_enc[0];
        delta_enc[1] = enc[1] - prev_enc[1];
        delta_enc[2] = enc[2] - prev_enc[2];
        prev_enc[0] = enc[0];
        prev_enc[1] = enc[1];
        prev_enc[2] = enc[2];
        delta_wheel[0] = (float) delta_enc[0] * 2 * M_PI
                         / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION;
        delta_wheel[1] = (float) delta_enc[1] * 2 * M_PI
                         / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION;
        delta_wheel[2] = (float) delta_enc[2] * 2 * M_PI
                         / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION;
        float dx, dy, dtheta;
        holonomic_base_mixer_wheels_to_robot(delta_wheel, &dx, &dy, &dtheta);
        int32_t delta_t = now - last_iteration;
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);
        pos_x += dx * cos_theta - dy * sin_theta;
        pos_y += dx * sin_theta + dy * cos_theta;
        theta += dtheta;
        vel_x = (dx * cos_theta - dy * sin_theta) / delta_t;
        vel_y = (dx * sin_theta + dy * cos_theta) / delta_t;
        omega = dtheta / delta_t;
        last_iteration = now;
    }
}

void position_reset(void)
{
    pos_x = 0;
    pos_y = 0;
    theta = 0;
    vel_x = 0;
    vel_y = 0;
    omega = 0;
}

void position_reset_to(float x, float y, float t)
{
    pos_x = x;
    pos_y = y;
    theta = t;
    vel_x = 0;
    vel_y = 0;
    omega = 0;
}

void start_position_integration(void)
{
    position_reset();
    OSTaskCreateExt(position_integration_task,
                    NULL,
                    &position_integration_stk[POSITION_INTEGRATION_TASK_STACKSIZE-1],
                    POSITION_INTEGRATION_TASK_PRIORITY,
                    POSITION_INTEGRATION_TASK_PRIORITY,
                    &position_integration_stk[0],
                    POSITION_INTEGRATION_TASK_STACKSIZE,
                    NULL, 0);
}
