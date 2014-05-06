#include <stdio.h>
#include <stdint.h>
#include <robot_base_mixer/robot_base_mixer.h>
#include <math.h>
#include <ucos_ii.h>
#include <uptime.h>
#include "hardware.h"

#include <trace.h>
#include "tasks.h"

#include "position_integration.h"


#define POSITON_INTEGRATION_FREQ 100 // [Hz]

#define LP_BUFFER_SIZE (POSITON_INTEGRATION_FREQ / 10)

OS_STK position_integration_stk[POSITION_INTEGRATION_TASK_STACKSIZE];

static double pos_x;
static double pos_y;
static double theta;
float vel_x;
float vel_y;
float omega;

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
    trace_var_t t_vx;
    trace_var_t t_vy;
    trace_var_t t_omega;
    trace_var_t t_x;
    trace_var_t t_y;
    trace_var_t t_theta;
    trace_var_add(&t_vx, "vx");
    trace_var_add(&t_vy, "vy");
    trace_var_add(&t_omega, "omega");
    trace_var_add(&t_x, "x");
    trace_var_add(&t_y, "y");
    trace_var_add(&t_theta, "theta");

    static float lp_buffer_x[LP_BUFFER_SIZE] = {0};
    static float lp_buffer_y[LP_BUFFER_SIZE] = {0};
    static float lp_buffer_omega[LP_BUFFER_SIZE] = {0};

    static float lp_acc_x = 0.0;
    static float lp_acc_y = 0.0;
    static float lp_acc_omega = 0.0;

    static int lp_buf_index_x = 0;
    static int lp_buf_index_y = 0;
    static int lp_buf_index_omega = 0;

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
        float delta_t = (float)(now - last_iteration) / 1000000;
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);
        pos_x += dx * cos_theta - dy * sin_theta;
        pos_y += dx * sin_theta + dy * cos_theta;
        theta += dtheta;

        float inst_vx, inst_vy, inst_omega;

        inst_vx = (dx * cos_theta - dy * sin_theta) / delta_t;
        inst_vy = (dx * sin_theta + dy * cos_theta) / delta_t;
        inst_omega = dtheta / delta_t;

        lp_acc_x += inst_vx - lp_buffer_x[lp_buf_index_x];
        lp_acc_y += inst_vy - lp_buffer_y[lp_buf_index_y];
        lp_acc_omega += inst_omega - lp_buffer_omega[lp_buf_index_omega];

        lp_buffer_x[lp_buf_index_x] = inst_vx;
        lp_buffer_y[lp_buf_index_y] = inst_vy;
        lp_buffer_omega[lp_buf_index_omega] = inst_omega;

        if (++lp_buf_index_x >= LP_BUFFER_SIZE) lp_buf_index_x = 0;
        if (++lp_buf_index_y >= LP_BUFFER_SIZE) lp_buf_index_y = 0;
        if (++lp_buf_index_omega >= LP_BUFFER_SIZE) lp_buf_index_omega = 0;

        vel_x = lp_acc_x / LP_BUFFER_SIZE;
        vel_y = lp_acc_y / LP_BUFFER_SIZE;
        omega = lp_acc_omega / LP_BUFFER_SIZE;

        trace_var_update(&t_vx, vel_x);
        trace_var_update(&t_vy, vel_y);
        trace_var_update(&t_omega, omega);
        trace_var_update(&t_x, pos_x);
        trace_var_update(&t_y, pos_y);
        trace_var_update(&t_theta, theta);

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
