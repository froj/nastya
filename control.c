#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <ucos_ii.h>
#include <robot_base_mixer/robot_base_mixer.h>
#include <trace/trace.h>
#include "uptime/uptime.h"
#include "cvra_param_robot.h"
#include "tasks.h"
#include "hardware.h"


#include "control.h"

#define CONTROL_FREQ 100 // [Hz]

// PID speed control integer range calculations:
// max input speed 4 m/s, resp 4 rad/s
// with scaling 2048 (VX_IN_SCALE, VY_IN_SCALE, OMEGA_IN_SCALE)
// 4 * 2048 = 8192
// maximal error is 2 * 8 * 1024 = 16384
// with maximal KP = 1024, KI = 40, KD = 1024, MAX_I <= 3000
// max command = 16384 * 1024 + 16384 * 40 * 3000 + 16384 * 2 * 1024
// = 2016411648 < 2^31 = 2147483648

#define VX_IN_SCALE     2048
#define VY_IN_SCALE     2048
#define OMEGA_IN_SCALE  2048

#define VX_MAX_ERR_INPUT    4096
#define VY_MAX_ERR_INPUT    4096
#define OMEGA_MAX_ERR_INPUT 4096

// The output is scaled to allow KP, KI & KD values close to
// maxima (as calculated above) for optimal resolution.
// (The scaling is done after the conversion to floating point numbers and not
//  with the pid-builtin bitshift. This is to avoid losing resolution
//  because the transform from robot coordinates to wheels introduces
//  another scaling factor before the command is output to PWM motor control)
#define VX_OUT_SCALE    4096
#define VY_OUT_SCALE    4096
#define OMEGA_OUT_SCALE 4096



OS_STK control_task_stk[CONTROL_TASK_STACKSIZE];

struct holonomic_base_speed_cs nastya_cs;

#define SPEED_CTRL_FREQ_DEFAULT 20 // [Hz]
static param_t speed_ctrl_freq;

void control_update_setpoint_vx(float vx)
{
    cs_set_consign(&nastya_cs.vx_cs, VX_IN_SCALE * vx);
}

void control_update_setpoint_vy(float vy)
{
    cs_set_consign(&nastya_cs.vy_cs, VY_IN_SCALE * vy);
}

void control_update_setpoint_omega(float omega)
{
    cs_set_consign(&nastya_cs.omega_cs, OMEGA_IN_SCALE * omega);
}

float control_get_setpoint_vx(void)
{
    return (float)cs_get_consign(&nastya_cs.vx_cs) / VX_IN_SCALE;
}

float control_get_setpoint_vy(void)
{
    return (float)cs_get_consign(&nastya_cs.vy_cs) / VY_IN_SCALE;
}

float control_get_setpoint_omega(void)
{
    return (float)cs_get_consign(&nastya_cs.omega_cs) / OMEGA_IN_SCALE;
}

static void update_parameters(void)
{
    // pid xy combined
    if (param_has_changed(&nastya_cs.vxy_pid_P)) {
        param_set(&nastya_cs.vx_pid_P, param_get(&nastya_cs.vxy_pid_P));
        param_set(&nastya_cs.vy_pid_P, param_get(&nastya_cs.vxy_pid_P));
    }
    if (param_has_changed(&nastya_cs.vxy_pid_I)) {
        param_set(&nastya_cs.vx_pid_I, param_get(&nastya_cs.vxy_pid_I));
        param_set(&nastya_cs.vy_pid_I, param_get(&nastya_cs.vxy_pid_I));
    }
    if (param_has_changed(&nastya_cs.vxy_pid_D)) {
        param_set(&nastya_cs.vx_pid_D, param_get(&nastya_cs.vxy_pid_D));
        param_set(&nastya_cs.vy_pid_D, param_get(&nastya_cs.vxy_pid_D));
    }
    if (param_has_changed(&nastya_cs.vxy_pid_D_filt)) {
        param_set(&nastya_cs.vx_pid_D_filt, param_get(&nastya_cs.vxy_pid_D_filt));
        param_set(&nastya_cs.vy_pid_D_filt, param_get(&nastya_cs.vxy_pid_D_filt));
    }
    if (param_has_changed(&nastya_cs.vxy_pid_I_bound)) {
        param_set(&nastya_cs.vx_pid_I_bound, param_get(&nastya_cs.vxy_pid_I_bound));
        param_set(&nastya_cs.vy_pid_I_bound, param_get(&nastya_cs.vxy_pid_I_bound));
    }
    // pid vx
    if (param_has_changed(&nastya_cs.vx_pid_P)
        || param_has_changed(&nastya_cs.vx_pid_I)
        || param_has_changed(&nastya_cs.vx_pid_D)) {
        pid_set_gains(&nastya_cs.vx_pid,
                      param_get(&nastya_cs.vx_pid_P),
                      param_get(&nastya_cs.vx_pid_I),
                      param_get(&nastya_cs.vx_pid_D));
    }
    if (param_has_changed(&nastya_cs.vx_pid_I_bound)) {
        pid_set_maximums(&nastya_cs.vx_pid, VX_MAX_ERR_INPUT,
                         param_get(&nastya_cs.vx_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&nastya_cs.vx_pid_D_filt)) {
        pid_set_derivate_filter(&nastya_cs.vx_pid,
                                param_get(&nastya_cs.vx_pid_D_filt));
    }
    // pid vy
    if (param_has_changed(&nastya_cs.vy_pid_P)
        || param_has_changed(&nastya_cs.vy_pid_I)
        || param_has_changed(&nastya_cs.vy_pid_D)) {
        pid_set_gains(&nastya_cs.vy_pid,
                      param_get(&nastya_cs.vy_pid_P),
                      param_get(&nastya_cs.vy_pid_I),
                      param_get(&nastya_cs.vy_pid_D));
    }
    if (param_has_changed(&nastya_cs.vy_pid_I_bound)) {
        pid_set_maximums(&nastya_cs.vy_pid, VY_MAX_ERR_INPUT,
                         param_get(&nastya_cs.vy_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&nastya_cs.vy_pid_D_filt)) {
        pid_set_derivate_filter(&nastya_cs.vy_pid,
                                param_get(&nastya_cs.vy_pid_D_filt));
    }
    // pid omega
    if (param_has_changed(&nastya_cs.omega_pid_P)
        || param_has_changed(&nastya_cs.omega_pid_I)
        || param_has_changed(&nastya_cs.omega_pid_D)) {
        pid_set_gains(&nastya_cs.omega_pid,
                      param_get(&nastya_cs.omega_pid_P),
                      param_get(&nastya_cs.omega_pid_I),
                      param_get(&nastya_cs.omega_pid_D));
    }
    if (param_has_changed(&nastya_cs.omega_pid_I_bound)) {
        pid_set_maximums(&nastya_cs.omega_pid, OMEGA_MAX_ERR_INPUT,
                         param_get(&nastya_cs.omega_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&nastya_cs.omega_pid_D_filt)) {
        pid_set_derivate_filter(&nastya_cs.omega_pid,
                                param_get(&nastya_cs.omega_pid_D_filt));
    }
}

void control_task(void *arg)
{
    static trace_var_t t_err_vx;
    static trace_var_t t_err_vy;
    static trace_var_t t_err_omega;
    static trace_var_t t_out_vx;
    static trace_var_t t_out_vy;
    static trace_var_t t_out_omega;
    trace_var_add(&t_err_vx, "cs_vx_err");
    trace_var_add(&t_err_vy, "cs_vy_err");
    trace_var_add(&t_err_omega, "cs_omega_err");
    trace_var_add(&t_out_vx, "cs_vx_out");
    trace_var_add(&t_out_vy, "cs_vy_out");
    trace_var_add(&t_out_omega, "cs_omega_out");

    int32_t prev_enc[3] = {
        hw_get_wheel_0_encoder(),
        hw_get_wheel_1_encoder(),
        hw_get_wheel_2_encoder()
    };
    timestamp_t prev_timest = uptime_get();
    int period_us = 1;
    while (42) {
        if (param_has_changed(&speed_ctrl_freq)) {
            period_us = OS_TICKS_PER_SEC / param_get(&speed_ctrl_freq);
        }
        OSTimeDly(period_us);
        update_parameters();
        int32_t enc[3];
        float wheel_ang_vel[3]; // [rad/s]
        float wheel_cmd[3];     // motor PWM (arbitrary unit)
        timestamp_t now = uptime_get();
        enc[0] = hw_get_wheel_0_encoder();
        enc[1] = hw_get_wheel_1_encoder();
        enc[2] = hw_get_wheel_2_encoder();
        float delta_t = (float)(now - prev_timest) / 1000000; // [s]
        prev_timest = now;
        wheel_ang_vel[0] = (float)(enc[0] - prev_enc[0]) / delta_t
                            / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION * 2*M_PI;
        wheel_ang_vel[1] = (float)(enc[1] - prev_enc[1]) / delta_t
                            / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION * 2*M_PI;
        wheel_ang_vel[2] = (float)(enc[2] - prev_enc[2]) / delta_t
                            / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION * 2*M_PI;
        prev_enc[0] = enc[0];
        prev_enc[1] = enc[1];
        prev_enc[2] = enc[2];

        holonomic_base_mixer_wheels_to_robot(wheel_ang_vel, &nastya_cs.vx,
                                             &nastya_cs.vy, &nastya_cs.omega);

        nastya_cs.out_x = 0;
        nastya_cs.out_y = 0;
        nastya_cs.out_rotation = 0;
        if (nastya_cs.vx_control_enable) {
            cs_manage(&nastya_cs.vx_cs);
        } else {
            pid_reset(&nastya_cs.vx_pid);
        }
        if (nastya_cs.vy_control_enable) {
            cs_manage(&nastya_cs.vy_cs);
        } else {
            pid_reset(&nastya_cs.vy_pid);
        }
        if (nastya_cs.omega_control_enable) {
            cs_manage(&nastya_cs.omega_cs);
        } else {
            pid_reset(&nastya_cs.omega_pid);
        }
        float cmd_x = (float)nastya_cs.out_x / VX_OUT_SCALE;
        float cmd_y = (float)nastya_cs.out_y / VY_OUT_SCALE;
        float cmd_rot = (float)nastya_cs.out_rotation / OMEGA_OUT_SCALE;

        holonomic_base_mixer_robot_to_wheels(cmd_x, cmd_y, cmd_rot, wheel_cmd);

        hw_set_wheel_0_motor_pwm(wheel_cmd[0]);
        hw_set_wheel_1_motor_pwm(wheel_cmd[1]);
        hw_set_wheel_2_motor_pwm(wheel_cmd[2]);

        trace_var_update(&t_err_vx, nastya_cs.vx - (float)cs_get_consign(&nastya_cs.vx_cs) / VX_IN_SCALE);
        trace_var_update(&t_err_vy, nastya_cs.vy - (float)cs_get_consign(&nastya_cs.vy_cs) / VY_IN_SCALE);
        trace_var_update(&t_err_omega, nastya_cs.omega - (float)cs_get_consign(&nastya_cs.omega_cs) / OMEGA_IN_SCALE);
        trace_var_update(&t_out_vx, cmd_x);
        trace_var_update(&t_out_vy, cmd_y);
        trace_var_update(&t_out_omega, cmd_rot);
    }
}


// the following static functions are for control system input & output
static void holonomic_base_cs_output(void *arg, int32_t out)
{
    *(int32_t*)arg = out;
}

static int32_t holonomic_base_cs_vx_input(void *arg)
{
    return nastya_cs.vx * VX_IN_SCALE;
}

static int32_t holonomic_base_cs_vy_input(void *arg)
{
    return nastya_cs.vy * VY_IN_SCALE;
}

static int32_t holonomic_base_cs_omega_input(void *arg)
{
    return nastya_cs.omega * OMEGA_IN_SCALE;
}


void holonomic_base_speed_cs_init(void)
{
    param_add(&nastya_cs.vxy_pid_P, "pid_vxy_P", NULL);
    param_add(&nastya_cs.vxy_pid_I, "pid_vxy_I", NULL);
    param_add(&nastya_cs.vxy_pid_D, "pid_vxy_D", NULL);
    param_add(&nastya_cs.vxy_pid_D_filt, "pid_vxy_D_filt", NULL);
    param_add(&nastya_cs.vxy_pid_I_bound, "pid_vxy_I_bound", NULL);
    param_add(&nastya_cs.vx_pid_P, "pid_vx_P", NULL);
    param_add(&nastya_cs.vx_pid_I, "pid_vx_I", NULL);
    param_add(&nastya_cs.vx_pid_D, "pid_vx_D", NULL);
    param_add(&nastya_cs.vx_pid_D_filt, "pid_vx_D_filt", NULL);
    param_add(&nastya_cs.vx_pid_I_bound, "pid_vx_I_bound", NULL);
    param_add(&nastya_cs.vy_pid_P, "pid_vy_P", NULL);
    param_add(&nastya_cs.vy_pid_I, "pid_vy_I", NULL);
    param_add(&nastya_cs.vy_pid_D, "pid_vy_D", NULL);
    param_add(&nastya_cs.vy_pid_D_filt, "pid_vy_D_filt", NULL);
    param_add(&nastya_cs.vy_pid_I_bound, "pid_vy_I_bound", NULL);
    param_add(&nastya_cs.omega_pid_P, "pid_omega_P", NULL);
    param_add(&nastya_cs.omega_pid_I, "pid_omega_I", NULL);
    param_add(&nastya_cs.omega_pid_D, "pid_omega_D", NULL);
    param_add(&nastya_cs.omega_pid_D_filt, "pid_omega_D_filt", NULL);
    param_add(&nastya_cs.omega_pid_I_bound, "pid_omega_I_bound", NULL);

    // velocity xy
    param_set(&nastya_cs.vxy_pid_P, 20);
    param_set(&nastya_cs.vxy_pid_I, 30);
    param_set(&nastya_cs.vxy_pid_D, 4);
    param_set(&nastya_cs.vxy_pid_D_filt, 15);
    param_set(&nastya_cs.vxy_pid_I_bound, 3000);
    // omega
    param_set(&nastya_cs.omega_pid_P, 30);
    param_set(&nastya_cs.omega_pid_I, 30);
    param_set(&nastya_cs.omega_pid_D, 10);
    param_set(&nastya_cs.omega_pid_D_filt, 15);
    param_set(&nastya_cs.omega_pid_I_bound, 3000);

    pid_init(&nastya_cs.vx_pid);
    pid_set_out_shift(&nastya_cs.vx_pid, 0);
    pid_init(&nastya_cs.vy_pid);
    pid_set_out_shift(&nastya_cs.vy_pid, 0);
    pid_init(&nastya_cs.omega_pid);
    pid_set_out_shift(&nastya_cs.omega_pid, 0);

    cs_init(&nastya_cs.vx_cs);
    cs_init(&nastya_cs.vy_cs);
    cs_init(&nastya_cs.omega_cs);
    cs_set_correct_filter(&nastya_cs.vx_cs, pid_do_filter, &nastya_cs.vx_pid);
    cs_set_correct_filter(&nastya_cs.vy_cs, pid_do_filter, &nastya_cs.vy_pid);
    cs_set_correct_filter(&nastya_cs.omega_cs, pid_do_filter, &nastya_cs.omega_pid);
    cs_set_process_in(&nastya_cs.vx_cs, holonomic_base_cs_output, &nastya_cs.out_x);
    cs_set_process_in(&nastya_cs.vy_cs, holonomic_base_cs_output, &nastya_cs.out_y);
    cs_set_process_in(&nastya_cs.omega_cs, holonomic_base_cs_output, &nastya_cs.out_rotation);
    cs_set_process_out(&nastya_cs.vx_cs, holonomic_base_cs_vx_input, NULL);
    cs_set_process_out(&nastya_cs.vy_cs, holonomic_base_cs_vy_input, NULL);
    cs_set_process_out(&nastya_cs.omega_cs, holonomic_base_cs_omega_input, NULL);
    cs_set_consign(&nastya_cs.vx_cs, 0);
    cs_set_consign(&nastya_cs.vy_cs, 0);
    cs_set_consign(&nastya_cs.omega_cs, 0);
    nastya_cs.vx_control_enable = true;
    nastya_cs.vy_control_enable = true;
    nastya_cs.omega_control_enable = true;
}


void control_init(void)
{
    holonomic_base_speed_cs_init();
    param_add(&speed_ctrl_freq, "speed_ctrl_freq", "[Hz]");
    param_set(&speed_ctrl_freq, SPEED_CTRL_FREQ_DEFAULT);

    OSTaskCreateExt(control_task,
                    NULL,
                    &control_task_stk[CONTROL_TASK_STACKSIZE-1],
                    CONTROL_TASK_PRIORITY,
                    CONTROL_TASK_PRIORITY,
                    &control_task_stk[0],
                    CONTROL_TASK_STACKSIZE,
                    NULL, 0);
}
