#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <ucos_ii.h>
#include <robot_base_mixer/robot_base_mixer.h>
#include "tasks.h"
#include "hardware.h"


#include "control.h"

#include "plot_task.h"

#define CONTROL_FREQ 100 // [Hz]

#define VX_CS_SCALE     1024
#define VY_CS_SCALE     1024
#define OMEGA_CS_SCALE  1024

OS_STK control_task_stk[CONTROL_TASK_STACKSIZE];

struct holonomic_base_speed_cs nastya_cs;



void control_update_setpoint_vx(float vx)
{
    cs_set_consign(&nastya_cs.vx_cs, VX_CS_SCALE * vx);
}

void control_update_setpoint_vy(float vy)
{
    cs_set_consign(&nastya_cs.vy_cs, VY_CS_SCALE * vy);
}

void control_update_setpoint_omega(float omega)
{
    cs_set_consign(&nastya_cs.omega_cs, OMEGA_CS_SCALE * omega);
}


void control_task(void *arg)
{
    float prev_enc[3] = {
        hw_get_wheel_0_encoder(),
        hw_get_wheel_1_encoder(),
        hw_get_wheel_2_encoder()
    };
    timestamp_t prev_timest = uptime_get();
    while (42) {
        OSTimeDly(OS_TICKS_PER_SEC / CONTROL_FREQ);
        float enc[3];
        float encdiff[3];
        float wheel_cmd[3];
        timestamp_t now = uptime_get();
        enc[0] = hw_get_wheel_0_encoder();
        enc[1] = hw_get_wheel_1_encoder();
        enc[2] = hw_get_wheel_2_encoder();
        int32_t delta_t = now - prev_timest;
        prev_timest = now;
        encdiff[0] = (float)(enc[0] - prev_enc[0])
                            / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION
                            * 2*M_PI / delta_t * 1000000;
        encdiff[1] = (float)(enc[1] - prev_enc[1])
                            / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION
                            * 2*M_PI / delta_t * 1000000;
        encdiff[2] = (float)(enc[2] - prev_enc[2])
                            / HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION
                            * 2*M_PI / delta_t * 1000000;
        prev_enc[0] = enc[0];
        prev_enc[1] = enc[1];
        prev_enc[2] = enc[2];

        holonomic_base_mixer_wheels_to_robot(encdiff, &nastya_cs.vx,
                                             &nastya_cs.vy, &nastya_cs.omega);

        cs_manage(&nastya_cs.vx_cs);
        cs_manage(&nastya_cs.vy_cs);
        cs_manage(&nastya_cs.omega_cs);

        holonomic_base_mixer_robot_to_wheels(nastya_cs.out_x, nastya_cs.out_y,
                                             nastya_cs.out_rotation, wheel_cmd);

        hw_set_wheel_0_motor_pwm(wheel_cmd[0]);
        hw_set_wheel_1_motor_pwm(wheel_cmd[1]);
        hw_set_wheel_2_motor_pwm(wheel_cmd[2]);
    }
}



static void holonomic_base_cs_output(void *arg, int32_t out)
{
    (int32_t*)arg = out;
}

static int32_t holonomic_base_cs_vx_input(void *arg)
{
    return nastya_cs.vx * VX_CS_SCALE;
}

static int32_t holonomic_base_cs_vx_input(void *arg)
{
    return nastya_cs.vy * VY_CS_SCALE;
}

static int32_t holonomic_base_cs_vx_input(void *arg)
{
    return nastya_cs.omega * OMEGA_CS_SCALE;
}


void holonomic_base_speed_cs_init(void)
{
    // velocity in x
    pid_init(&nastya_cs.vx_pid);
    pid_set_gains(&nastya_cs.vx_pid, 0, 0, 0); // KP, KI, KD
    pid_set_maximums(&nastya_cs.vx_pid, 0, 5000, 30000); // in , integral, out
    pid_set_out_shift(&nastya_cs.vx_pid, 10);
    // velocity in y
    pid_init(&nastya_cs.vy_pid);
    pid_set_gains(&nastya_cs.vy_pid, 0, 0, 0); // KP, KI, KD
    pid_set_maximums(&nastya_cs.vy_pid, 0, 5000, 30000); // in , integral, out
    pid_set_out_shift(&nastya_cs.vy_pid, 10);
    // angular velocity omega
    pid_init(&nastya_cs.omega_pid);
    pid_set_gains(&nastya_cs.omega_pid, 0, 0, 0); // KP, KI, KD
    pid_set_maximums(&nastya_cs.omega_pid, 0, 5000, 30000); // in , integral, out
    pid_set_out_shift(&nastya_cs.omega_pid, 10);

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
    // plot_add_variable("0:", &pid_error_speed_x, PLOT_FLOAT);
    // plot_add_variable("1:", &pid_error_speed_y, PLOT_FLOAT);
    // plot_add_variable("2:", &pid_error_omega, PLOT_FLOAT);
    // plot_add_variable("3:", &u_x, PLOT_FLOAT);
    // //plot_add_variable("4:", &u_y, PLOT_FLOAT);
    // //plot_add_variable("5:", &u_r, PLOT_FLOAT);
    // plot_add_variable("4:", &wheel_cmd[1], PLOT_FLOAT);

    OSTaskCreateExt(control_task,
                    NULL,
                    &control_task_stk[CONTROL_TASK_STACKSIZE-1],
                    CONTROL_TASK_PRIORITY,
                    CONTROL_TASK_PRIORITY,
                    &control_task_stk[0],
                    CONTROL_TASK_STACKSIZE,
                    NULL, NULL);
}
