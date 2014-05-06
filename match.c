#include <stdio.h>
#include <math.h>
#include <ucos_ii.h>
#include <control_system_manager/control_system_manager.h>
#include <pid/pid.h>
#include "tasks.h"
#include <uptime.h>
#include "control.h"
#include "position_integration.h"
#include "hardware.h"
#include "plot_task.h"

#include "match.h"
#include "param.h"



#define GOTO_POS_FREQ 20    // [Hz]

#define MAX_ACCELERATION_XY 1.5     //  [m/s/s]
#define MAX_ACCELERATION_OMG (M_PI * 6)     //  [rad/s/s]

#define MAX_SPEED_XY    0.5     // [m/s]
#define MAX_OMEGA       M_PI     // [rad/s]

#define X_MAX_ERR_INPUT 2.0 * 1024
#define Y_MAX_ERR_INPUT 2.0 * 1024
#define THETA_MAX_ERR_INPUT 0.3 *1024


// bool disable_postion_control;
bool team_red;

timestamp_t match_start;
bool match_has_startd = false;

static float limit_sym(float val, float max)
{
    if (val > max)
        return max;
    if (val < -max)
        return -max;
    return val;
}



static param_t pos_xy_pid_P;
static param_t pos_xy_pid_I;
static param_t pos_xy_pid_D;
static param_t pos_xy_pid_D_filt;
static param_t pos_xy_pid_I_bound;
static param_t pos_x_pid_P;
static param_t pos_x_pid_I;
static param_t pos_x_pid_D;
static param_t pos_x_pid_D_filt;
static param_t pos_x_pid_I_bound;
static param_t pos_y_pid_P;
static param_t pos_y_pid_I;
static param_t pos_y_pid_D;
static param_t pos_y_pid_D_filt;
static param_t pos_y_pid_I_bound;
static param_t theta_pid_P;
static param_t theta_pid_I;
static param_t theta_pid_D;
static param_t theta_pid_D_filt;
static param_t theta_pid_I_bound;
static struct cs pos_x_cs;
static struct cs pos_y_cs;
static struct cs theta_cs;
struct pid_filter pos_x_pid;
struct pid_filter pos_y_pid;
struct pid_filter theta_pid;
static int32_t out_x;
static int32_t out_y;
static int32_t out_rotation;
static int32_t in_x;
static int32_t in_y;
static int32_t in_rotation;

float circular_range(float ang)
{
    while (ang > M_PI)
        ang -= 2*M_PI;
    while (ang <= -M_PI)
        ang += 2*M_PI;
    return ang;
}

static void cs_out(void *arg, int32_t out)
{
    *(int32_t*)arg = out;
}

static int32_t cs_in(void *arg)
{
    return *(int32_t*)arg;
}

static float goto_stop_thershold;
static param_t goto_stop_thershold_param;

void position_control_init()
{
    cvra_beacon_init(&beacon, AVOIDING_BASE, AVOIDING_IRQ, 100, 1., 1.);

    param_add(&pos_xy_pid_P, "pid_pos_xy_P", NULL);
    param_add(&pos_xy_pid_I, "pid_pos_xy_I", NULL);
    param_add(&pos_xy_pid_D, "pid_pos_xy_D", NULL);
    param_add(&pos_xy_pid_D_filt, "pid_pos_xy_D_filt", NULL);
    param_add(&pos_xy_pid_I_bound, "pid_pos_xy_I_bound", NULL);
    param_add(&pos_x_pid_P, "pid_pos_x_P", NULL);
    param_add(&pos_x_pid_I, "pid_pos_x_I", NULL);
    param_add(&pos_x_pid_D, "pid_pos_x_D", NULL);
    param_add(&pos_x_pid_D_filt, "pid_pos_x_D_filt", NULL);
    param_add(&pos_x_pid_I_bound, "pid_pos_x_I_bound", NULL);
    param_add(&pos_y_pid_P, "pid_pos_y_P", NULL);
    param_add(&pos_y_pid_I, "pid_pos_y_I", NULL);
    param_add(&pos_y_pid_D, "pid_pos_y_D", NULL);
    param_add(&pos_y_pid_D_filt, "pid_pos_y_D_filt", NULL);
    param_add(&pos_y_pid_I_bound, "pid_pos_y_I_bound", NULL);
    param_add(&theta_pid_P, "pid_theta_P", NULL);
    param_add(&theta_pid_I, "pid_theta_I", NULL);
    param_add(&theta_pid_D, "pid_theta_D", NULL);
    param_add(&theta_pid_D_filt, "pid_theta_D_filt", NULL);
    param_add(&theta_pid_I_bound, "pid_theta_I_bound", NULL);

    // pos xy
    param_set(&pos_xy_pid_P, 100);
    param_set(&pos_xy_pid_I, 0);
    param_set(&pos_xy_pid_D, 3000);
    param_set(&pos_xy_pid_D_filt, 3);
    param_set(&pos_xy_pid_I_bound, 800);
    // theta
    param_set(&theta_pid_P, 4);
    param_set(&theta_pid_I, 10);
    param_set(&theta_pid_D, 2500);
    param_set(&theta_pid_D_filt, 3);
    param_set(&theta_pid_I_bound, 400);

    pid_init(&pos_x_pid);
    pid_set_out_shift(&pos_x_pid, 0);
    pid_init(&pos_y_pid);
    pid_set_out_shift(&pos_y_pid, 0);
    pid_init(&theta_pid);
    pid_set_out_shift(&theta_pid, 0);

    cs_init(&pos_x_cs);
    cs_init(&pos_y_cs);
    cs_init(&theta_cs);
    cs_set_correct_filter(&pos_x_cs, pid_do_filter, &pos_x_pid);
    cs_set_correct_filter(&pos_y_cs, pid_do_filter, &pos_y_pid);
    cs_set_correct_filter(&theta_cs, pid_do_filter, &theta_pid);
    cs_set_process_in(&pos_x_cs, cs_out, &out_x);
    cs_set_process_in(&pos_y_cs, cs_out, &out_y);
    cs_set_process_in(&theta_cs, cs_out, &out_rotation);
    cs_set_process_out(&pos_x_cs, cs_in, &in_x);
    cs_set_process_out(&pos_y_cs, cs_in, &in_y);
    cs_set_process_out(&theta_cs, cs_in, &in_rotation);
    cs_set_consign(&pos_x_cs, 0);
    cs_set_consign(&pos_y_cs, 0);
    cs_set_consign(&theta_cs, 0);

    param_add(&goto_stop_thershold_param, "goto_stop", NULL);
    param_set(&goto_stop_thershold_param, 0.0032);

    plot_add_variable("6: ", &in_x, PLOT_INT32);
    plot_add_variable("7: ", &in_y, PLOT_INT32);
    plot_add_variable("8: ", &in_rotation, PLOT_INT32);
    plot_add_variable("9: ", &out_x, PLOT_INT32);
    plot_add_variable("10: ", &out_y, PLOT_INT32);
    plot_add_variable("11: ", &out_rotation, PLOT_INT32);
}

static void update_parameters(void)
{
    // pid xy combined
    if (param_has_changed(&pos_xy_pid_P)) {
        param_set(&pos_x_pid_P, param_get(&pos_xy_pid_P));
        param_set(&pos_y_pid_P, param_get(&pos_xy_pid_P));
    }
    if (param_has_changed(&pos_xy_pid_I)) {
        param_set(&pos_x_pid_I, param_get(&pos_xy_pid_I));
        param_set(&pos_y_pid_I, param_get(&pos_xy_pid_I));
    }
    if (param_has_changed(&pos_xy_pid_D)) {
        param_set(&pos_x_pid_D, param_get(&pos_xy_pid_D));
        param_set(&pos_y_pid_D, param_get(&pos_xy_pid_D));
    }
    if (param_has_changed(&pos_xy_pid_D_filt)) {
        param_set(&pos_x_pid_D_filt, param_get(&pos_xy_pid_D_filt));
        param_set(&pos_y_pid_D_filt, param_get(&pos_xy_pid_D_filt));
    }
    if (param_has_changed(&pos_xy_pid_I_bound)) {
        param_set(&pos_x_pid_I_bound, param_get(&pos_xy_pid_I_bound));
        param_set(&pos_y_pid_I_bound, param_get(&pos_xy_pid_I_bound));
    }
    // pid x
    if (param_has_changed(&pos_x_pid_P)
        || param_has_changed(&pos_x_pid_I)
        || param_has_changed(&pos_x_pid_D)) {
        pid_set_gains(&pos_x_pid,
                      param_get(&pos_x_pid_P),
                      param_get(&pos_x_pid_I),
                      param_get(&pos_x_pid_D));
    }
    if (param_has_changed(&pos_x_pid_I_bound)) {
        pid_set_maximums(&pos_x_pid, X_MAX_ERR_INPUT,
                         param_get(&pos_x_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&pos_x_pid_D_filt)) {
        pid_set_derivate_filter(&pos_x_pid,
                                param_get(&pos_x_pid_D_filt));
    }
    // pid y
    if (param_has_changed(&pos_y_pid_P)
        || param_has_changed(&pos_y_pid_I)
        || param_has_changed(&pos_y_pid_D)) {
        pid_set_gains(&pos_y_pid,
                      param_get(&pos_y_pid_P),
                      param_get(&pos_y_pid_I),
                      param_get(&pos_y_pid_D));
    }
    if (param_has_changed(&pos_y_pid_I_bound)) {
        pid_set_maximums(&pos_y_pid, Y_MAX_ERR_INPUT,
                         param_get(&pos_y_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&pos_y_pid_D_filt)) {
        pid_set_derivate_filter(&pos_y_pid,
                                param_get(&pos_y_pid_D_filt));
    }
    // pid theta
    if (param_has_changed(&theta_pid_P)
        || param_has_changed(&theta_pid_I)
        || param_has_changed(&theta_pid_D)) {
        pid_set_gains(&theta_pid,
                      param_get(&theta_pid_P),
                      param_get(&theta_pid_I),
                      param_get(&theta_pid_D));
    }
    if (param_has_changed(&theta_pid_I_bound)) {
        pid_set_maximums(&theta_pid, THETA_MAX_ERR_INPUT,
                         param_get(&theta_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&theta_pid_D_filt)) {
        pid_set_derivate_filter(&theta_pid,
                                param_get(&theta_pid_D_filt));
    }
}

int goto_position(float dest_x, float dest_y, float lookat_x, float lookat_y)
{
    while (1) {
        OSTimeDly(OS_TICKS_PER_SEC / GOTO_POS_FREQ);
        update_parameters();
        // if (disable_postion_control)
        //     continue;
        float pos_x, pos_y, heading;
        get_position(&pos_x, &pos_y);
        heading = get_heading();
        float set_heading = atan2(lookat_y - pos_y, lookat_x - pos_x);
        float heading_err = circular_range(heading - set_heading);
        float x_err = pos_x - dest_x;
        float y_err = pos_y - dest_y;
        float current_speed_x, current_speed_y;
        float current_omega = get_omega();
        get_velocity(&current_speed_x, &current_speed_y);

        if (param_has_changed(&goto_stop_thershold)){
            goto_stop_thershold = param_get(&goto_stop_thershold);
        }

        if (x_err*x_err + y_err*y_err + heading_err*heading_err + current_speed_x*current_speed_x + current_speed_y*current_speed_y + current_omega*current_omega < goto_stop_thershold)
            return 0;
        in_x = x_err * 1024;
        in_y = y_err * 1024;
        in_rotation = heading_err * 1024;
        cs_manage(&pos_x_cs);
        cs_manage(&pos_y_cs);
        cs_manage(&theta_cs);
        float set_speed_x = limit_sym(current_speed_x + limit_sym((float)out_x / 8192 / 16, MAX_ACCELERATION_XY / GOTO_POS_FREQ), MAX_SPEED_XY);
        float set_speed_y = limit_sym(current_speed_y + limit_sym((float)out_y / 8192 / 16, MAX_ACCELERATION_XY / GOTO_POS_FREQ), MAX_SPEED_XY);
        printf("setspeed x: %f, y: %f\n", set_speed_x, set_speed_y);
        float set_omega = limit_sym(current_omega + limit_sym((float)out_rotation / 8192 / 16, MAX_ACCELERATION_OMG / GOTO_POS_FREQ), MAX_OMEGA);
        float cos_heading = cos(heading);
        float sin_heading = sin(heading);
        float set_speed_x_robot = cos_heading * set_speed_x + sin_heading * set_speed_y;
        float set_speed_y_robot = -sin_heading * set_speed_x + cos_heading * set_speed_y;
        control_update_setpoint_vx(set_speed_x_robot);
        control_update_setpoint_vy(set_speed_y_robot);
        control_update_setpoint_omega(set_omega);
    }
}





static void calibrate_position(void)
{
    control_update_setpoint_omega(M_PI/2/5);
    OSTimeDly(OS_TICKS_PER_SEC * 5);
    control_update_setpoint_omega(0);

    control_update_setpoint_vx(-0.02);
    OSTimeDly(OS_TICKS_PER_SEC * 4);
    control_update_setpoint_vx(0);
    OSTimeDly(OS_TICKS_PER_SEC / 10);
    position_reset();
    control_update_setpoint_vx(0.05);
    OSTimeDly(OS_TICKS_PER_SEC * 4);
    control_update_setpoint_vx(0);

    control_update_setpoint_omega(-M_PI/2/5);
    OSTimeDly(OS_TICKS_PER_SEC * 5);
    control_update_setpoint_omega(0);

    float x, y, unused;
    get_position(&y, &unused);

    control_update_setpoint_vx(-0.02);
    OSTimeDly(OS_TICKS_PER_SEC * 4);
    control_update_setpoint_vx(0);
    OSTimeDly(OS_TICKS_PER_SEC / 10);
    position_reset();
    control_update_setpoint_vx(0.03);
    OSTimeDly(OS_TICKS_PER_SEC * 4);
    control_update_setpoint_vx(0);

    get_position(&x, &unused);
    position_reset_to(x, y, 0);
}


static bool wait_for_start(void)
{
    return !(IORD(PIO_BASE, 0) & 0x1000);
}

void match_task(void *arg)
{
    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);
    nastya_cs.vx_control_enable = false;
    nastya_cs.vy_control_enable = false;
    nastya_cs.omega_control_enable = false;

    OSTimeDly(OS_TICKS_PER_SEC / 2);

    // calibrate_position();


    while (!wait_for_start()) OSTimeDly(OS_TICKS_PER_SEC/100);
    nastya_cs.vx_control_enable = true;
    nastya_cs.vy_control_enable = true;
    nastya_cs.omega_control_enable = true;
    OSTimeDly(OS_TICKS_PER_SEC);
    // wait for start signal
    while (wait_for_start()) OSTimeDly(OS_TICKS_PER_SEC/100);
    if (team_red)
        position_reset_to(2.898, 0.120, 3.14159);
    else
        position_reset_to(0.102, 0.120, 0);

    match_start = uptime_get();
    match_has_startd = true;
    printf("much started [%d]\nwow\n", (int)match_start);

    OSTimeDly(OS_TICKS_PER_SEC*4);




    if (team_red) {
        goto_position(2.8, 0.6, -10, 0);
        goto_position(1.65, 0.6, 0, 1);
        float ang = 0.5235987756;
        goto_position(1.65, 0.015, 1.65 - 10*cos(ang), 0.015 + 10*sin(ang));
        goto_position(1.65, 0.0, 1.65 - 10*cos(ang), 0.0 + 10*sin(ang));
        goto_position(1.65, 0.1, 1.65 - 10*cos(ang), 0.1 + 10*sin(ang));
        goto_position(1.5,  0.6, 0, 0);
    }
    else {
        goto_position(0.2, 0.6, 10, 0);
        goto_position(1.35, 0.6, 3, 1);
        float ang = 0.5235987756;
        goto_position(1.35, 0.015, 1.35 + 10*cos(ang), 0.015 + 10*sin(ang));
        goto_position(1.35, 0.0, 1.35 + 10*cos(ang), 0.0 + 10*sin(ang));
        goto_position(1.35, 0.1, 1.35 + 10*cos(ang), 0.1 + 10*sin(ang));
        goto_position(1.5,  0.6, 0, 0);
    }

    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);
    nastya_cs.vx_control_enable = false;
    nastya_cs.vy_control_enable = false;
    nastya_cs.omega_control_enable = false;
    OSTaskDel(MATCH_TASK_PRIORITY);
}


void ready_for_match(void)
{

    OSTaskCreateExt(match_task,
                    NULL,
                    &match_task_stk[MATCH_TASK_STACKSIZE-1],
                    MATCH_TASK_PRIORITY,
                    MATCH_TASK_PRIORITY,
                    &match_task_stk[0],
                    MATCH_TASK_STACKSIZE,
                    NULL, 0);
}

void match_set_red(void)
{
    team_red = true;
}

void match_set_yellow(void)
{
    team_red = false;
}

void match_set_disable_position_control(bool dis_pos_ctl)
{
    // disable_postion_control = dis_pos_ctl;
}
