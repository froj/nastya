#include <stdio.h>
#include <math.h>
#include <ucos_ii.h>
#include <control_system_manager/control_system_manager.h>
#include <pid/pid.h>
#include <cvra_beacon.h>
#include "tasks.h"
#include <uptime.h>
#include "control.h"
#include "position_integration.h"
#include "hardware.h"
#include "plot_task.h"

#include "match.h"
#include "param.h"


#define MATCH_DURATION 90*1000000 // [us]

#define GOTO_POS_FREQ 20    // [Hz]

#define MAX_ACCELERATION_XY 1.5     //  [m/s/s]
#define MAX_ACCELERATION_OMG (M_PI * 6)     //  [rad/s/s]

#define MAX_SPEED_XY    0.5     // [m/s]
#define MAX_OMEGA       M_PI     // [rad/s]

#define X_MAX_ERR_INPUT 2.0 * 1024
#define Y_MAX_ERR_INPUT 2.0 * 1024
#define THETA_MAX_ERR_INPUT 0.3 *1024

OS_STK match_task_stk[MATCH_TASK_STACKSIZE];
OS_STK emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE];

timestamp_t match_start;
static cvra_beacon_t beacon;

bool disable_postion_control;
bool team_red;


static float limit_sym(float val, float max)
{
    if (val > max)
        return max;
    if (val < -max)
        return -max;
    return val;
}



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

void position_control_init()
{
    cvra_beacon_init(&beacon, AVOIDING_BASE, AVOIDING_IRQ, 100, 1., 1.);

    pid_init(&pos_x_pid);
    pid_set_gains(&pos_x_pid, 100, 0, 3000); // KP, KI, KD
    pid_set_maximums(&pos_x_pid, X_MAX_ERR_INPUT, 800, 0); // in , integral, out
    pid_set_out_shift(&pos_x_pid, 0);
    pid_set_derivate_filter(&pos_x_pid, 3);
    pid_init(&pos_y_pid);
    pid_set_gains(&pos_y_pid, 100, 0, 3000); // KP, KI, KD
    pid_set_maximums(&pos_y_pid, Y_MAX_ERR_INPUT, 800, 0); // in , integral, out
    pid_set_out_shift(&pos_y_pid, 0);
    pid_set_derivate_filter(&pos_y_pid, 3);
    pid_init(&theta_pid);
    pid_set_gains(&theta_pid, 4, 10, 2500); // KP, KI, KD
    pid_set_maximums(&theta_pid, THETA_MAX_ERR_INPUT, 400, 0); // in , integral, out
    pid_set_out_shift(&theta_pid, 0);
    pid_set_derivate_filter(&theta_pid, 3);
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
}

static float goto_stop_thershold;
static param_t goto_stop_thershold_param;

int goto_position(float dest_x, float dest_y, float lookat_x, float lookat_y)
{
    static bool is_init = false;

    if (!is_init) {
        position_control_init();
        is_init = true;
    }

    while (1) {
        OSTimeDly(OS_TICKS_PER_SEC / GOTO_POS_FREQ);
        if (disable_postion_control)
            continue;
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





#define EMERGENCY_STOP_ACCELERATION_XY    5.0 // [m/s^2]
#define EMERGENCY_STOP_ACCELERATION_ALPHA 5.0 // [rad/s^2]
#define EMERGENCY_STOP_UPDATE_FREQ        100 // [Hz]

#define EMERGENCY_STOP_DELTA_OMEGA EMERGENCY_STOP_ACCELERATION_ALPHA / EMERGENCY_STOP_UPDATE_FREQ
#define EMERGENCY_STOP_DELTA_VXY EMERGENCY_STOP_ACCELERATION_XY / EMERGENCY_STOP_UPDATE_FREQ

static bool emergency_stop(void)
{
    int i;

    for (i = 0; i < beacon.nb_beacon; i++) {
        // TODO also take heading into account
        if (beacon.beacon[i].distance > 15) {
            return true;
        }
    }

    return false;
}


void emergency_stop_task(void *arg)
{
    int stop_timeout = 0;
    while (1) {
        if (emergency_stop() || uptime_get() - match_start > MATCH_DURATION) {
            stop_timeout = EMERGENCY_STOP_UPDATE_FREQ / 10; // reset stop timer
        }
        if (stop_timeout > 0) {
            stop_timeout--;
            disable_postion_control = true;
            // ramp speed to 0
            float vx, vy, omega;
            vx = control_get_setpoint_vx();
            vy = control_get_setpoint_vy();
            omega = control_get_setpoint_omega();
            // get_velocity(&vx, &vy);
            // omega = get_omega();
            if (fabs(omega) < EMERGENCY_STOP_DELTA_OMEGA) {
                omega = 0;
            } else if (omega > 0) {
                omega -= EMERGENCY_STOP_DELTA_OMEGA;
            } else if (omega < 0) {
                omega += EMERGENCY_STOP_DELTA_OMEGA;
            }
            if (fabs(vx) < EMERGENCY_STOP_DELTA_VXY) {
                vx = 0;
            } else if (vx > 0) {
                vx -= EMERGENCY_STOP_DELTA_VXY;
            } else if (vx < 0) {
                vx += EMERGENCY_STOP_DELTA_VXY;
            }
            if (fabs(vy) < EMERGENCY_STOP_DELTA_VXY) {
                vy = 0;
            } else if (vy > 0) {
                vy -= EMERGENCY_STOP_DELTA_VXY;
            } else if (vy < 0) {
                vy += EMERGENCY_STOP_DELTA_VXY;
            }
            // printf("stop: %f %f %f\n", vx, vy, omega);
            control_update_setpoint_vx(vx);
            control_update_setpoint_vy(vy);
            control_update_setpoint_omega(omega);
        } else {
            disable_postion_control = false;
        }
        OSTimeDly(OS_TICKS_PER_SEC/EMERGENCY_STOP_UPDATE_FREQ);
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
    printf("much started [%d]\nwow\n", (int)match_start);

    OSTimeDly(OS_TICKS_PER_SEC*4);


    OSTaskCreateExt(emergency_stop_task,
                    NULL,
                    &emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE-1],
                    EMERGENCY_STOP_TASK_PRIORITY,
                    EMERGENCY_STOP_TASK_PRIORITY,
                    &emergency_stop_task_stk[0],
                    EMERGENCY_STOP_TASK_STACKSIZE,
                    NULL, 0);


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
    OSTaskDel(EMERGENCY_STOP_TASK_PRIORITY);
    OSTaskDel(MATCH_TASK_PRIORITY);
}


void ready_for_match(void)
{
    param_add(&goto_stop_thershold, "goto_stop", NULL);
    param_set(&goto_stop_thershold, 0.0032);

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
    disable_postion_control = dis_pos_ctl;
}
