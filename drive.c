
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ucos_ii.h>
#include "control.h"
#include <param/param.h>
#include <trace/trace.h>
#include "position_integration.h"
#include "match.h"
#include <uptime.h>
#include "util.h"
#include "drive_waypoint.h"
#include "tasks.h"

#include "drive.h"

OS_STK    drive_task_stk[DRIVE_TASK_STACKSIZE];

bool emergency_stop_en = false;

bool enable_postion_control = true;
bool enable_heading_control = true;
static bool emergency_stop_disable_heading_and_pos_ctrl = false;

static float dest_x = 0;
static float dest_y = 0;
static float dest_heading = 0;
static float look_at_x = 0;
static float look_at_y = 0;
#define DRIVE_HEADING_MODE_FREE   0
#define DRIVE_HEADING_MODE_ANGLE  1
#define DRIVE_HEADING_MODE_POINT  2
static int drive_heading_mode = DRIVE_HEADING_MODE_FREE;




static bool emergency_stop(void);


bool destination_reached()
{
    float goto_stop_thershold = 0.0032;
    float current_speed_x, current_speed_y;
    get_velocity(&current_speed_x, &current_speed_y);
    float current_omega = get_omega();
    float pos_x, pos_y;
    get_position(&pos_x, &pos_y);
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    float x_err = pos_x - dest_x;
    float y_err = pos_y - dest_y;
    OS_EXIT_CRITICAL();
    if (x_err*x_err + y_err*y_err < goto_stop_thershold
        && current_speed_x*current_speed_x + current_speed_y*current_speed_y + current_omega*current_omega < goto_stop_thershold) {
        return true; // destination reached
    } else {
        return false;
    }
}

void drive_set_heading(float heading)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    dest_heading = heading;
    drive_heading_mode = DRIVE_HEADING_MODE_ANGLE;
    OS_EXIT_CRITICAL();
}

void drive_set_look_at(float x, float y)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    look_at_x = x;
    look_at_y = y;
    drive_heading_mode = DRIVE_HEADING_MODE_POINT;
    OS_EXIT_CRITICAL();
}

void drive_disable_heading_ctrl()
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    drive_heading_mode = DRIVE_HEADING_MODE_FREE;
    OS_EXIT_CRITICAL();
}


void drive_set_dest(float x, float y)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    dest_x = x;
    dest_y = y;
    OS_EXIT_CRITICAL();
    drive_waypoint_set_destination(x, y);
}

#define DRIVE_OK                    0
#define DRIVE_TIMEOUT               1
#define DRIVE_MATCH_ACTION_TIMEOUT  2
#define DRIVE_BLOCKED_BY_OPPONENT   3

// timeouts > 0: return after 'timeout' us
// timeouts < 0: return 'timeout' us before end of match
// cancel_if_blocked_by_opponent: return if opponent blocks the way to destination
int drive_goto(float x, float y, int timeout, bool cancel_if_blocked_by_opponent)
{
    timestamp_t fn_enter = uptime_get();
    drive_set_dest(x, y);
    while (!destination_reached()) {
        if (timeout > 0 && uptime_get() - fn_enter > timeout)
                return DRIVE_TIMEOUT;
        int t = 0;
        if (timeout < 0)
            t = -timeout;
        if (match_action_timeout(t))
            return DRIVE_MATCH_ACTION_TIMEOUT;
        if (cancel_if_blocked_by_opponent && emergency_stop())
            return DRIVE_BLOCKED_BY_OPPONENT;
        OSTimeDly(OS_TICKS_PER_SEC/20);
    }
    return DRIVE_OK;
}

static float calc_heading_err(void)
{
    if (drive_heading_mode == DRIVE_HEADING_MODE_FREE) {
        return 0;
    }
    if (drive_heading_mode == DRIVE_HEADING_MODE_POINT) {
        float pos_x = get_position_x();
        float pos_y = get_position_y();
        float set = atan2(look_at_y - pos_y, look_at_x - pos_x);
        return circular_range(get_heading() - set);
    }
    if (drive_heading_mode == DRIVE_HEADING_MODE_ANGLE) {
        return circular_range(get_heading() - dest_heading);
    }
    return 0;
}

int drive_sync_heading(int timeout)
{
    timestamp_t fn_enter = uptime_get();
    while (fabsf(calc_heading_err()) > 3.14*1/180) {
        if (timeout > 0 && uptime_get() - fn_enter > timeout)
            return DRIVE_TIMEOUT;
        int t = 0;
        if (timeout < 0)
            t = -timeout;
        if (match_action_timeout(t))
            return DRIVE_MATCH_ACTION_TIMEOUT;
        OSTimeDly(OS_TICKS_PER_SEC/20);
    }
    return DRIVE_OK;
}

#define DRIVE_CTRL_FREQ_DEFAULT 33.333 // [Hz]
static param_t drive_ctrl_freq;

#define MAX_ACC_XY_DEFAULT      0.03         // [m/s^2]
#define MAX_SPEED_XY_DEFAULT    0.5          // [m/s]
#define MAX_ALPHA_DEFAULT       0.2          // [rad/s^2]
#define MAX_OMEGA_DEFAULT       1            // [rad/s]
static param_t max_acc_xy_p;
static param_t max_speed_xy_p;
static param_t max_alpha_p;
static param_t max_omega_p;
static float max_acc_xy = 0;
static float max_speed_xy = 0;
static float max_alpha = 0;
static float max_omega = 0;

struct pos_cs_s {
    param_t pos_xy_pid_P;
    param_t pos_xy_pid_I;
    param_t pos_xy_pid_D;
    param_t pos_xy_pid_D_filt;
    param_t pos_xy_pid_I_bound;
    param_t pos_x_pid_P;
    param_t pos_x_pid_I;
    param_t pos_x_pid_D;
    param_t pos_x_pid_D_filt;
    param_t pos_x_pid_I_bound;
    param_t pos_y_pid_P;
    param_t pos_y_pid_I;
    param_t pos_y_pid_D;
    param_t pos_y_pid_D_filt;
    param_t pos_y_pid_I_bound;
    struct cs pos_x_cs;
    struct cs pos_y_cs;
    struct pid_filter pos_x_pid;
    struct pid_filter pos_y_pid;
};

struct heading_cs_s {
    param_t theta_pid_P;
    param_t theta_pid_I;
    param_t theta_pid_D;
    param_t theta_pid_D_filt;
    param_t theta_pid_I_bound;
    struct cs theta_cs;
    struct pid_filter theta_pid;
};

#define PID_SCALE_OUT           131072
#define PID_SCALE_IN            1024

#define X_MAX_ERR_INPUT         2.0 * PID_SCALE_IN
#define Y_MAX_ERR_INPUT         2.0 * PID_SCALE_IN
#define THETA_MAX_ERR_INPUT     0.3 * PID_SCALE_IN

static struct pos_cs_s pos_cs;
static struct pos_cs_s fallback_pos_cs;
static struct heading_cs_s heading_cs;

static int32_t out_x;
static int32_t out_y;
static int32_t out_rotation;
static int32_t in_x;
static int32_t in_y;
static int32_t in_rotation;

static void cs_out(void *arg, int32_t out)
{
    *(int32_t*)arg = out;
}

static int32_t cs_in(void *arg)
{
    return *(int32_t*)arg;
}

static void position_control_init()
{
    // waypoint position control

    param_add(&pos_cs.pos_xy_pid_P, "pid_pos_xy_P", NULL);
    param_add(&pos_cs.pos_xy_pid_I, "pid_pos_xy_I", NULL);
    param_add(&pos_cs.pos_xy_pid_D, "pid_pos_xy_D", NULL);
    param_add(&pos_cs.pos_xy_pid_D_filt, "pid_pos_xy_D_filt", NULL);
    param_add(&pos_cs.pos_xy_pid_I_bound, "pid_pos_xy_I_bound", NULL);
    param_add(&pos_cs.pos_x_pid_P, "pid_pos_x_P", NULL);
    param_add(&pos_cs.pos_x_pid_I, "pid_pos_x_I", NULL);
    param_add(&pos_cs.pos_x_pid_D, "pid_pos_x_D", NULL);
    param_add(&pos_cs.pos_x_pid_D_filt, "pid_pos_x_D_filt", NULL);
    param_add(&pos_cs.pos_x_pid_I_bound, "pid_pos_x_I_bound", NULL);
    param_add(&pos_cs.pos_y_pid_P, "pid_pos_y_P", NULL);
    param_add(&pos_cs.pos_y_pid_I, "pid_pos_y_I", NULL);
    param_add(&pos_cs.pos_y_pid_D, "pid_pos_y_D", NULL);
    param_add(&pos_cs.pos_y_pid_D_filt, "pid_pos_y_D_filt", NULL);
    param_add(&pos_cs.pos_y_pid_I_bound, "pid_pos_y_I_bound", NULL);

    param_set(&pos_cs.pos_xy_pid_P, 0);
    param_set(&pos_cs.pos_xy_pid_I, 0);
    param_set(&pos_cs.pos_xy_pid_D, 0);
    param_set(&pos_cs.pos_xy_pid_D_filt, 3);
    param_set(&pos_cs.pos_xy_pid_I_bound, 800);

    pid_init(&pos_cs.pos_x_pid);
    pid_set_out_shift(&pos_cs.pos_x_pid, 0);
    pid_init(&pos_cs.pos_y_pid);
    pid_set_out_shift(&pos_cs.pos_y_pid, 0);

    cs_init(&pos_cs.pos_x_cs);
    cs_init(&pos_cs.pos_y_cs);
    cs_set_correct_filter(&pos_cs.pos_x_cs, pid_do_filter, &pos_cs.pos_x_pid);
    cs_set_correct_filter(&pos_cs.pos_y_cs, pid_do_filter, &pos_cs.pos_y_pid);
    cs_set_process_in(&pos_cs.pos_x_cs, cs_out, &out_x);
    cs_set_process_in(&pos_cs.pos_y_cs, cs_out, &out_y);
    cs_set_process_out(&pos_cs.pos_x_cs, cs_in, &in_x);
    cs_set_process_out(&pos_cs.pos_y_cs, cs_in, &in_y);
    cs_set_consign(&pos_cs.pos_x_cs, 0);
    cs_set_consign(&pos_cs.pos_y_cs, 0);

    // fallback position control

    param_add(&fallback_pos_cs.pos_xy_pid_P, "fallback_pid_pos_xy_P", NULL);
    param_add(&fallback_pos_cs.pos_xy_pid_I, "fallback_pid_pos_xy_I", NULL);
    param_add(&fallback_pos_cs.pos_xy_pid_D, "fallback_pid_pos_xy_D", NULL);
    param_add(&fallback_pos_cs.pos_xy_pid_D_filt, "fallback_pid_pos_xy_D_filt", NULL);
    param_add(&fallback_pos_cs.pos_xy_pid_I_bound, "fallback_pid_pos_xy_I_bound", NULL);
    param_add(&fallback_pos_cs.pos_x_pid_P, "fallback_pid_pos_x_P", NULL);
    param_add(&fallback_pos_cs.pos_x_pid_I, "fallback_pid_pos_x_I", NULL);
    param_add(&fallback_pos_cs.pos_x_pid_D, "fallback_pid_pos_x_D", NULL);
    param_add(&fallback_pos_cs.pos_x_pid_D_filt, "fallback_pid_pos_x_D_filt", NULL);
    param_add(&fallback_pos_cs.pos_x_pid_I_bound, "fallback_pid_pos_x_I_bound", NULL);
    param_add(&fallback_pos_cs.pos_y_pid_P, "fallback_pid_pos_y_P", NULL);
    param_add(&fallback_pos_cs.pos_y_pid_I, "fallback_pid_pos_y_I", NULL);
    param_add(&fallback_pos_cs.pos_y_pid_D, "fallback_pid_pos_y_D", NULL);
    param_add(&fallback_pos_cs.pos_y_pid_D_filt, "fallback_pid_pos_y_D_filt", NULL);
    param_add(&fallback_pos_cs.pos_y_pid_I_bound, "fallback_pid_pos_y_I_bound", NULL);

    param_set(&fallback_pos_cs.pos_xy_pid_P, 400);
    param_set(&fallback_pos_cs.pos_xy_pid_I, 1);
    param_set(&fallback_pos_cs.pos_xy_pid_D, 60);
    param_set(&fallback_pos_cs.pos_xy_pid_D_filt, 3);
    param_set(&fallback_pos_cs.pos_xy_pid_I_bound, 800);

    pid_init(&fallback_pos_cs.pos_x_pid);
    pid_set_out_shift(&fallback_pos_cs.pos_x_pid, 0);
    pid_init(&fallback_pos_cs.pos_y_pid);
    pid_set_out_shift(&fallback_pos_cs.pos_y_pid, 0);

    cs_init(&fallback_pos_cs.pos_x_cs);
    cs_init(&fallback_pos_cs.pos_y_cs);
    cs_set_correct_filter(&fallback_pos_cs.pos_x_cs, pid_do_filter, &fallback_pos_cs.pos_x_pid);
    cs_set_correct_filter(&fallback_pos_cs.pos_y_cs, pid_do_filter, &fallback_pos_cs.pos_y_pid);
    cs_set_process_in(&fallback_pos_cs.pos_x_cs, cs_out, &out_x);
    cs_set_process_in(&fallback_pos_cs.pos_y_cs, cs_out, &out_y);
    cs_set_process_out(&fallback_pos_cs.pos_x_cs, cs_in, &in_x);
    cs_set_process_out(&fallback_pos_cs.pos_y_cs, cs_in, &in_y);
    cs_set_consign(&fallback_pos_cs.pos_x_cs, 0);
    cs_set_consign(&fallback_pos_cs.pos_y_cs, 0);

    // heading control

    param_add(&heading_cs.theta_pid_P, "pid_theta_P", NULL);
    param_add(&heading_cs.theta_pid_I, "pid_theta_I", NULL);
    param_add(&heading_cs.theta_pid_D, "pid_theta_D", NULL);
    param_add(&heading_cs.theta_pid_D_filt, "pid_theta_D_filt", NULL);
    param_add(&heading_cs.theta_pid_I_bound, "pid_theta_I_bound", NULL);

    param_set(&heading_cs.theta_pid_P, 1200);
    param_set(&heading_cs.theta_pid_I, 10);
    param_set(&heading_cs.theta_pid_D, 200);
    param_set(&heading_cs.theta_pid_D_filt, 3);
    param_set(&heading_cs.theta_pid_I_bound, 400);

    pid_init(&heading_cs.theta_pid);
    pid_set_out_shift(&heading_cs.theta_pid, 0);

    cs_init(&heading_cs.theta_cs);
    cs_set_correct_filter(&heading_cs.theta_cs, pid_do_filter, &heading_cs.theta_pid);
    cs_set_process_in(&heading_cs.theta_cs, cs_out, &out_rotation);
    cs_set_process_out(&heading_cs.theta_cs, cs_in, &in_rotation);
    cs_set_consign(&heading_cs.theta_cs, 0);
}

static void update_pos_pid_parameters(struct pos_cs_s *p)
{
    // pid xy combined
    if (param_has_changed(&p->pos_xy_pid_P)) {
        param_set(&p->pos_x_pid_P, param_get(&p->pos_xy_pid_P));
        param_set(&p->pos_y_pid_P, param_get(&p->pos_xy_pid_P));
    }
    if (param_has_changed(&p->pos_xy_pid_I)) {
        param_set(&p->pos_x_pid_I, param_get(&p->pos_xy_pid_I));
        param_set(&p->pos_y_pid_I, param_get(&p->pos_xy_pid_I));
    }
    if (param_has_changed(&p->pos_xy_pid_D)) {
        param_set(&p->pos_x_pid_D, param_get(&p->pos_xy_pid_D));
        param_set(&p->pos_y_pid_D, param_get(&p->pos_xy_pid_D));
    }
    if (param_has_changed(&p->pos_xy_pid_D_filt)) {
        param_set(&p->pos_x_pid_D_filt, param_get(&p->pos_xy_pid_D_filt));
        param_set(&p->pos_y_pid_D_filt, param_get(&p->pos_xy_pid_D_filt));
    }
    if (param_has_changed(&p->pos_xy_pid_I_bound)) {
        param_set(&p->pos_x_pid_I_bound, param_get(&p->pos_xy_pid_I_bound));
        param_set(&p->pos_y_pid_I_bound, param_get(&p->pos_xy_pid_I_bound));
    }
    // pid x
    if (param_has_changed(&p->pos_x_pid_P)
        || param_has_changed(&p->pos_x_pid_I)
        || param_has_changed(&p->pos_x_pid_D)) {
        pid_set_gains(&p->pos_x_pid,
                      param_get(&p->pos_x_pid_P),
                      param_get(&p->pos_x_pid_I),
                      param_get(&p->pos_x_pid_D));
    }
    if (param_has_changed(&p->pos_x_pid_I_bound)) {
        pid_set_maximums(&p->pos_x_pid, X_MAX_ERR_INPUT,
                         param_get(&p->pos_x_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&p->pos_x_pid_D_filt)) {
        pid_set_derivate_filter(&p->pos_x_pid,
                                param_get(&p->pos_x_pid_D_filt));
    }
    // pid y
    if (param_has_changed(&p->pos_y_pid_P)
        || param_has_changed(&p->pos_y_pid_I)
        || param_has_changed(&p->pos_y_pid_D)) {
        pid_set_gains(&p->pos_y_pid,
                      param_get(&p->pos_y_pid_P),
                      param_get(&p->pos_y_pid_I),
                      param_get(&p->pos_y_pid_D));
    }
    if (param_has_changed(&p->pos_y_pid_I_bound)) {
        pid_set_maximums(&p->pos_y_pid, Y_MAX_ERR_INPUT,
                         param_get(&p->pos_y_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&p->pos_y_pid_D_filt)) {
        pid_set_derivate_filter(&p->pos_y_pid,
                                param_get(&p->pos_y_pid_D_filt));
    }
}

static void update_heading_pid_parameters(struct heading_cs_s *p)
{
    if (param_has_changed(&p->theta_pid_P)
        || param_has_changed(&p->theta_pid_I)
        || param_has_changed(&p->theta_pid_D)) {
        pid_set_gains(&p->theta_pid,
                      param_get(&p->theta_pid_P),
                      param_get(&p->theta_pid_I),
                      param_get(&p->theta_pid_D));
    }
    if (param_has_changed(&p->theta_pid_I_bound)) {
        pid_set_maximums(&p->theta_pid, THETA_MAX_ERR_INPUT,
                         param_get(&p->theta_pid_I_bound), 0); // in , integral, out
    }
    if (param_has_changed(&p->theta_pid_D_filt)) {
        pid_set_derivate_filter(&p->theta_pid,
                                param_get(&p->theta_pid_D_filt));
    }
}

static void update_drive_params(void)
{
    if (param_has_changed(&max_acc_xy_p)) {
        max_acc_xy = param_get(&max_acc_xy_p);
    }
    if (param_has_changed(&max_speed_xy_p)) {
        max_speed_xy = param_get(&max_speed_xy_p);
    }
    if (param_has_changed(&max_alpha_p)) {
        max_alpha = param_get(&max_alpha_p);
    }
    if (param_has_changed(&max_omega_p)) {
        max_omega = param_get(&max_omega_p);
    }
}

void drive_task(void *pdata)
{
    printf("drive task started\n");
    int period_us = 1;
    float set_vx = 0;
    float set_vy = 0;
    float set_omega = 0;
    float prev_set_vx = 0;
    float prev_set_vy = 0;
    float prev_set_omega = 0;
    trace_var_t x_err_tr;
    trace_var_t y_err_tr;
    trace_var_t theta_err_tr;
    trace_var_t x_out_tr;
    trace_var_t y_out_tr;
    trace_var_t theta_out_tr;
    trace_var_add(&x_err_tr, "x_err");
    trace_var_add(&y_err_tr, "y_err");
    trace_var_add(&theta_err_tr, "theta_err");
    trace_var_add(&x_out_tr, "x_out");
    trace_var_add(&y_out_tr, "y_out");
    trace_var_add(&theta_out_tr, "theta_out");
    while (1) {
        if (param_has_changed(&drive_ctrl_freq)) {
            period_us = OS_TICKS_PER_SEC / param_get(&drive_ctrl_freq);
        }
        OSTimeDly(period_us);

        float pos_x, pos_y;
        get_position(&pos_x, &pos_y);

        update_drive_params();
        float x_err, y_err;
        drive_waypoint_t *wp;
        if ((wp = drive_waypoint_get_next()) != NULL) { // waypoints available
            printf("drive using waypoints %f %f\n (%f %f %f %f)\n\n",
                dest_x, dest_y, wp->x, wp->y, wp->vx, wp->vy);
            pid_reset(&fallback_pos_cs.pos_x_pid);
            pid_reset(&fallback_pos_cs.pos_y_pid);
            update_pos_pid_parameters(&pos_cs);
            set_vx = wp->vx;
            set_vy = wp->vy;
            set_omega = 0;
            x_err = pos_x - wp->x;
            y_err = pos_y - wp->y;
            // pid control
            in_x = x_err * PID_SCALE_IN;
            in_y = y_err * PID_SCALE_IN;
            cs_manage(&pos_cs.pos_x_cs);
            cs_manage(&pos_cs.pos_y_cs);
        } else { // no waypoints available: use fallback position controller
            // printf("drive using fallback pid %f %f\n", dest_x, dest_y);
            pid_reset(&pos_cs.pos_x_pid);
            pid_reset(&pos_cs.pos_y_pid);
            update_pos_pid_parameters(&fallback_pos_cs);
            set_vx = 0;
            set_vy = 0;
            set_omega = 0;
            OS_CPU_SR cpu_sr;
            OS_ENTER_CRITICAL();
            x_err = pos_x - dest_x;
            y_err = pos_y - dest_y;
            OS_EXIT_CRITICAL();
            // pid control
            in_x = x_err * PID_SCALE_IN;
            in_y = y_err * PID_SCALE_IN;
            cs_manage(&fallback_pos_cs.pos_x_cs);
            cs_manage(&fallback_pos_cs.pos_y_cs);
        }
        set_vx += (float)out_x / PID_SCALE_OUT;
        set_vy += (float)out_y / PID_SCALE_OUT;

        float heading_err;
        switch (drive_heading_mode) {
        case DRIVE_HEADING_MODE_POINT:
        case DRIVE_HEADING_MODE_ANGLE:
            update_heading_pid_parameters(&heading_cs);
            heading_err = calc_heading_err();
            in_rotation = heading_err * PID_SCALE_IN;
            cs_manage(&heading_cs.theta_cs);
            set_omega += (float)out_rotation / PID_SCALE_OUT;
            break;
        case DRIVE_HEADING_MODE_FREE:
            set_omega = 0;
            break;
        }

        // limit acceleration & maximum speed
        float delta_vx = limit_sym(set_vx - prev_set_vx, max_acc_xy);
        set_vx = limit_sym(prev_set_vx + delta_vx, max_speed_xy);
        float delta_vy = limit_sym(set_vy - prev_set_vy, max_acc_xy);
        set_vy = limit_sym(prev_set_vy + delta_vy, max_speed_xy);
        float delta_omega = limit_sym(set_omega - prev_set_omega, max_alpha);
        set_omega = limit_sym(prev_set_omega + delta_omega, max_omega);
        prev_set_vx = set_vx;
        prev_set_vy = set_vy;
        prev_set_omega = set_omega;

        trace_var_update(&x_err_tr, x_err);
        trace_var_update(&y_err_tr, y_err);
        trace_var_update(&theta_err_tr, heading_err);
        trace_var_update(&x_out_tr, set_vx);
        trace_var_update(&y_out_tr, set_vy);
        trace_var_update(&theta_out_tr, set_omega);

        // coordinate transform to robot coordinate system
        float current_heading = get_heading();
        float sin_heading = sin(current_heading);
        float cos_heading = cos(current_heading);
        if (enable_postion_control && !emergency_stop_disable_heading_and_pos_ctrl) {
            control_update_setpoint_vx(cos_heading * set_vx + sin_heading * set_vy);
            control_update_setpoint_vy(-sin_heading * set_vx + cos_heading * set_vy);
        } else {
            prev_set_vx = 0;
            prev_set_vy = 0;
            pid_reset(&pos_cs.pos_x_pid);
            pid_reset(&pos_cs.pos_y_pid);
            pid_reset(&fallback_pos_cs.pos_x_pid);
            pid_reset(&fallback_pos_cs.pos_y_pid);
        }
        if (enable_heading_control && !emergency_stop_disable_heading_and_pos_ctrl) {
            control_update_setpoint_omega(set_omega);
        } else {
            prev_set_omega = 0;
            pid_reset(&heading_cs.theta_pid);
        }
    }
}


// TODO move to drive_emergency_stop.c/h

#include <cvra_beacon.h>

OS_STK match_task_stk[MATCH_TASK_STACKSIZE];
OS_STK emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE];

#define EMERGENCY_STOP_ACCELERATION_XY    1.0 // [m/s^2]
#define EMERGENCY_STOP_ACCELERATION_ALPHA 3.0 // [rad/s^2]
#define EMERGENCY_STOP_UPDATE_FREQ        100 // [Hz]

#define EMERGENCY_STOP_DELTA_OMEGA EMERGENCY_STOP_ACCELERATION_ALPHA / EMERGENCY_STOP_UPDATE_FREQ
#define EMERGENCY_STOP_DELTA_VXY EMERGENCY_STOP_ACCELERATION_XY / EMERGENCY_STOP_UPDATE_FREQ

static cvra_beacon_t beacon;
param_t emergency_stop_dist1_p, emergency_stop_ang1_p;
param_t emergency_stop_dist2_p, emergency_stop_ang2_p;

static bool emergency_stop(void)
{
    static float emergency_stop_ang1, emergency_stop_dist1;
    static float emergency_stop_ang2, emergency_stop_dist2;
    if (param_has_changed(&emergency_stop_dist1_p))
        emergency_stop_dist1 = param_get(&emergency_stop_dist1_p);
    if (param_has_changed(&emergency_stop_ang1_p))
        emergency_stop_ang1 = param_get(&emergency_stop_ang1_p);
    if (param_has_changed(&emergency_stop_dist2_p))
        emergency_stop_dist2 = param_get(&emergency_stop_dist2_p);
    if (param_has_changed(&emergency_stop_ang2_p))
        emergency_stop_ang2 = param_get(&emergency_stop_ang2_p);
    float pos_x, pos_y;
    get_position(&pos_x, &pos_y);
    if ((dest_x - pos_x)*(dest_x - pos_x) + (dest_y - pos_y)*(dest_y - pos_y) < 0.01*0.01)
        return false; // don't stop if close to destination
    float heading = get_heading();
    float dest_dir = atan2(dest_y - pos_y, dest_x - pos_x);
    int i;
    for (i = 0; i < beacon.nb_beacon; i++) {
        // printf("beacon %d ang: %f dist: %f\n", i, beacon.beacon[i].direction, beacon.beacon[i].distance);
        float beacon_dir = beacon.beacon[i].direction/180*M_PI + heading;

        // printf("rel ang: %f\n", fabsf(circular_range(dest_dir - beacon_dir)));
        if (beacon.beacon[i].distance > emergency_stop_dist1
            && fabsf(circular_range(dest_dir - beacon_dir)) < emergency_stop_ang1) {
            return true;
        }
        if (beacon.beacon[i].distance > emergency_stop_dist2
            && fabsf(circular_range(dest_dir - beacon_dir)) < emergency_stop_ang2) {
            return true;
        }
    }

    return false;
}

void emergency_stop_task(void *arg)
{
    param_add(&emergency_stop_dist1_p, "emerg_stop_dist1", "[beacon size (ang)]");
    param_add(&emergency_stop_ang1_p, "emerg_stop_ang1", "[rad]");
    param_set(&emergency_stop_dist1_p, 9.0);
    param_set(&emergency_stop_ang1_p, M_PI / 6);
    param_add(&emergency_stop_dist2_p, "emerg_stop_dist2", "[beacon size (ang)]");
    param_add(&emergency_stop_ang2_p, "emerg_stop_ang2", "[rad]");
    param_set(&emergency_stop_dist2_p, 15.0);
    param_set(&emergency_stop_ang2_p, M_PI / 3);

    static bool controllers_disabled = false;

    int stop_timeout = 0;
    while (1) {
        if (emergency_stop() ||
            (match_running && uptime_get() - match_start > MATCH_DURATION - 100000)) {
            stop_timeout = EMERGENCY_STOP_UPDATE_FREQ / 10; // reset stop timer
        }
        if (stop_timeout > 0 && emergency_stop_en) {
            stop_timeout--;
            emergency_stop_disable_heading_and_pos_ctrl = true;
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

            // disable controllers to look innocent and keep the wheels from
            //  slipping in case of a collision
            if (vx == 0 && vy == 0 && omega ==0) {
                if (!controllers_disabled) {
                    controllers_disabled = true;
                    nastya_cs.vx_control_enable = false;
                    nastya_cs.vy_control_enable = false;
                    nastya_cs.omega_control_enable = false;
                }
            }
        } else {
            if (controllers_disabled) {
                controllers_disabled = false;
                nastya_cs.vx_control_enable = true;
                nastya_cs.vy_control_enable = true;
                nastya_cs.omega_control_enable = true;
            }
            emergency_stop_disable_heading_and_pos_ctrl = false;
        }
        OSTimeDly(OS_TICKS_PER_SEC/EMERGENCY_STOP_UPDATE_FREQ);
    }
}





void start_drive_task(void)
{
    drive_waypoint_init();

    param_add(&drive_ctrl_freq, "drive_ctrl_freq", "[Hz]");
    param_set(&drive_ctrl_freq, DRIVE_CTRL_FREQ_DEFAULT);

    param_add(&max_acc_xy_p, "max_acc_xy", NULL);
    param_add(&max_speed_xy_p, "max_speed_xy", NULL);
    param_add(&max_alpha_p, "max_alpha", NULL);
    param_add(&max_omega_p, "max_omega", NULL);
    param_set(&max_acc_xy_p, MAX_ACC_XY_DEFAULT);
    param_set(&max_speed_xy_p, MAX_SPEED_XY_DEFAULT);
    param_set(&max_alpha_p, MAX_ALPHA_DEFAULT);
    param_set(&max_omega_p, MAX_OMEGA_DEFAULT);

    position_control_init();

    OSTaskCreateExt(drive_task,
                    NULL,
                    &drive_task_stk[DRIVE_TASK_STACKSIZE-1],
                    DRIVE_TASK_PRIORITY,
                    DRIVE_TASK_PRIORITY,
                    &drive_task_stk[0],
                    DRIVE_TASK_STACKSIZE,
                    NULL, 0);

    // Emergency stop init
    cvra_beacon_init(&beacon, (void*)AVOIDING_BASE, AVOIDING_IRQ, 50, 1., 1.);
    cvra_beacon_set_direction_offset(&beacon, 68);
    OSTaskCreateExt(emergency_stop_task,
                    NULL,
                    &emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE-1],
                    EMERGENCY_STOP_TASK_PRIORITY,
                    EMERGENCY_STOP_TASK_PRIORITY,
                    &emergency_stop_task_stk[0],
                    EMERGENCY_STOP_TASK_STACKSIZE,
                    NULL, 0);
}

