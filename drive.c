
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ucos_ii.h>
#include "control.h"
#include <param/param.h>
#include <trace/trace.h>
#include "position_integration.h"
#include "match.h"
#include "util.h"
#include "drive_waypoint.h"
#include "tasks.h"

#include "drive.h"

OS_STK    drive_task_stk[DRIVE_TASK_STACKSIZE];



bool disable_postion_control = false;


static float dest_x = 0;
static float dest_y = 0;
static float dest_heading = 0;
static float look_at_x = 0;
static float look_at_y = 0;
#define DRIVE_HEADING_MODE_FREE   0
#define DRIVE_HEADING_MODE_ANGLE  1
#define DRIVE_HEADING_MODE_POINT  2
static int drive_heading_mode = DRIVE_HEADING_MODE_FREE;


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


void drive_set_dest(float x, float y)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    dest_x = x;
    dest_y = y;
    OS_EXIT_CRITICAL();
    drive_waypoint_set_destination(x, y);
}

int drive_goto(float x, float y)
{
    drive_set_dest(x, y);
    while (!destination_reached()) {
        OSTimeDly(OS_TICKS_PER_SEC/20);
    }
    return 0;
}



#define DRIVE_CTRL_FREQ_DEFAULT 20 // [Hz]
static param_t drive_ctrl_freq;

#define MAX_ACC_XY_DEFAULT      1.5         // [m/s^2]
#define MAX_SPEED_XY_DEFAULT    0.5         // [m/s]
#define MAX_ALPHA_DEFAULT       (M_PI * 6)  // [rad/s^2]
#define MAX_OMEGA_DEFAULT       M_PI        // [rad/s]
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
    param_t theta_pid_P;
    param_t theta_pid_I;
    param_t theta_pid_D;
    param_t theta_pid_D_filt;
    param_t theta_pid_I_bound;
    struct cs pos_x_cs;
    struct cs pos_y_cs;
    struct cs theta_cs;
    struct pid_filter pos_x_pid;
    struct pid_filter pos_y_pid;
    struct pid_filter theta_pid;
};

#define PID_SCALE_OUT           131072
#define PID_SCALE_IN            1024

#define X_MAX_ERR_INPUT         2.0 * PID_SCALE_IN
#define Y_MAX_ERR_INPUT         2.0 * PID_SCALE_IN
#define THETA_MAX_ERR_INPUT     0.3 * PID_SCALE_IN

static struct pos_cs_s pos_cs;
static struct pos_cs_s fallback_pos_cs;

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
    param_add(&pos_cs.theta_pid_P, "pid_theta_P", NULL);
    param_add(&pos_cs.theta_pid_I, "pid_theta_I", NULL);
    param_add(&pos_cs.theta_pid_D, "pid_theta_D", NULL);
    param_add(&pos_cs.theta_pid_D_filt, "pid_theta_D_filt", NULL);
    param_add(&pos_cs.theta_pid_I_bound, "pid_theta_I_bound", NULL);

    // pos xy
    param_set(&pos_cs.pos_xy_pid_P, 0);
    param_set(&pos_cs.pos_xy_pid_I, 0);
    param_set(&pos_cs.pos_xy_pid_D, 0);
    param_set(&pos_cs.pos_xy_pid_D_filt, 3);
    param_set(&pos_cs.pos_xy_pid_I_bound, 800);
    // theta
    param_set(&pos_cs.theta_pid_P, 0);
    param_set(&pos_cs.theta_pid_I, 0);
    param_set(&pos_cs.theta_pid_D, 0);
    param_set(&pos_cs.theta_pid_D_filt, 3);
    param_set(&pos_cs.theta_pid_I_bound, 400);

    pid_init(&pos_cs.pos_x_pid);
    pid_set_out_shift(&pos_cs.pos_x_pid, 0);
    pid_init(&pos_cs.pos_y_pid);
    pid_set_out_shift(&pos_cs.pos_y_pid, 0);
    pid_init(&pos_cs.theta_pid);
    pid_set_out_shift(&pos_cs.theta_pid, 0);

    cs_init(&pos_cs.pos_x_cs);
    cs_init(&pos_cs.pos_y_cs);
    cs_init(&pos_cs.theta_cs);
    cs_set_correct_filter(&pos_cs.pos_x_cs, pid_do_filter, &pos_cs.pos_x_pid);
    cs_set_correct_filter(&pos_cs.pos_y_cs, pid_do_filter, &pos_cs.pos_y_pid);
    cs_set_correct_filter(&pos_cs.theta_cs, pid_do_filter, &pos_cs.theta_pid);
    cs_set_process_in(&pos_cs.pos_x_cs, cs_out, &out_x);
    cs_set_process_in(&pos_cs.pos_y_cs, cs_out, &out_y);
    cs_set_process_in(&pos_cs.theta_cs, cs_out, &out_rotation);
    cs_set_process_out(&pos_cs.pos_x_cs, cs_in, &in_x);
    cs_set_process_out(&pos_cs.pos_y_cs, cs_in, &in_y);
    cs_set_process_out(&pos_cs.theta_cs, cs_in, &in_rotation);
    cs_set_consign(&pos_cs.pos_x_cs, 0);
    cs_set_consign(&pos_cs.pos_y_cs, 0);
    cs_set_consign(&pos_cs.theta_cs, 0);


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
    param_add(&fallback_pos_cs.theta_pid_P, "fallback_pid_theta_P", NULL);
    param_add(&fallback_pos_cs.theta_pid_I, "fallback_pid_theta_I", NULL);
    param_add(&fallback_pos_cs.theta_pid_D, "fallback_pid_theta_D", NULL);
    param_add(&fallback_pos_cs.theta_pid_D_filt, "fallback_pid_theta_D_filt", NULL);
    param_add(&fallback_pos_cs.theta_pid_I_bound, "fallback_pid_theta_I_bound", NULL);

    // pos xy
    param_set(&fallback_pos_cs.pos_xy_pid_P, 30);
    param_set(&fallback_pos_cs.pos_xy_pid_I, 0);
    param_set(&fallback_pos_cs.pos_xy_pid_D, 10);
    param_set(&fallback_pos_cs.pos_xy_pid_D_filt, 3);
    param_set(&fallback_pos_cs.pos_xy_pid_I_bound, 800);
    // theta
    param_set(&fallback_pos_cs.theta_pid_P, 4);
    param_set(&fallback_pos_cs.theta_pid_I, 10);
    param_set(&fallback_pos_cs.theta_pid_D, 2500);
    param_set(&fallback_pos_cs.theta_pid_D_filt, 3);
    param_set(&fallback_pos_cs.theta_pid_I_bound, 400);

    pid_init(&fallback_pos_cs.pos_x_pid);
    pid_set_out_shift(&fallback_pos_cs.pos_x_pid, 0);
    pid_init(&fallback_pos_cs.pos_y_pid);
    pid_set_out_shift(&fallback_pos_cs.pos_y_pid, 0);
    pid_init(&fallback_pos_cs.theta_pid);
    pid_set_out_shift(&fallback_pos_cs.theta_pid, 0);

    cs_init(&fallback_pos_cs.pos_x_cs);
    cs_init(&fallback_pos_cs.pos_y_cs);
    cs_init(&fallback_pos_cs.theta_cs);
    cs_set_correct_filter(&fallback_pos_cs.pos_x_cs, pid_do_filter, &fallback_pos_cs.pos_x_pid);
    cs_set_correct_filter(&fallback_pos_cs.pos_y_cs, pid_do_filter, &fallback_pos_cs.pos_y_pid);
    cs_set_correct_filter(&fallback_pos_cs.theta_cs, pid_do_filter, &fallback_pos_cs.theta_pid);
    cs_set_process_in(&fallback_pos_cs.pos_x_cs, cs_out, &out_x);
    cs_set_process_in(&fallback_pos_cs.pos_y_cs, cs_out, &out_y);
    cs_set_process_in(&fallback_pos_cs.theta_cs, cs_out, &out_rotation);
    cs_set_process_out(&fallback_pos_cs.pos_x_cs, cs_in, &in_x);
    cs_set_process_out(&fallback_pos_cs.pos_y_cs, cs_in, &in_y);
    cs_set_process_out(&fallback_pos_cs.theta_cs, cs_in, &in_rotation);
    cs_set_consign(&fallback_pos_cs.pos_x_cs, 0);
    cs_set_consign(&fallback_pos_cs.pos_y_cs, 0);
    cs_set_consign(&fallback_pos_cs.theta_cs, 0);
}

static void update_pid_parameters(struct pos_cs_s *p)
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
    // pid theta
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


typedef struct {
    timestamp_t last_sign_change;
    int prev_sign;
    int32_t min_period;
} anti_osc_filt_t;

void anti_osc_filt_init(anti_osc_filt_t *f, float freq)
{
    f->last_sign_change = uptime_get();
    f->prev_sign = 1;
    f->min_period = 1000000 / freq;
}

float anti_osc_filt(anti_osc_filt_t *f, float in)
{
    if (in > 0 && f->prev_sign > 0) {
        return in;
    }
    if (in < 0 && f->prev_sign < 0) {
        return in;
    }
    // input sign changed
    timestamp_t now = uptime_get();
    if (now - f->last_sign_change < f->min_period) {
        return 0; // don't change sing yet, out = 0
    } else {
        f->prev_sign = - f->prev_sign;
        f->last_sign_change = now;
        return in;
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
        // static bool pos_ctrl_was_prev_disabled = true;
        if (disable_postion_control) {
            // pos_ctrl_was_prev_disabled = true;
            continue;
        }

        float pos_x, pos_y;
        get_position(&pos_x, &pos_y);

        // if (pos_ctrl_was_prev_disabled) {

        //     pos_ctrl_was_prev_disabled = false;
        // }

        update_drive_params();
        float x_err, y_err;
        drive_waypoint_t *wp;
        if ((wp = drive_waypoint_get_next()) != NULL) { // waypoints available
            printf("drive using waypoints %f %f\n (%f %f %f %f)\n\n",
                dest_x, dest_y, wp->x, wp->y, wp->vx, wp->vy);
            update_pid_parameters(&pos_cs);
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
            printf("drive using fallback pid %f %f\n", dest_x, dest_y);
            update_pid_parameters(&fallback_pos_cs);
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

        float current_heading = get_heading();
        float heading_err = 0;
        // switch heading contorl mode
        //      heading control -> pid_rot_out
        // float heading = get_heading();
        // float set_heading = atan2(lookat_y - pos_y, lookat_x - pos_x);
        // float heading_err = circular_range(heading - set_heading);
        // in_rotation = heading_err * PID_SCALE_IN;
        // cs_manage(&pos_cs.theta_cs);
        // set_omega += (float)out_rotation / PID_SCALE_OUT;
        trace_var_update(&x_err_tr, x_err);
        trace_var_update(&y_err_tr, y_err);
        trace_var_update(&theta_err_tr, heading_err);

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
        trace_var_update(&x_out_tr, set_vx);
        trace_var_update(&y_out_tr, set_vy);
        trace_var_update(&theta_out_tr, set_omega);

        // coordinate transform to robot coordinate system
        float sin_heading = sin(current_heading);
        float cos_heading = cos(current_heading);
        control_update_setpoint_vx(cos_heading * set_vx + sin_heading * set_vy);
        control_update_setpoint_vy(-sin_heading * set_vx + cos_heading * set_vy);
        control_update_setpoint_omega(set_omega);
    }
}


// TODO move to drive_emergency_stop.c/h

#include <cvra_beacon.h>

OS_STK match_task_stk[MATCH_TASK_STACKSIZE];
OS_STK emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE];

#define EMERGENCY_STOP_ACCELERATION_XY    5.0 // [m/s^2]
#define EMERGENCY_STOP_ACCELERATION_ALPHA 5.0 // [rad/s^2]
#define EMERGENCY_STOP_UPDATE_FREQ        100 // [Hz]

#define EMERGENCY_STOP_DELTA_OMEGA EMERGENCY_STOP_ACCELERATION_ALPHA / EMERGENCY_STOP_UPDATE_FREQ
#define EMERGENCY_STOP_DELTA_VXY EMERGENCY_STOP_ACCELERATION_XY / EMERGENCY_STOP_UPDATE_FREQ

static cvra_beacon_t beacon;

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
        if (emergency_stop() ||
            (match_has_started && uptime_get() - match_start > MATCH_DURATION)) {
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
    cvra_beacon_init(&beacon, (void*)AVOIDING_BASE, AVOIDING_IRQ, 100, 1., 1.);
    OSTaskCreateExt(emergency_stop_task,
                    NULL,
                    &emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE-1],
                    EMERGENCY_STOP_TASK_PRIORITY,
                    EMERGENCY_STOP_TASK_PRIORITY,
                    &emergency_stop_task_stk[0],
                    EMERGENCY_STOP_TASK_STACKSIZE,
                    NULL, 0);
}

