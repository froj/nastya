
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ucos_ii.h>
#include "control.h"
#include <param/param.h>
#include "match.h"
#include "tasks.h"

#include "drive.h"

OS_STK    drive_task_stk[DRIVE_TASK_STACKSIZE];

#define DRIVE_CTRL_FREQ_DEFAULT 20 // [Hz]
static param_t drive_ctrl_freq;


bool disable_postion_control = false;


int drive_goto(float x, float y)
{

}



void drive_task(void *pdata)
{
    printf("drive task started\n");
    int period_us = 1;
    while (1) {
        if (param_has_changed(&drive_ctrl_freq)) {
            period_us = OS_TICKS_PER_SEC / param_get(&drive_ctrl_freq);
        }
        OSTimeDly(period_us);
        if (disable_postion_control)
            continue;

        // if 

        // if waypts avail
        //   use next waypt for destxy

        // pid x,y

        // vxy += dxy

        // switch theta ctrl mode

    }
}


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
            (match_has_startd && uptime_get() - match_start > MATCH_DURATION)) {
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
    param_add(&drive_ctrl_freq, "drive_ctrl_freq", "[Hz]");
    param_set(&drive_ctrl_freq, DRIVE_CTRL_FREQ_DEFAULT);

    OSTaskCreateExt(drive_task,
                    NULL,
                    &drive_task_stk[DRIVE_TASK_STACKSIZE-1],
                    DRIVE_TASK_PRIORITY,
                    DRIVE_TASK_PRIORITY,
                    &drive_task_stk[0],
                    DRIVE_TASK_STACKSIZE,
                    NULL, 0);

    OSTaskCreateExt(emergency_stop_task,
                    NULL,
                    &emergency_stop_task_stk[EMERGENCY_STOP_TASK_STACKSIZE-1],
                    EMERGENCY_STOP_TASK_PRIORITY,
                    EMERGENCY_STOP_TASK_PRIORITY,
                    &emergency_stop_task_stk[0],
                    EMERGENCY_STOP_TASK_STACKSIZE,
                    NULL, 0);
}

