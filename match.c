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

OS_STK match_task_stk[MATCH_TASK_STACKSIZE];

// bool disable_postion_control;
bool team_red;

timestamp_t match_start;
bool match_has_startd = false;


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




    // if (team_red) {
    //     goto_position(2.8, 0.6, -10, 0);
    //     goto_position(1.65, 0.6, 0, 1);
    //     float ang = 0.5235987756;
    //     goto_position(1.65, 0.015, 1.65 - 10*cos(ang), 0.015 + 10*sin(ang));
    //     goto_position(1.65, 0.0, 1.65 - 10*cos(ang), 0.0 + 10*sin(ang));
    //     goto_position(1.65, 0.1, 1.65 - 10*cos(ang), 0.1 + 10*sin(ang));
    //     goto_position(1.5,  0.6, 0, 0);
    // }
    // else {
    //     goto_position(0.2, 0.6, 10, 0);
    //     goto_position(1.35, 0.6, 3, 1);
    //     float ang = 0.5235987756;
    //     goto_position(1.35, 0.015, 1.35 + 10*cos(ang), 0.015 + 10*sin(ang));
    //     goto_position(1.35, 0.0, 1.35 + 10*cos(ang), 0.0 + 10*sin(ang));
    //     goto_position(1.35, 0.1, 1.35 + 10*cos(ang), 0.1 + 10*sin(ang));
    //     goto_position(1.5,  0.6, 0, 0);
    // }

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

