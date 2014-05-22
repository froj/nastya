#include <stdio.h>
#include <string.h>
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
#include "util.h"
#include "drive.h"

#include "match.h"
#include "param.h"

OS_STK match_task_stk[MATCH_TASK_STACKSIZE];

bool next_match_team_red = false;
static bool restart_match = false;

timestamp_t match_start;
bool match_running = false;
static bool match_abort;



// mirror functions to change coordinates to the other team

float mirror_x(float x)
{
    return MATCH_TABLE_LENGHT - x;
}

float mirror_y(float y)
{
    return y;
}

float mirror_heading(float h)
{
    return circular_range(M_PI - h);
}



#define MAMMOTH_CAPTURE_SETUP_TIME 3000000 // [us]

#define MAX_NB_MATCH_ACTIONS    128
static match_action_t match_actions[MAX_NB_MATCH_ACTIONS] = {
    {1, 0.466437, 0.553406},
    {1, 1.300000, 0.600000},
    {2, 0.520000, 0.000000},
    {1, 1.300000, 0.120000},
    {1, 1.300000, 0.250000},
    {1, 1.100000, 0.600000},
    {1, 0.800000, 0.600000},
    {0, 0.000000, 0.000000},
    {0, 0.000000, 0.000000}
};


static void match_exec(bool team_red, match_action_t *a);


bool match_action_timeout()
{
    return (match_abort ||
        (match_running && uptime_get() - match_start > MATCH_DURATION - MAMMOTH_CAPTURE_SETUP_TIME));
}


static bool wait_for_start(void)
{
    return !(IORD(PIO_BASE, 0) & 0x1000);
}

#define RESET_POS_X     0.125
#define RESET_POS_Y     0.135
#define RESET_HEADING   (- M_PI / 3)

#define START_POS_X     0.27091094851494
#define START_POS_Y     0.35258761048317
#define START_HEADING   -4.0729007720947

void match_run(void)
{
    emergency_stop_en = false;
    match_running = false;
    match_abort = false;
    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);
    nastya_cs.vx_control_enable = false;
    nastya_cs.vy_control_enable = false;
    nastya_cs.omega_control_enable = false;
    enable_postion_control = false;
    enable_heading_control = false;

    hw_cannon_arm_all();

    OSTimeDly(OS_TICKS_PER_SEC / 2);
    while (!wait_for_start()) OSTimeDly(OS_TICKS_PER_SEC/100);

    OSTimeDly(OS_TICKS_PER_SEC / 2);

    bool team_red = next_match_team_red;
    if (team_red) {
        position_reset_to(mirror_x(RESET_POS_X), mirror_y(RESET_POS_Y), mirror_heading(RESET_HEADING));
        drive_set_dest(mirror_x(RESET_POS_X), mirror_y(RESET_POS_Y));
    } else {
        position_reset_to(RESET_POS_X, RESET_POS_Y, RESET_HEADING);
        drive_set_dest(RESET_POS_X, RESET_POS_Y);
    }
    drive_disable_heading_ctrl();
    enable_postion_control = true;
    enable_heading_control = true;
    nastya_cs.vx_control_enable = true;
    nastya_cs.vy_control_enable = true;
    nastya_cs.omega_control_enable = true;

    OSTimeDly(OS_TICKS_PER_SEC * 2);

    if (team_red) {
        drive_goto(mirror_x(0.19162034988403), mirror_y(0.19018436968327));
        drive_goto(mirror_x(0.21569117903709), mirror_y(0.25893285870552));
        OSTimeDly(OS_TICKS_PER_SEC * 3);
        drive_set_heading(mirror_heading(START_HEADING));
        drive_goto(mirror_x(0.26092061400414), mirror_y(0.35386016964912));
        drive_goto(mirror_x(START_POS_X), mirror_y(START_POS_Y));
    } else {
        drive_goto(0.19162034988403, 0.19018436968327);
        drive_goto(0.21569117903709, 0.25893285870552);
        OSTimeDly(OS_TICKS_PER_SEC * 3);
        drive_set_heading(START_HEADING);
        drive_goto(0.26092061400414, 0.35386016964912);
        drive_goto(START_POS_X, START_POS_Y);
    }

    // wait for start signal
    while (wait_for_start()) {
        OSTimeDly(OS_TICKS_PER_SEC/100);
        if (match_abort)
            goto abort_match;
    }
    emergency_stop_en = true;

    match_start = uptime_get();
    match_running = true;
    printf("much started [%d]\nwow\n", (int)match_start);

    int i;
    for (i = 0; i < MAX_NB_MATCH_ACTIONS; i++) {
        match_exec(team_red, &match_actions[i]);
        if (match_abort)
            goto abort_match;
        if (uptime_get() - match_start > MATCH_DURATION - MAMMOTH_CAPTURE_SETUP_TIME)
            goto end_of_match;
    }

end_of_match:
    while (uptime_get() - match_start < MATCH_DURATION - MAMMOTH_CAPTURE_SETUP_TIME) {
        if (match_abort)
            goto abort_match;
        OSTimeDly(OS_TICKS_PER_SEC/100);
    }


    // choose nearest mammoth
    float pos_x, pos_y;
    get_position(&pos_x, &pos_y);
    float mammoth_x, mammoth_y = 0;
    if (pos_x > MATCH_TABLE_LENGHT/2) {
        mammoth_x = MATCH_TABLE_LENGHT - 0.8;
    } else {
        mammoth_x = 0.8;
    }
    // orient robot for mammoth capture
    drive_set_dest(pos_x, pos_y);
    drive_set_look_at(mammoth_x, mammoth_y);

    emergency_stop_en = false;

    // wait until .5s after match
    while (uptime_get() - match_start < MATCH_DURATION + 500000)
        OSTimeDly(OS_TICKS_PER_SEC/100);

    // fire
    hw_set_net(1);
    OSTimeDly(OS_TICKS_PER_SEC);
    hw_set_net(0);

    printf("captured mammoth\n");

abort_match:
    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);
    match_running = false;
    match_abort = false;
    return;

    // wait for new match
    while (!restart_match) OSTimeDly(OS_TICKS_PER_SEC/10);
    restart_match = false;
}



void match_task(void *arg)
{
    while (!wait_for_start() && !restart_match) OSTimeDly(OS_TICKS_PER_SEC/100);
    while (1) {
        match_run();
    }
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
    next_match_team_red = true;
}

void match_set_yellow(void)
{
    next_match_team_red = false;
}

void match_restart(bool team_red)
{
    match_abort = true;
    restart_match = true;
    next_match_team_red = team_red;
}


static void match_exec(bool team_red, match_action_t *a)
{
    int wait_ms;
    switch (a->cmd) {
    case MATCH_ACTION_MOVE:
        if (team_red) {
            drive_goto(mirror_x(a->arg1), mirror_y(a->arg2));
        } else {
            drive_goto(a->arg1, a->arg2);
        }
        break;
    case MATCH_ACTION_SET_HEADING:
        if (team_red) {
            drive_set_heading(mirror_heading(a->arg1));
        } else {
            drive_set_heading(a->arg1);
        }
        break;
    case MATCH_ACTION_SET_LOOK_AT:
        if (team_red) {
            drive_set_look_at(mirror_x(a->arg1), mirror_y(a->arg2));
        } else {
            drive_set_look_at(a->arg1, a->arg2);
        }
        break;
    case MATCH_ACTION_SYNC_HEADING:
        drive_sync_heading();
        break;
    case MATCH_ACTION_FIRE_CANNON:
        hw_cannon_fire(a->arg1);
        // printf("cannon %d: boooom!\n", (int)a->arg1);
        break;
    case MATCH_ACTION_SLEEP_MS:
        wait_ms = a->arg1;
        while (wait_ms > 0) {
            OSTimeDly(OS_TICKS_PER_SEC / 100);
            wait_ms -= 10;
            if (match_action_timeout())
                break;
        }
        break;
    case MATCH_ACTION_WAIT_END_OF_MATCH:
        while (!match_action_timeout())
            OSTimeDly(OS_TICKS_PER_SEC / 100);
        break;
    }
}

int match_action_list(char* buffer, int buf_len)
{
    int i, ret, remaining_sz = buf_len;

    for(i = 0; i < MAX_NB_MATCH_ACTIONS; i++) {

        switch (match_actions[i].cmd) {
            case MATCH_ACTION_NOP:
                if (i == 0 || (i != 0 && match_actions[i-1].cmd != MATCH_ACTION_NOP)){
                    ret = snprintf(buffer, remaining_sz, "[%3d] NOP\n", i);
                } else {
                    ret = 0;
                }
                break;
            case MATCH_ACTION_MOVE:
                ret = snprintf(buffer, remaining_sz,
                               "[%3d] Move (%1.3f, %1.3f)\n",
                               i, match_actions[i].arg1, match_actions[i].arg2);
                break;
            case MATCH_ACTION_SET_HEADING:
                ret = snprintf(buffer, remaining_sz,
                               "[%3d] Set heading to %+1.2f\n",
                               i, match_actions[i].arg1);
                break;
            case MATCH_ACTION_SET_LOOK_AT:
                ret = snprintf(buffer, remaining_sz,
                               "[%3d] Look at (%1.3f, %1.3f)\n",
                               i, match_actions[i].arg1, match_actions[i].arg2);
                break;
            case MATCH_ACTION_SYNC_HEADING:
                ret = snprintf(buffer, remaining_sz, "[%3d] Sync heading.\n", i);
                break;
            case MATCH_ACTION_FIRE_CANNON:
                ret = snprintf(buffer, remaining_sz,
                               "[%3d] Fire cannon Nb. %1.0f Arrrr!\n",
                               i, match_actions[i].arg1);
                break;
            case MATCH_ACTION_WAIT_END_OF_MATCH:
                ret = snprintf(buffer, remaining_sz, "[%3d] Wait end of match.\n", i);
                break;
            case MATCH_ACTION_SLEEP_MS:
                ret = snprintf(buffer, remaining_sz, "[%3d] Sleep %.0f ms.\n",
                               i, match_actions[i].arg1);
                break;
            default:
                ret = snprintf(buffer, remaining_sz, "[%3d] Unknown command.\n", i);
                break;
        }

        if (ret < 0) {
            // snprintf error
            return -1;
        }

        if (ret > remaining_sz) {
            // buffer overflow
            return -2;
        }

        buffer += ret;
        remaining_sz -= ret;
    }

    return 0;
}

void match_action_modify(int index, int cmd, float arg1, float arg2)
{
    if (index < MAX_NB_MATCH_ACTIONS && index >= 0) {
        match_actions[index].cmd = cmd;
        match_actions[index].arg1 = arg1;
        match_actions[index].arg2 = arg2;
    }
}

void match_action_insert(int index) {
    if (index < MAX_NB_MATCH_ACTIONS - 1 && index >= 0) {
        memmove(&match_actions[index+1], &match_actions[index],
                (MAX_NB_MATCH_ACTIONS - index - 1) * sizeof(match_action_t));
    } else if (index != MAX_NB_MATCH_ACTIONS - 1){
        return; // invalid index
    }
    match_action_modify(index, MATCH_ACTION_NOP, 0, 0);
}

void match_action_delete(int index)
{
    if (index < MAX_NB_MATCH_ACTIONS - 1 && index >= 0) {
        memmove(&match_actions[index], &match_actions[index+1],
                (MAX_NB_MATCH_ACTIONS - index - 1) * sizeof(match_action_t));
    } else if (index != MAX_NB_MATCH_ACTIONS - 1){
        return; // invalid index
    }
    match_action_modify(MAX_NB_MATCH_ACTIONS - 1, MATCH_ACTION_NOP, 0, 0);
}

int match_action_save_as_c_code(char* buffer, int buf_len)
{
    int i, ret, remaining_sz = buf_len;

    // print first line
    ret = snprintf(buffer, remaining_sz,
        "static match_action_t match_actions[MAX_NB_MATCH_ACTIONS] = {\n");

    if (ret < 0) return -1;
    if (ret > remaining_sz) return -2;
    buffer += ret;
    remaining_sz -= ret;

    for(i = 0; i < MAX_NB_MATCH_ACTIONS; i++) {

        if (i != MAX_NB_MATCH_ACTIONS - 1) {
            if (i > 0 && match_actions[i - 1].cmd == MATCH_ACTION_NOP &&
                       match_actions[i].cmd == MATCH_ACTION_NOP) {
                ret = 0;
            } else {
                ret = snprintf(buffer, remaining_sz, "\t{%d, %f, %f},\n",
                               match_actions[i].cmd,
                               match_actions[i].arg1,
                               match_actions[i].arg2);
           }
        } else {
            ret = snprintf(buffer, remaining_sz, "\t{%d, %f, %f}\n};\n",
                           match_actions[i].cmd,
                           match_actions[i].arg1,
                           match_actions[i].arg2);
        }

        if (ret < 0) {
            // snprintf error
            return -1;
        }

        if (ret > remaining_sz) {
            // buffer overflow
            return -2;
        }

        buffer += ret;
        remaining_sz -= ret;
    }

    return 0;
}
