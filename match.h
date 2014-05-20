#ifndef MATCH_H
#define MATCH_H

#include <pid.h>
#include <stdbool.h>
#include <uptime.h>

#define MATCH_DURATION 90*1000000 // [us]

extern bool match_has_started;
extern timestamp_t match_start;

void ready_for_match(void);
int goto_position(float dest_x, float dest_y, float lookat_x, float lookat_y);
void match_set_red(void);
void match_set_yellow(void);
void match_restart(bool team_red);

#define MATCH_ACTION_NOP                0
#define MATCH_ACTION_MOVE               1   //  (x, y)
#define MATCH_ACTION_SET_HEADING        2   //  (heading)
#define MATCH_ACTION_SET_LOOK_AT        3   //  (x, y)
#define MATCH_ACTION_SYNC_HEADING       4   //  wait for heading error to be 0
#define MATCH_ACTION_FIRE_CANNON        5   //  (cannon_index)
#define MATCH_ACTION_CAPTURE_MAMMOTH    6   //  ()
#define MATCH_ACTION_WAIT_END_OF_MATCH  7

typedef struct {
    int cmd;
    float arg1, arg2;
} match_action_t;

#endif // MATCH_H
