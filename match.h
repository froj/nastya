#ifndef MATCH_H
#define MATCH_H

#include <pid.h>
#include <stdbool.h>
#include <uptime.h>

#define MATCH_DURATION 90*1000000 // [us]
#define MATCH_TABLE_LENGHT  3
#define MATCH_TABLE_WIDTH   2

extern bool match_running;
extern timestamp_t match_start;

void ready_for_match(void);
int goto_position(float dest_x, float dest_y, float lookat_x, float lookat_y);
void match_set_red(void);
void match_set_yellow(void);
void match_restart(bool team_red);
bool match_action_timeout(int time_to_end_of_match);

#define MATCH_ACTION_NOP                0
#define MATCH_ACTION_MOVE               1   //  (x, y)
#define MATCH_ACTION_SET_HEADING        2   //  (heading)
#define MATCH_ACTION_SET_LOOK_AT        3   //  (x, y)
#define MATCH_ACTION_SYNC_HEADING       4   //  wait for heading error to be 0
#define MATCH_ACTION_FIRE_CANNON        5   //  (cannon_index)
#define MATCH_ACTION_WAIT_END_OF_MATCH  6
#define MATCH_ACTION_SLEEP_MS           7   //  (ms)

typedef struct {
    int cmd;
    float arg1, arg2;
} match_action_t;

int match_action_list(char* buffer, int buf_len);
void match_action_modify(int index, int cmd, float arg1, float arg2);
void match_action_insert(int index);
void match_action_delete(int index);
int match_action_save_as_c_code(char* buffer, int buf_len);

#endif // MATCH_H
