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

#endif // MATCH_H
