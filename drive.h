#ifndef DRIVE_H
#define DRIVE_H

extern bool emergency_stop_en;

extern bool enable_postion_control;
extern bool enable_heading_control;

void drive_set_heading(float heading);
void drive_set_look_at(float x, float y);
void drive_disable_heading_ctrl();
void drive_set_dest(float x, float y);
int drive_goto(float x, float y, int timeout, bool cancel_if_blocked_by_opponent);
int drive_sync_heading(int timeout);
void start_drive_task(void);

#endif // DRIVE_H
