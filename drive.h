#ifndef DRIVE_H
#define DRIVE_H

void drive_set_heading(float heading);
void drive_set_look_at(float x, float y);
void drive_disable_heading_ctrl();
void drive_set_dest(float x, float y);
int drive_goto(float x, float y);
void drive_sync_heading(void);
void start_drive_task(void);

#endif // DRIVE_H
