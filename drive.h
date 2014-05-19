#ifndef DRIVE_H
#define DRIVE_H

void drive_set_dest(float x, float y);
int drive_goto(float x, float y);
void start_drive_task(void);

#endif // DRIVE_H
