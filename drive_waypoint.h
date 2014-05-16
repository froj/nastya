#ifndef DRIVE_WAYPOINT_H_
#define DRIVE_WAYPOINT_H_

typedef struct{
   float x, y;
   float vx, vy;
} drive_waypoint_t;


void drive_waypoint_init();

drive_waypoint_t* drive_waypoint_get_next();

void drive_waypoint_set_destination(float x, float y);

#endif
