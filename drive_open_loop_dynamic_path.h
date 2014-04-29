#ifndef DRIVE_OPEN_LOOP_DYNAMIC_PATH
#define DRIVE_OPEN_LOOP_DYNAMIC_PATH

#include <stdint.h>

struct dynamic_waypoint {
    float vx;          // [m/s] speed in x (robot coordinate system)
    float vy;          // [m/s] speed in y (robot coordinate system)
    float omega;       // [rad/s] rotational speed (robot coordinate system)
    uint32_t duration; // [us]
};


// this function returns after finishing the trajectory, it must
// be called from a task context (sleep system call is used)
void drive_open_loop_dynamic_path(struct dynamic_waypoint *wp,
    int nb_waypoints);

#endif // DRIVE_OPEN_LOOP_DYNAMIC_PATH
