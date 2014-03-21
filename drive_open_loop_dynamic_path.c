
#include <stdint.h>
#include <uptime.h>
#include <ucos_ii.h>

#include "drive_open_loop_dynamic_path.h"

void drive_open_loop_dynamic_path(struct dynamic_waypoint *wp,
    int nb_waypoints)
{
    timestamp_t start_time = uptime_get();
    int i;
    for (i = 0; i < nb_waypoints; i++) {
        control_update_setpoint_vx(wp[i].vx);
        control_update_setpoint_vy(wp[i].vy);
        control_update_setpoint_omega(wp[i].omega);
        timestamp_t end_time = start_time + wp[i].duration;
        timestamp_t now = uptime_get();
        start_time = end_time;
        int32_t delay = end_time - now;
        if (delay > 0)
            OSTimeDly((int64_t)OS_TICKS_PER_SEC*delay/1000000);
    }
}


