
#include <stdint.h>
#include <uptime.h>


#include "drive_open_loop_dynamic_path.h"

void drive_open_loop_dynamic_path(struct dynamic_waypoint *wp,
    int nb_waypoints)
{
    timestamp_t start_time = uptime_get();
    int i;
    for (i = 0; i < nb_waypoints; i++) {
        // cs set consign from wp[i]
        timestamp_t end_time = start_time + wp[i].duration;
        timestamp_t now = uptime_get();
        // delay (end_time - now)
        start_time = end_time;
    }
}


