#include <modules/modules/uptime/uptime.h>
#include <lwip/ip.h>
#include <param/param.h>
#include "position_integration.h"
#include "obstacle_avoidance_protocol.h"
#include <ucos_ii.h>
#include "tasks.h"

#include "drive_waypoint.h"

static OS_CPU_SR cpu_sr;
static OS_EVENT* mutex;
OS_STK drive_waypoint_stk[DRIVE_WAYPOINT_STACKSIZE];
#define LOCK() {OS_ENTER_CRITICAL();}
#define UNLOCK() {OS_EXIT_CRITICAL();}

#define DRIVE_REQUEST_TIMEOUT_DEFAULT   100000 // [us]
#define DESIRED_NB_DATAPOINTS_DEFAULT       50
#define DESIRED_SAMPLE_PERIOD_DEFAULT       20 // [ms]


static param_t drive_request_timeout;
static param_t desired_nb_datapoints;
static param_t desired_sample_period;


static obstacle_avoidance_point_t destination;
static float dest_x = 0, dest_y = 0;
static obstacle_avoidance_path_t path;
static timestamp_t request_time;
static int waypoint_index;

void drive_waypoint_task(void *arg);
static void send_request();

void drive_waypoint_init()
{
    waypoint_index = 0;
    mutex = OSSemCreate(1);

    destination.vx = 0;
    destination.vy = 0;

    param_add(&drive_request_timeout, "drive_request_timeout", "[us]");
    param_add(&desired_nb_datapoints, "desired_nb_datapoints", NULL);
    param_add(&desired_sample_period, "desired_sample_period", "[ms]");

    param_set(&drive_request_timeout, DRIVE_REQUEST_TIMEOUT_DEFAULT);
    param_set(&desired_nb_datapoints, DESIRED_NB_DATAPOINTS_DEFAULT);
    param_set(&desired_sample_period, DESIRED_SAMPLE_PERIOD_DEFAULT);

    OSTaskCreateExt(drive_waypoint_task,
                    NULL,
                    &drive_waypoint_stk[DRIVE_WAYPOINT_STACKSIZE-1],
                    DRIVE_WAYPOINT_PRIORITY,
                    DRIVE_WAYPOINT_PRIORITY,
                    &drive_waypoint_stk[0],
                    DRIVE_WAYPOINT_STACKSIZE,
                    NULL, 0);
}

void drive_waypoint_set_destination(float x, float y)
{
    LOCK();

    destination.x = x*1000;
    destination.y = y*1000;
    dest_x = x;
    dest_y = y;

    UNLOCK();
}


drive_waypoint_t* drive_waypoint_get_next()
{
    static drive_waypoint_t next_waypoint;
    // lock path with mutex
    INT8U uCErr;
    OSSemPend(mutex, 0, &uCErr);

    if (path.len == 0) {
        OSSemPost(mutex);
        LOCK();
        next_waypoint.x = dest_x;
        next_waypoint.y = dest_y;
        UNLOCK();
        next_waypoint.vx = 0;
        next_waypoint.vy = 0;
        return &next_waypoint;
    }

    int32_t relative_now = uptime_get() - request_time;
    if (relative_now > param_get(&drive_request_timeout)) {
        OSSemPost(mutex);
        return NULL;
    }

    int i;
    for (i = waypoint_index; i < path.len; i++) {
        if (path.points[i].timestamp >= relative_now) {
            // TODO closest, not just next
            break;
        }
    }
    next_waypoint.x = (float)path.points[i].x / 1000;
    next_waypoint.y = (float)path.points[i].y / 1000;
    next_waypoint.vx = (float)path.points[i].vx / 1000;
    next_waypoint.vy = (float)path.points[i].vy / 1000;

    waypoint_index = i;

    OSSemPost(mutex);

    return &next_waypoint;

}

static void send_request()
{
    static obstacle_avoidance_path_t path_buffer;
    obstacle_avoidance_request_t request;
    int err;
    INT8U uCErr;
    struct ip_addr remote_ip;

    IP4_ADDR(&remote_ip, 10, 0, 0, 21);

    timestamp_t request_started = uptime_get();
    obstacle_avoidance_request_create(&request, 0);
    request.desired_datapoints = param_get(&desired_nb_datapoints);
    request.desired_samplerate = param_get(&desired_sample_period);
    request.start.x = get_position_x();
    request.start.y = get_position_y();
    request.start.vx = get_velocity_x();
    request.start.vy = get_velocity_y();

    LOCK();
    request.end = destination;
    UNLOCK();

    err = obstacle_avoidance_send_request(&request, remote_ip, 1337, &path_buffer);

    if (err == ERR_OK) {
        OSSemPend(mutex, 0, &uCErr);

        obstacle_avoidance_delete_path(&path);
        // copy path
        path.points = path_buffer.points;
        path_buffer.points = NULL;
        path.len = path_buffer.len;
        path_buffer.len = 0;
        obstacle_avoidance_delete_path(&path_buffer);

        waypoint_index = 0;

        request_time = request_started;

        OSSemPost(mutex);
    }

    obstacle_avoidance_request_delete(&request);
}

void drive_waypoint_task(void *arg)
{
    while(42) {
        send_request();
    }
}

