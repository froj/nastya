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

#define DRIVE_REQUEST_TIMEOUT_DEFAULT   300000 // [us]
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
static int send_request();

void drive_waypoint_init()
{
    waypoint_index = 0;
    mutex = OSSemCreate(1);

    destination.vx = 0;
    destination.vy = 0;

    path.points = NULL;
    path.len = 0;

    param_add(&drive_request_timeout, "drive_request_timeout", "[us]");
    param_add(&desired_nb_datapoints, "desired_nb_datapoints", NULL);
    param_add(&desired_sample_period, "desired_sample_period", "[ms]");

    param_set(&drive_request_timeout, DRIVE_REQUEST_TIMEOUT_DEFAULT);
    param_set(&desired_nb_datapoints, DESIRED_NB_DATAPOINTS_DEFAULT);
    param_set(&desired_sample_period, DESIRED_SAMPLE_PERIOD_DEFAULT);

    // ensure timeout if there has never been a connection to server
    request_time = uptime_get() - param_get(&drive_request_timeout);

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

    timestamp_t now = uptime_get();
    int32_t relative_now = now - request_time;
    if (relative_now > param_get(&drive_request_timeout)) {
        printf("rel now: %d, req: %d, now %d\n", relative_now, request_time, now);
        OSSemPost(mutex);
        return NULL;
    }

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

    int i;
    float delta_pos, old_delta_pos;
    for (i = waypoint_index; i < path.len - 1; i++) {
        delta_pos = (get_position_x() - (float)path.points[i].x / 1000) *
                    (get_position_x() - (float)path.points[i].x / 1000) +
                    (get_position_y() - (float)path.points[i].y / 1000) *
                    (get_position_y() - (float)path.points[i].y / 1000);
        if (i != waypoint_index && delta_pos > old_delta_pos) {
            i--;
            break;
        }
        old_delta_pos = delta_pos;
    }
    //if (i == path.len) {
    //    OSSemPost(mutex);
    //    return NULL;
    //}
    printf("wp nb %d (%d): %d %d\n", i, path.len, path.points[i].x, path.points[i].y);
    next_waypoint.x = (float)path.points[i].x / 1000;
    next_waypoint.y = (float)path.points[i].y / 1000;
    next_waypoint.vx = (float)path.points[i].vx / 1000;
    next_waypoint.vy = (float)path.points[i].vy / 1000;

    waypoint_index = i;

    OSSemPost(mutex);

    return &next_waypoint;

}

static int send_request()
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
    request.start.x = get_position_x() * 1000;
    request.start.y = get_position_y() * 1000;
    request.start.vx = get_velocity_x() * 1000;
    request.start.vy = get_velocity_y() * 1000;

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

        // int i;
        // for (i = 0; i < path.len; i++) {
        //     printf("   [%d] %d %d %d %d\n", path.points[i].timestamp,
        //         path.points[i].x, path.points[i].y,
        //         path.points[i].vx,path.points[i].vy);
        // }
        // printf("Path len: %d\n", path.len);

        waypoint_index = 0;

        request_time = request_started;

        OSSemPost(mutex);
    } else {
        printf("pius err %d\n", err);
    }

    obstacle_avoidance_request_delete(&request);
    return err;
}

void drive_waypoint_task(void *arg)
{
    while(42) {
        printf("new req: %d %d\n", destination.x, destination.y);
        int err = send_request();
        OSTimeDly(OS_TICKS_PER_SEC*0.5);
        if (err != ERR_OK) {
            OSTimeDly(OS_TICKS_PER_SEC/20);
        }
    }
}

