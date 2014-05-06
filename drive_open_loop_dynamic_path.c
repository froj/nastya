
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <uptime.h>
#include "control.h"
#include <lwip/udp.h>
#include <ucos_ii.h>

#include "imu_readout_task.h"
#include "encoder_readout_task.h"

#include "drive_open_loop_dynamic_path.h"


#define MAX_NB_WP 1024
static struct dynamic_waypoint wp[MAX_NB_WP];
static int nb_wp;
static int output;

OS_EVENT *dyn_path_exec;
void udp_get_dynamic_path_rcv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    ip_addr_t *addr, u16_t port)
{
    if (nb_wp < MAX_NB_WP) {
        char cmd;
        float vx, vy, omega;
        int timest;
        sscanf(p->payload, "%c %f %f %f %d", &cmd, &vx, &vy, &omega, &timest);
        //printf("pkt %c\n", cmd);
        if (cmd == 'R') {
            nb_wp = 0;
        }
        if (cmd == 'D') {
            // data
            //printf("timest %d\n", timest);
            static int prev_timest;
            if (nb_wp > 0) {
                int dur = timest - prev_timest;
                //printf("dur %d\n", dur);
                if (dur > 0) {
                    wp[nb_wp - 1].duration = dur;
                } else { // current packet too old
                    printf("old packet\n");
                    goto FAIL;
                }
            }
            prev_timest = timest;
            wp[nb_wp].vx = vx;
            wp[nb_wp].vy = vy;
            wp[nb_wp].omega = omega;
            wp[nb_wp].duration = 0;
            nb_wp++;
            //printf("%d pkts\n", nb_wp);
        }
        if (cmd == 'X') {
            // execute
            OSSemPost(dyn_path_exec);
        }
        if (cmd == '*') {
            output = 1;
        }
    } else {
        printf("pkt too long\n");
        OSSemPost(dyn_path_exec);
    }
FAIL:
    pbuf_free(p);
}

void udp_get_dynamic_path(void)
{
    encoder_readout_init();
    imu_readout_init();

    unsigned port = 2000;
    err_t err;
    struct udp_pcb *pcb;
    pcb = udp_new();
    err = udp_bind(pcb, IP_ADDR_ANY, port);
    dyn_path_exec = OSSemCreate(0);

    struct netconn *listen_conn = netconn_new(NETCONN_TCP);
    netconn_bind(listen_conn, NULL, 2000);
    netconn_listen(listen_conn);

    while (1) {
        printf("drive task: waiting for path\n");
        nb_wp = 0;
        output = 0;
        udp_recv(pcb, udp_get_dynamic_path_rcv_cb, NULL);
        INT8U ucErr;
        OSSemPend(dyn_path_exec, 0, &ucErr);
        udp_recv(pcb, NULL, NULL);
        int i;
        for (i=0; i < nb_wp; i++) {
            //printf("%f %f %f %d\n", wp[i].vx, wp[i].vy, wp[i].omega, wp[i].duration);
        }
        printf("path received\n");
        encoder_readout_start();
        imu_readout_start();
        if (output) {
            OSTimeDly(OS_TICKS_PER_SEC * 3);
        }
        drive_open_loop_dynamic_path(wp, nb_wp);
        control_update_setpoint_vx(0);
        control_update_setpoint_vy(0);
        control_update_setpoint_omega(0);
        if (output) {
            OSTimeDly(OS_TICKS_PER_SEC * 3);
        }
        encoder_readout_stop();
        imu_readout_stop();
        if (output) {
            printf("connect to send imu & enc values\n");
            err_t err;
            struct netconn *conn;
            err = netconn_accept(listen_conn, &conn);
            if (err != ERR_OK) {
                printf("connection error\n");
            } else {
                encoder_readout_send(conn);
                netconn_close(conn);
            }
            netconn_delete(conn);
            err = netconn_accept(listen_conn, &conn);
            if (err != ERR_OK) {
                printf("connection error\n");
            } else {
                imu_readout_send(conn);
                netconn_close(conn);
            }
            netconn_delete(conn);
        }
    }
    udp_remove(pcb);
}

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


