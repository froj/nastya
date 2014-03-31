#include <sys/socket.h>
#include <lwip/inet.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "tasks.h"
#include <ucos_ii.h>
#include "control.h"

#include "pid_config.h"

OS_STK pid_conf_task_stk[PID_CONF_TASK_STACKSIZE];
static int listen_port;

void pid_conf_shell(int socket)
{
    static char rcv_buf[128];
    // printf("new connection\n");
    int n;
    struct pid_filter *pid = &nastya_cs.vx_pid;
    char param = 'p';
    char *pid_name = "pid x";
    while ((n = read(socket, rcv_buf, sizeof(rcv_buf)-1)) > 0) {
        rcv_buf[n] = '\0';
        static char cmd[80];
        sscanf(rcv_buf, "%79s", cmd);
        char w_buf[1024];
        int kp = pid_get_gain_P(pid);
        int ki = pid_get_gain_I(pid);
        int kd = pid_get_gain_D(pid);
        int max_in = pid_get_max_in(pid);
        int max_i = pid_get_max_I(pid);
        int max_out =pid_get_max_out(pid);
        if (strcmp(cmd, "pid") == 0) {
            sscanf(rcv_buf, "%*s%79s", cmd);
            if (strcmp(cmd, "x") == 0) {
                pid = &nastya_cs.vx_pid;
                pid_name = "pid x";
                snprintf(w_buf, sizeof(w_buf)-1, "pid: x\n");
            } else if (strcmp(cmd, "y") == 0) {
                pid = &nastya_cs.vy_pid;
                pid_name = "pid y";
                snprintf(w_buf, sizeof(w_buf)-1, "pid: y\n");
            } else if (strcmp(cmd, "r") == 0) {
                pid = &nastya_cs.omega_pid;
                pid_name = "pid omega";
                snprintf(w_buf, sizeof(w_buf)-1, "pid: r\n");
            } else {
                snprintf(w_buf, sizeof(w_buf)-1, "invalid pid [x,y,r]\n");
            }
        } else if (strcmp(cmd, "p") == 0) {
            int x;
            if (sscanf(rcv_buf, "%*s%d", &x) == 1)
                kp = x;
            snprintf(w_buf, sizeof(w_buf)-1, "%s: kp %d ki %d kd %d int_bound %d\n", pid_name, kp, ki, kd, max_i);
        } else if (strcmp(cmd, "i") == 0) {
            int x;
            if (sscanf(rcv_buf, "%*s%d", &x) == 1)
                ki = x;
            snprintf(w_buf, sizeof(w_buf)-1, "%s: kp %d ki %d kd %d int_bound %d\n", pid_name, kp, ki, kd, max_i);
        } else if (strcmp(cmd, "d") == 0) {
            int x;
            if (sscanf(rcv_buf, "%*s%d", &x) == 1)
                kd = x;
            snprintf(w_buf, sizeof(w_buf)-1, "%s: kp %d ki %d kd %d int_bound %d\n", pid_name, kp, ki, kd, max_i);
        } else if (strcmp(cmd, "b") == 0) {
            int x;
            if (sscanf(rcv_buf, "%*s%d", &x) == 1)
                max_i = x;
            snprintf(w_buf, sizeof(w_buf)-1, "%s: kp %d ki %d kd %d int_bound %d\n", pid_name, kp, ki, kd, max_i);
        } else if (strcmp(cmd, "+") == 0) {
        } else if (strcmp(cmd, "-") == 0) {
        } else {
            snprintf(w_buf, sizeof(w_buf)-1, "unknown command: %s\n", rcv_buf);
        }
        pid_set_maximums(pid, max_in, max_i, max_out);
        pid_set_gains(pid, kp, ki, kd);
        write(socket, w_buf, strlen(w_buf));
        write(socket, rcv_buf, n);
    }
    close(socket);
}

void pid_conf_shell_listen(void)
{
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serv_addr;
    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(listen_port);

    int yes = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    listen(listenfd, 1);

    while (1) {
        int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
        // printf("%d %d\n", connfd, errno);
        printf("pid config shell connect\n");
        pid_conf_shell(connfd);
        getc(stdin);
    }
}


void pid_conf_task(void *arg)
{
    printf("pid config shell started (port %d)\n", listen_port);
    pid_conf_shell_listen();
}

void start_pid_conf_shell(int port)
{
    listen_port = port;
    OSTaskCreateExt(pid_conf_task,
                    NULL,
                    &pid_conf_task_stk[PID_CONF_TASK_STACKSIZE-1],
                    PID_CONF_TASK_PRIORITY,
                    PID_CONF_TASK_PRIORITY,
                    &pid_conf_task_stk[0],
                    PID_CONF_TASK_STACKSIZE,
                    NULL, NULL);
}

