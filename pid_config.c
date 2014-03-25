#include <sys/socket.h>
#include <lwip/inet.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "control.h"

void pid_conf_shell(int socket)
{
    static char rcv_buf[128];
    // printf("new connection\n");
    int n;
    pid_t *pid = &pid_x;
    while ((n = read(socket, rcv_buf, sizeof(rcv_buf)-1)) > 0) {
        rcv_buf[n] = '\0';
        static char cmd[80];
        sscanf(rcv_buf, "%79s", cmd);
        char w_buf[1024];
        if (strcmp(cmd, "pid") == 0) {
            sscanf(rcv_buf, "%*s%79s", cmd);
            if (strcmp(cmd, "x") == 0) {
                pid = &pid_x;
                snprintf(w_buf, sizeof(w_buf)-1, "pid: x\n");
            } else if (strcmp(cmd, "y") == 0) {
                pid = &pid_y;
                snprintf(w_buf, sizeof(w_buf)-1, "pid: y\n");
            } else if (strcmp(cmd, "r") == 0) {
                pid = &pid_r;
                snprintf(w_buf, sizeof(w_buf)-1, "pid: r\n");
            } else {
                snprintf(w_buf, sizeof(w_buf)-1, "invalid pid [x,y,r]\n");
            }
        } else if (strcmp(cmd, "p") == 0) {
            float x;
            if (sscanf(rcv_buf, "%*s%f", &x) == 1)
                pid->kp = x;
            snprintf(w_buf, sizeof(w_buf)-1, "p %f i %f d %f intbound %f\n", pid->kp, pid->ki, pid->kd, pid->integration_bound);
        } else if (strcmp(cmd, "i") == 0) {
            float x;
            if (sscanf(rcv_buf, "%*s%f", &x) == 1)
                pid->ki = x;
            snprintf(w_buf, sizeof(w_buf)-1, "p %f i %f d %f intbound %f\n", pid->kp, pid->ki, pid->kd, pid->integration_bound);
        } else if (strcmp(cmd, "d") == 0) {
            float x;
            if (sscanf(rcv_buf, "%*s%f", &x) == 1)
                pid->kd = x;
            snprintf(w_buf, sizeof(w_buf)-1, "p %f i %f d %f intbound %f\n", pid->kp, pid->ki, pid->kd, pid->integration_bound);
        } else if (strcmp(cmd, "b") == 0) {
            float x;
            if (sscanf(rcv_buf, "%*s%f", &x) == 1)
                pid->integration_bound = x;
            snprintf(w_buf, sizeof(w_buf)-1, "p %f i %f d %f intbound %f\n", pid->kp, pid->ki, pid->kd, pid->integration_bound);
        } else {
            snprintf(w_buf, sizeof(w_buf)-1, "unknown command: %s\n", rcv_buf);
        }
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
    serv_addr.sin_port = htons(1337);

    int yes = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    listen(listenfd, 1);

    while (1) {
        int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
        // printf("%d %d\n", connfd, errno);
        pid_conf_shell(connfd);
        getc(stdin);
    }
}
