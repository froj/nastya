
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucos_ii.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include "tasks.h"
#include "plot_task.h"

struct plot_data{
    char description[8];
    void* variable;
    enum plot_types type;
    struct plot_data *next;
};

static struct plot_data *plots = NULL;
static int sock = 0;
OS_STK plot_stk[PLOT_TASK_STACKSIZE];


void plot_task(void)
{
    int i;
    char plot_string[1024];
    char* pos;

    static struct sockaddr_in* client_addr = NULL;


    if (client_addr == NULL) {
        char recv_data[1024];
        socklen_t addr_len;
        addr_len = sizeof(struct sockaddr);
        client_addr = malloc(sizeof(struct sockaddr_in));
        recvfrom(   sock, recv_data, 1024, 0,
                    (struct sockaddr *)&client_addr, &addr_len);
    }

    pos = plot_string;

    struct plot_data *to_plot;
    to_plot = plots;
    while (to_plot != NULL) {
        switch(to_plot->type) {
            case INT8 :
                pos += sprintf( pos, "%s %d\n", to_plot->description,
                                *(int8_t*)to_plot->variable);
                break;
            case INT16 :
                pos += sprintf( pos, "%s %d\n", to_plot->description,
                                *(int16_t*)to_plot->variable);
                break;
            case INT32 :
                pos += sprintf( pos, "%s %d\n", to_plot->description,
                                *(int32_t*)to_plot->variable);
                break;
            case UINT8 :
                pos += sprintf( pos, "%s %d\n", to_plot->description,
                                *(uint8_t*)to_plot->variable);
                break;
            case UINT16 :
                pos += sprintf( pos, "%s %d\n", to_plot->description,
                                *(uint16_t*)to_plot->variable);
                break;
            case UINT32 :
                pos += sprintf( pos, "%s %d\n", to_plot->description,
                                *(uint32_t*)to_plot->variable);
                break;
            case FLOAT :
                pos += sprintf( pos, "%s %f\n", to_plot->description,
                                *(float*)to_plot->variable);
                break;
            case DOUBLE :
                pos += sprintf( pos, "%s %lf\n", to_plot->description,
                                *(double*)to_plot->variable);
                break;
        }
        pos--;  // fuck that null terminator
        if (pos - plot_string > 1000) break;    // prevent segfaults

        to_plot = to_plot->next;
    }


    sendto( sock, plot_string, strlen(plot_string), 0,
            (struct sockaddr *)&client_addr, sizeof(struct sockaddr));

    OSTimeDly((int64_t)OS_TICKS_PER_SEC/50);
}


void plot_add_variable(char description[8], void* variable, enum plot_types type)
{
    static struct plot_data *plot;
    plot = plots;

    if (plot == NULL) {
        plots = malloc(sizeof(struct plot_data));
        memcpy(plots->description, description, 8);
        plots->variable = variable;
        plots->type = type;
        plots->next = NULL;
        return;
    }

    while (plot->next != NULL) {
        plot = plot->next;
    }

    plot->next = malloc(sizeof(struct plot_data));
    memcpy(plots->description, description, 8);
    plot->next->variable = variable;
    plot->next->type = type;
    plot->next->next = NULL;
}


void plot_init(void)
{
    int addr_len, bytes_read;
    struct sockaddr_in server_addr , client_addr;


    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        // error
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(5000);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(&(server_addr.sin_zero), 0, 8);


    if (bind(sock,(struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
        // error
    }
}
