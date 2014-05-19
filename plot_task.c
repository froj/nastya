
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucos_ii.h>
#include <lwip/udp.h>
#include <errno.h>
#include <param.h>
#include "tasks.h"
#include "plot_task.h"

#define PLOT_FREQ 1 // [Hz]
param_t plot_freq;

OS_STK    plot_stk[PLOT_TASK_STACKSIZE];

struct plot_data{
    char description[8];
    void* variable;
    enum plot_types type;
    struct plot_data *next;
};

static struct plot_data *plots = NULL;
struct udp_pcb *pcb;


void plot_task(void *arg)
{
    uint32_t plot_period = 0;
    int i;
    char plot_string[1024];
    char* pos;

    static struct sockaddr_in* client_addr = NULL;

    printf("plot task\n");
//    if (client_addr == NULL) {
//        //printf("clieent_addr == NULL\n");
//        char recv_data[1024];
//        socklen_t addr_len;
//        addr_len = sizeof(struct sockaddr);
//        client_addr = malloc(sizeof(struct sockaddr_in));
//        int ret = recvfrom(   sock, recv_data, 1024, 0,
//                    (struct sockaddr *)&client_addr, &addr_len);
//        //printf("udp received (%d, errno = %d, sock = %d)\n", ret, errno, sock);
//    }

    struct plot_data *to_plot;
    while (23) {
        if (param_has_changed(&plot_freq))
            plot_period = (int64_t)OS_TICKS_PER_SEC / param_get(&plot_freq);

        pos = plot_string;
        to_plot = plots;
        while (to_plot != NULL) {
            switch(to_plot->type) {
                case PLOT_INT8 :
                    pos += sprintf( pos, "%s %d\n", to_plot->description,
                                    *(int8_t*)to_plot->variable);
                    break;
                case PLOT_INT16 :
                    pos += sprintf( pos, "%s %d\n", to_plot->description,
                                    *(int16_t*)to_plot->variable);
                    break;
                case PLOT_INT32 :
                    pos += sprintf( pos, "%s %d\n", to_plot->description,
                                    *(int32_t*)to_plot->variable);
                    break;
                case PLOT_UINT8 :
                    pos += sprintf( pos, "%s %d\n", to_plot->description,
                                    *(uint8_t*)to_plot->variable);
                    break;
                case PLOT_UINT16 :
                    pos += sprintf( pos, "%s %d\n", to_plot->description,
                                    *(uint16_t*)to_plot->variable);
                    break;
                case PLOT_UINT32 :
                    pos += sprintf( pos, "%s %d\n", to_plot->description,
                                    *(uint32_t*)to_plot->variable);
                    break;
                case PLOT_FLOAT :
                    pos += sprintf( pos, "%s %f\n", to_plot->description,
                                    *(float*)to_plot->variable);
                    break;
                case PLOT_DOUBLE :
                    pos += sprintf( pos, "%s %lf\n", to_plot->description,
                                    *(double*)to_plot->variable);
                    break;
            }
            //pos--;  // fuck that null terminator
            if (pos - plot_string > 1000) break;    // prevent segfaults

            to_plot = to_plot->next;
        }

        struct pbuf *p;
        p = pbuf_alloc(PBUF_TRANSPORT, strlen(plot_string), PBUF_REF);
        p->payload = plot_string;

        udp_send(pcb, p);

        //printf("before sendto %s\n", plot_string);

        //int ret = sendto( sock, plot_string, strlen(plot_string), 0,
        //        (struct sockaddr *)&client_addr, sizeof(struct sockaddr));

        //printf("sendto ret %d, errno = %d\n", ret, errno);
        OSTimeDly(plot_period);
    }
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
    memcpy(plot->next->description, description, 8);
    plot->next->variable = variable;
    plot->next->type = type;
    plot->next->next = NULL;
}


void plot_init(void)
{

    param_add(&plot_freq, "plot freq", "UDP plot freq");
    param_set(&plot_freq, PLOT_FREQ);

//    int addr_len, bytes_read;
//    struct sockaddr_in server_addr , client_addr;

    ip_addr_t ipaddr;

    /* initliaze IP addresses to be used */
    IP4_ADDR(&ipaddr,  192, 168,   1, 201);

    /* start the UDP server */
    //------------------------
    err_t err;
    unsigned port = 4000;
    unsigned pc_port = 5000;

    /* create new UDP PCB structure */
    pcb = udp_new();
    err = udp_bind(pcb, IP_ADDR_ANY, port);
    err = udp_connect(pcb, &ipaddr, pc_port);


    //if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    //    printf("plot_init() sock fail, errno = %d\n", errno);
    //    // error
    //}

    //server_addr.sin_family = AF_INET;
    //server_addr.sin_port = htons(5000);
    //server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //memset(&(server_addr.sin_zero), 0, 8);

    //client_addr.sin_family = AF_INET;
    //client_addr.sin_port = htons(5000);
    //inet_aton("192.168.3.222", &client_addr.sin_addr);
    //memset(&(client_addr.sin_zero), 0, 8);

    //if (bind(sock,(struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
    //    // error
    //}

    //char *msg = "hello"
    //sendto( sock, msg, strlen(msg), 0,
    //            (struct sockaddr *)&client_addr, sizeof(struct sockaddr));
    //printf("udp sent\n");

    OSTaskCreateExt(plot_task,
                    NULL,
                    &plot_stk[PLOT_TASK_STACKSIZE-1],
                    PLOT_TASK_PRIORITY,
                    PLOT_TASK_PRIORITY,
                    &plot_stk[0],
                    PLOT_TASK_STACKSIZE,
                    NULL, 0);

}
