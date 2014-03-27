/** @file main.c
 * @author Antoine Albertelli
 * @author Florian "Flopy" Glardon
 * @date 2011
 * @brief Le fichier principal du programme.
 *
 * Ce fichier contient uniquement les fonctions de bases du code. Il se charge
 * de l'init des differents systemes et d'appeller le scheduling toutes les ms.
 */

#include <aversive.h>

#include <error.h>

#include <cvra_dc.h>
#include <stdio.h>
#include <stdarg.h>
#include <uptime.h>
#include <string.h>
#include <commandline.h>


/* nios2.h contient toutes les fonctions dont nous avons besoin pour dialoguer
 * avec le nios alt_irq_register, IORD, etc... */
#ifdef COMPILE_ON_ROBOT
#include <ucos_ii.h>
#include <nios2.h>
#else
#error "add COMPILE_ON_ROBOT to defines"
#endif


#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/ip.h>

//#include "sntp.h"

#include <control.h>
#include "drive_open_loop_dynamic_path.h"

#include <netif/slipif.h>

#include "hardware.h"
//#include "cvra_cs.h"

#include "plot_task.h"

#include "tasks.h"

extern command_t commands_list[];

OS_STK    shell_task_stk[SHELL_TASK_STACKSIZE];
OS_STK    heartbeat_task_stk[HEARTBEAT_TASK_STACKSIZE];
OS_STK    init_task_stk[INIT_TASK_STACKSIZE];
OS_STK    plot_stk[PLOT_TASK_STACKSIZE];
OS_STK    drive_task_stk[DRIVE_TASK_STACKSIZE];


void shell_task(void *pdata);
void heartbeat_task(void *pdata);
void init_task(void *pdata);


/** Shared semaphore to signal when lwIP init is done. */
sys_sem_t lwip_init_done;

/** Serial net interface */
struct netif slipf;


void list_netifs(void)
{
    struct netif *n; /* used for iteration. */
    for(n=netif_list;n !=NULL;n=n->next) {
        /* Converts the IP adress to a human readable format. */
        char buf[16+1];
        ipaddr_ntoa_r(&n->ip_addr, buf, 17);
        printf("%s%d: %s\n", n->name, n->num, buf);
    }
}

/** @rief Callback for lwIP init completion.
 *
 * This callback is automatically called from the lwIP thread after the
 * initialization is complete. It must then tell the main init task that it
 * can proceed. To do thism we use a semaphore that is posted from the lwIP
 * thread and on which the main init task is pending. */
void ipinit_done_cb(void *a)
{
    sys_sem_signal(&lwip_init_done);
}


/** @brief Inits the IP stack and the network interfaces.
 *
 * This function is responsible for the following :
 * 1. Initialize the lwIP library.
 * 2. Wait for lwIP init to be complete.
 * 3. Create the SLIP interface and give it a static adress/netmask.
 * 4. Set the SLIP interface as default and create a gateway.
 * 5. List all network interfaces and their settings, for debug purposes.
 */
void ip_stack_init(void) {
    /* Netif configuration */
    static ip_addr_t ipaddr, netmask, gw;

#ifdef __unix__
    /* for some reason 192.168.4.9 fails on my test station. */
    IP4_ADDR(&gw, 10,0,0,1);
    IP4_ADDR(&ipaddr, 10,0,0,2);
    IP4_ADDR(&netmask, 255,255,255,0);
#else
    IP4_ADDR(&gw, 192,168,0,1);
    IP4_ADDR(&ipaddr, 192,168,4,9);
    IP4_ADDR(&netmask, 255,255,255,0);
#endif

    /* Creates the "Init done" semaphore. */
    sys_sem_new(&lwip_init_done, 0);

    /* We start the init of the IP stack. */
    tcpip_init(ipinit_done_cb, NULL);

    /* We wait for the IP stack to be fully initialized. */
    printf("Waiting for LWIP init...\n");
    sys_sem_wait(&lwip_init_done);

    /* Deletes the init done semaphore. */
    sys_sem_free(&lwip_init_done);
    printf("LWIP init complete\n");


#ifdef __unix__
    /* Adds a tap pseudo interface for unix debugging. */
    netif_add(&slipf,&ipaddr, &netmask, &gw, NULL, tapif_init, tcpip_input);
#else
    /* Adds the serial interface to the list of network interfaces and makes it the default route. */
    netif_add(&slipf, &ipaddr, &netmask, &gw, NULL, slipif_init, tcpip_input);
#endif

    netif_set_default(&slipf);
    netif_set_up(&slipf);

}


/** Logs an event.
 *
 * This function is never called directly, but instead, the error
 * modules fills an error structure and calls it.
 * @param [in] e The error structure, filled with every needed info.
 */
void mylog(struct error * e, ...)
{
    va_list ap;
    va_start(ap, e);
    /* Prints the filename (not the full path) and line number. */
    fprintf(stderr, "%s:%d ", strrchr(e->file, '/') ? strrchr(e->file, '/')+1:e->file, e->line);
    vfprintf(stderr, e->text, ap);
    fprintf(stderr, "\r\n");
    va_end(ap);
}


void shell_task(void *pdata)
{
    /* Inits the commandline interface. */
    commandline_init(commands_list);

    /* Runs the commandline system. */
    for(;;) commandline_input_char(getchar());
}

#define MAX_NB_WP 1024
struct dynamic_waypoint wp[MAX_NB_WP];
int nb_wp;

sys_sem_t dyn_path_exec;
void udp_get_dynamic_path_rcv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    ip_addr_t *addr, u16_t port)
{
    if (nb_wp < MAX_NB_WP) {
        char cmd;
        float vx, vy, omega;
        int timest;
        sscanf(p->payload, "%c %f %f %f %d", &cmd, &vx, &vy, &omega, &timest);
        printf("pkt %c\n", cmd);
        if (cmd == 'D') {
            // data
            printf("timest %d\n", timest);
            static int prev_timest;
            if (nb_wp > 0) {
                int dur = timest - prev_timest;
                printf("dur %d\n", dur);
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
            printf("%d pkts\n", nb_wp);
        }
        if (cmd == 'X') {
            // execute
            sys_sem_signal(&dyn_path_exec);
        }
    } else {
        sys_sem_signal(&dyn_path_exec);
    }
FAIL:
    pbuf_free(p);
}

void udp_get_dynamic_path(void)
{
    unsigned port = 2000;
    err_t err;
    struct udp_pcb *pcb;
    pcb = udp_new();
    err = udp_bind(pcb, IP_ADDR_ANY, port);
    sys_sem_new(&dyn_path_exec, 0);
    while (1) {
        nb_wp = 0;
        udp_recv(pcb, udp_get_dynamic_path_rcv_cb, NULL);
        sys_sem_wait(&dyn_path_exec);
        udp_recv(pcb, NULL, NULL);
        int i;
        for (i=0; i < nb_wp; i++) {
            printf("%f %f %f %d\n", wp[i].vx, wp[i].vy, wp[i].omega, wp[i].duration);
        }
        drive_open_loop_dynamic_path(wp, nb_wp);
        control_update_setpoint_vx(0);
        control_update_setpoint_vy(0);
        control_update_setpoint_omega(0);
    }
    udp_remove(pcb);
}




void drive_task(void *pdata)
{
    printf("drive task\n");
    udp_get_dynamic_path();
    while (0) {
        control_update_setpoint_vx(0.1);
        OSTimeDlyHMSM(0, 0, 10, 0);
        control_update_setpoint_vx(-0.1);
        OSTimeDlyHMSM(0, 0, 10, 0); 
    }
    while (1) {
        control_update_setpoint_vx(0.8);
        OSTimeDlyHMSM(0, 0, 4, 0);
        control_update_setpoint_vy(0.8);
        OSTimeDlyHMSM(0, 0, 4, 0);
        control_update_setpoint_vx(-0.8);
        OSTimeDlyHMSM(0, 0, 4, 0);
        control_update_setpoint_vy(-0.8);
        OSTimeDlyHMSM(0, 0, 4, 0); 
    }
    
    control_update_setpoint_vx(0.1);
    OSTimeDlyHMSM(1, 0, 10, 0);
    control_update_setpoint_vx(0);
    OSTimeDlyHMSM(1, 0, 3, 0);
}


extern float wheel_cmd[3];
extern float y_x;

void heartbeat_task(void *pdata)
{
    printf("starting heartbeat task\n");
    OS_CPU_SR cpu_sr;
    int leds;
    while(1) {
        OS_ENTER_CRITICAL();

        leds = IORD(LED_BASE, 0);
        /* toggles bit 0. */
        leds = leds ^ (1<<0);
        IOWR(LED_BASE, 0, leds);
        //printf("%d %d %d\n",  -cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE),  -cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE),  -cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE));
        //printf("%f %f %f\n", wheel_cmd[0], wheel_cmd[1], wheel_cmd[2]);
        //printf("%f\n", y_x);
        OS_EXIT_CRITICAL();
        OSTimeDlyHMSM(0, 0, 0, 500);
    }

}


void init_task(void *pdata)
{
    NOTICE(0, "System boot !");

    control_init();

    // OSTaskCreateExt(shell_task,
    //                 NULL,
    //                 &shell_task_stk[SHELL_TASK_STACKSIZE-1],
    //                 SHELL_TASK_PRIORITY,
    //                 SHELL_TASK_PRIORITY,
    //                 &shell_task_stk[0],
    //                 SHELL_TASK_STACKSIZE,
    //                 NULL, NULL);

    OSTaskCreateExt(drive_task,
                    NULL,
                    &drive_task_stk[DRIVE_TASK_STACKSIZE-1],
                    DRIVE_TASK_PRIORITY,
                    DRIVE_TASK_PRIORITY,
                    &drive_task_stk[0],
                    DRIVE_TASK_STACKSIZE,
                    NULL, NULL);

    OSTaskCreateExt(heartbeat_task,
                    NULL,
                    &heartbeat_task_stk[HEARTBEAT_TASK_STACKSIZE-1],
                    HEARTBEAT_TASK_PRIORITY,
                    HEARTBEAT_TASK_PRIORITY,
                    &heartbeat_task_stk[0],
                    HEARTBEAT_TASK_STACKSIZE,
                    NULL, NULL);

    ip_stack_init();

    /* Lists every network interface and shows its IP. */
    printf("Listing network interfaces...\n");
    list_netifs();

    plot_init();

    OSTaskCreateExt(plot_task,
                    NULL,
                    &plot_stk[PLOT_TASK_STACKSIZE-1],
                    PLOT_TASK_PRIORITY,
                    PLOT_TASK_PRIORITY,
                    &plot_stk[0],
                    PLOT_TASK_STACKSIZE,
                    NULL, NULL);

    /* Creates a simple demo app. */
//    ping_init();


    // getchar();
    // sntp_init();

    pid_conf_shell_listen();

    /* We delete the init task before returning. */
    OSTaskDel(INIT_TASK_PRIORITY);
}


int main(__attribute__((unused)) int argc, __attribute__((unused)) char **argv)
{

    /* Setup UART speed, must be first. */
    cvra_set_uart_speed(COMBT2_BASE, 57600);
    cvra_set_uart_speed(COMBT1_BASE, 57600);
    cvra_set_uart_speed(COMPC_BASE, 57600);
    cvra_set_uart_speed(COMBEACON_BASE, 57600);

    /* Inits the logging system. */
    error_register_emerg(mylog);
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);

    OSInit();

    OSTaskCreateExt(init_task,
                    NULL,
                    &init_task_stk[INIT_TASK_STACKSIZE-1],
                    INIT_TASK_PRIORITY,
                    INIT_TASK_PRIORITY,
                    &init_task_stk[0],
                    INIT_TASK_STACKSIZE,
                    NULL, NULL);

    NOTICE(0, "Starting OS");

    OSStart();

    for(;;);

    return 0;
}

