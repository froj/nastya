
#include <ucos_ii.h>
#include <nios2.h>

#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/ip.h>
#include <netif/slipif.h>

#include "control.h"

#include "plot_task.h"

#include "drive.h"

#include "encoder_readout_task.h"

#include "tasks.h"


OS_STK init_task_stk[INIT_TASK_STACKSIZE];


//
// IP stack init
//

/** Shared semaphore to signal when lwIP init is done. */
sys_sem_t lwip_init_done;

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
    static struct netif slipf;

#ifdef __unix__
    /* for some reason 192.168.4.9 fails on my test station. */
    IP4_ADDR(&gw, 10,0,0,1);
    IP4_ADDR(&ipaddr, 10,0,0,2);
    IP4_ADDR(&netmask, 255,255,255,0);
#else
    IP4_ADDR(&gw, 10,0,0,1);
    IP4_ADDR(&ipaddr, 10,0,0,20);
    IP4_ADDR(&netmask, 255,255,255,255);
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
    printf("Listing network interfaces...\n");
    list_netifs();
}




void init_task(void *pdata)
{
    NOTICE(0, "System boot !");

    control_init();

    ip_stack_init();

    plot_init();

    luaconsole_init();

    start_position_integration();

    start_pid_conf_shell(1337);

    start_drive_task();

    //ready_for_match();

    OSTaskDel(INIT_TASK_PRIORITY);
}


void create_init_task(void)
{
    OSTaskCreateExt(init_task,
                    NULL,
                    &init_task_stk[INIT_TASK_STACKSIZE-1],
                    INIT_TASK_PRIORITY,
                    INIT_TASK_PRIORITY,
                    &init_task_stk[0],
                    INIT_TASK_STACKSIZE,
                    NULL, 0);
}

