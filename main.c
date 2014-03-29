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

#include <stdio.h>
#include <stdarg.h>
#include <uptime.h>
#include <string.h>


/* nios2.h contient toutes les fonctions dont nous avons besoin pour dialoguer
 * avec le nios alt_irq_register, IORD, etc... */
#ifdef COMPILE_ON_ROBOT
#include <ucos_ii.h>
#include <nios2.h>
#else
#error "add COMPILE_ON_ROBOT to defines"
#endif

#include "hardware.h"

#include "tasks.h"


OS_STK    heartbeat_task_stk[HEARTBEAT_TASK_STACKSIZE];

void heartbeat_task(void *pdata);


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


void create_heartbeat_task(void)
{
    OSTaskCreateExt(heartbeat_task,
                    NULL,
                    &heartbeat_task_stk[HEARTBEAT_TASK_STACKSIZE-1],
                    HEARTBEAT_TASK_PRIORITY,
                    HEARTBEAT_TASK_PRIORITY,
                    &heartbeat_task_stk[0],
                    HEARTBEAT_TASK_STACKSIZE,
                    NULL, NULL);
}

void heartbeat_task(void *pdata)
{
    OS_CPU_SR cpu_sr;
    int leds;
    while(1) {
        OS_ENTER_CRITICAL();
        leds = IORD(LED_BASE, 0);
        leds = leds ^ (1<<0); // toggle bit 0.
        IOWR(LED_BASE, 0, leds);
        OS_EXIT_CRITICAL();
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

int main(__attribute__((unused)) int argc, __attribute__((unused)) char **argv)
{

    /* Setup UART speed, must be first. */
    hw_set_uart_speed(COMBT2_BASE, 57600);
    hw_set_uart_speed(COMBT1_BASE, 57600);
    hw_set_uart_speed(COMPC_BASE, 57600);
    hw_set_uart_speed(COMBEACON_BASE, 57600);

    /* Inits the logging system. */
    error_register_emerg(mylog);
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);

    OSInit();
    create_init_task();
    create_heartbeat_task();
    OSStart(); // never returns

    while (1);
}

