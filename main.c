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
#include <commandline.h>


/* nios2.h contient toutes les fonctions dont nous avons besoin pour dialoguer
 * avec le nios alt_irq_register, IORD, etc... */
#ifdef COMPILE_ON_ROBOT
#include <ucos_ii.h>
#include <nios2.h>
#else
#error "add COMPILE_ON_ROBOT to defines"
#endif

#include "hardware.h"
#include "cvra_cs.h"

extern command_t commands_list[];

#define   TASK_STACKSIZE          2048
#define   SHELL_TASK_PRIORITY       40
#define   HEARTBEAT_TASK_PRIORITY   5

OS_STK    shell_task_stk[TASK_STACKSIZE];
OS_STK    heartbeat_task_stk[TASK_STACKSIZE];

void shell_task(void *pdata);
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


void shell_task(void *pdata)
{
    /* Inits the commandline interface. */
    commandline_init(commands_list);

    /* Runs the commandline system. */
    for(;;) commandline_input_char(getchar());
}


void heartbeat_task(void *pdata)
{
    OS_CPU_SR cpu_sr;
    int leds;
    while(1) {
        OS_ENTER_CRITICAL();

        leds = IORD(LED_BASE, 0);
        /* toggles bit 0. */
        leds = leds ^ (1<<0);
        IOWR(LED_BASE, 0, leds);

        OS_EXIT_CRITICAL();
        OSTimeDlyHMSM(0, 0, 0, 500);
    }

}

int init(void)
{
    NOTICE(0, "System boot !");
    NOTICE(0, "Main control system init.");
    cvra_cs_init();

    int ret = OSTaskCreateExt(shell_task,
                    NULL,
                    &shell_task_stk[TASK_STACKSIZE-1],
                    SHELL_TASK_PRIORITY,
                    SHELL_TASK_PRIORITY,
                    &shell_task_stk[0],
                    TASK_STACKSIZE,
                    NULL, NULL);
    printf("create err = %d\n", ret);

    OSTaskCreateExt(heartbeat_task,
                    NULL,
                    &heartbeat_task_stk[TASK_STACKSIZE-1],
                    HEARTBEAT_TASK_PRIORITY,
                    HEARTBEAT_TASK_PRIORITY,
                    &heartbeat_task_stk[0],
                    TASK_STACKSIZE,
                    NULL, NULL);

}


int main(__attribute__((unused)) int argc, __attribute__((unused)) char **argv)
{
    robot.verbosity_level = ERROR_SEVERITY_NOTICE;

    /* Setup UART speed, must be first. */
    cvra_set_uart_speed(COMBT1_BASE, 9600);
    cvra_set_uart_speed(COMBT2_BASE, 9600);

    /* Inits the logging system. */
    error_register_emerg(mylog);
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);

    OSInit();

    init();
    NOTICE(0, "Starting OS");

    OSStart();

    for(;;);

    return 0;
}

