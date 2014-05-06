
#include <stdio.h>
#include <stdlib.h>
#include <ucos_ii.h>
#include "control.h"
#include "tasks.h"

#include "drive.h"

OS_STK    drive_task_stk[DRIVE_TASK_STACKSIZE];


void drive_task(void *pdata)
{
    printf("drive task started\n");

}


void start_drive_task(void)
{
    OSTaskCreateExt(drive_task,
                    NULL,
                    &drive_task_stk[DRIVE_TASK_STACKSIZE-1],
                    DRIVE_TASK_PRIORITY,
                    DRIVE_TASK_PRIORITY,
                    &drive_task_stk[0],
                    DRIVE_TASK_STACKSIZE,
                    NULL, 0);
}

