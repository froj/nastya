#include <stdio.h>
#include <ucos_ii.h>
#include "tasks.h"
#include <uptime.h>

#include "match.h"

OS_STK match_task_stk[MATCH_TASK_STACKSIZE];

timestamp_t match_start;


void match_task(void *arg)
{
    // wait for start signal
    while (0) OSTimeDly(OS_TICKS_PER_SEC/100);

    match_start = uptime_get();
    printf("match started [%d]\n", match_start);

}


void ready_for_match(void)
{
    OSTaskCreateExt(match_task,
                    NULL,
                    &match_task_stk[MATCH_TASK_STACKSIZE-1],
                    MATCH_TASK_PRIORITY,
                    MATCH_TASK_PRIORITY,
                    &match_task_stk[0],
                    MATCH_TASK_STACKSIZE,
                    NULL, 0);
}
