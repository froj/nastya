// ucOS II task priorities & stack sizes
// smaller priority number equals higher priority
// NEVER REUSE SAME PRIORITY FOR TWO TASKS
// in current config: max (lowest priority) is 63

#ifndef TASKS_H
#define TASKS_H

#define INIT_TASK_PRIORITY                          1
#define INIT_TASK_STACKSIZE                      2048

#define HEARTBEAT_TASK_PRIORITY                    60
#define HEARTBEAT_TASK_STACKSIZE                 2048

#define PID_CONF_TASK_PRIORITY                     20
#define PID_CONF_TASK_STACKSIZE                  2048

#define ENCODER_TASK_PRIORITY                       2
#define ENCODER_TASK_STACKSIZE                   2048

#define IMU_TASK_PRIORITY                           3
#define IMU_TASK_STACKSIZE                       2048

#define POSITION_INTEGRATION_TASK_PRIORITY          5
#define POSITION_INTEGRATION_TASK_STACKSIZE      2048

#define CONTROL_TASK_PRIORITY                       4
#define CONTROL_TASK_STACKSIZE                   2048

#define PLOT_TASK_PRIORITY                         21
#define PLOT_TASK_STACKSIZE                      2048

#define DRIVE_TASK_PRIORITY                        19
#define DRIVE_TASK_STACKSIZE                     2048

#define MATCH_TASK_PRIORITY                        10
#define MATCH_TASK_STACKSIZE                     2048

#define MATCH_TASK_PRIORITY                         9
#define MATCH_TASK_STACKSIZE                     2048

// IP stack
#define TCPIP_THREAD_PRIO                          30
#define SLIPIF_THREAD_PRIO                         31
#define SNTP_THREAD_PRIO                           34

#endif // TASKS_H
