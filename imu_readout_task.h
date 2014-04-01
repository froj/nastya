#ifndef IMU_READOUT_TASK_H
#define IMU_READOUT_TASK_H

#include <lwip/api.h>

void imu_readout_init(void);
void imu_readout_start(void);
void imu_readout_stop(void);
void imu_readout_send(struct netconn *conn);


#endif // IMU_READOUT_TASK_H
