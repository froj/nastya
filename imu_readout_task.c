#include <stddef.h>
#include <stdbool.h>
#include <cvra_dc.h>
#include <ucos_ii.h>
#include <lwip/api.h>
#include <uptime.h>
#include "tasks.h"
#include "serial_datagram.h"
#include "imu.h"

struct imu_datapoint {
    imu_t imu;
    timestamp_t timestamp;
};

FILE *imu_serial;

#define IMU_BUFFER_SIZE 100000
static struct imu_datapoint imu_buffer[IMU_BUFFER_SIZE];
static int imu_buffer_index = 0;
static bool imu_buffer_en = false;
static OS_EVENT *imu_buffer_mutex;

OS_STK imu_readout_stk[IMU_TASK_STACKSIZE];

static void callback_fn(uint8_t *pkg, int len)
{
    timestamp_t now = uptime_get();
    static char buf[100];
    imu_t *imu = imu_deserialize(pkg, buf);
    INT8U err;
    OSMutexPend(imu_buffer_mutex, 0, &err);
    if (imu_buffer_index < IMU_BUFFER_SIZE && imu_buffer_en) {
        imu_buffer[imu_buffer_index].imu = *imu;
        imu_buffer[imu_buffer_index].timestamp = now;
        imu_buffer_index++;
    }
    OSMutexPost(imu_buffer_mutex);
    // printf("%d  acc: %f %f %f gyro: %f %f %f\n", imu_buffer_index, imu->acc_x, imu->acc_y, imu->acc_z, 
    //     imu->gyro_x, imu->gyro_y, imu->gyro_z);

}

void imu_readout_task(void *pdata)
{
    printf("imu readout task started\n");
    static char pkt_buffer[100];
    serial_datagram_rcv_buffer_t rcv_buf;
    serial_datagram_rcv_buffer_init(&rcv_buf, pkt_buffer, sizeof(pkt_buffer), callback_fn);
    while (1) {
        static char inbuf[16];
        int nb_bytes = fread(inbuf, sizeof(char), sizeof(inbuf), imu_serial);
        serial_datagram_rcv(&rcv_buf, inbuf, nb_bytes);
        // printf("imu bytes read %d\n", nb_bytes);
        // int c = getc(imu_serial);
        // serial_datagram_rcv(&rcv_buf, &c, 1);
    }
}

void imu_readout_start(void)
{
    INT8U err;
    OSMutexPend(imu_buffer_mutex, 0, &err);
    imu_buffer_index = 0;
    imu_buffer_en = true;
    OSMutexPost(imu_buffer_mutex);

}

void imu_readout_stop(void)
{
    INT8U err;
    OSMutexPend(imu_buffer_mutex, 0, &err);
    imu_buffer_en = false;
    OSMutexPost(imu_buffer_mutex);
}


void imu_readout_send(struct netconn *conn)
{
    printf("%d imu values read\n", imu_buffer_index);
    int i;
    printf("< imu readout\n");
    static char sendbuf[300];
    // sprintf(sendbuf, "timestamp, accx, accy, accz, gyrox, gyroy, gyroz", imu_buffer_index);
    // netconn_write(conn, sendbuf, strlen(sendbuf), NETCONN_COPY);
    for (i = 0; i < imu_buffer_index - 1; i++) {
        snprintf(sendbuf, sizeof(sendbuf), "%d, %f, %f, %f, %f, %f, %f\n",
            imu_buffer[i].timestamp,
            imu_buffer[i].imu.acc_x, imu_buffer[i].imu.acc_y, imu_buffer[i].imu.acc_z,
            imu_buffer[i].imu.gyro_x, imu_buffer[i].imu.gyro_y, imu_buffer[i].imu.gyro_z);
        netconn_write(conn, sendbuf, strlen(sendbuf), NETCONN_COPY);
        // printf("%s\n", sendbuf);
    }
    // sprintf(sendbuf, "imu end\n");
    // netconn_write(conn, sendbuf, strlen(sendbuf), NETCONN_COPY);
    printf("imu readout >\n");
}

void imu_readout_init(void)
{
    imu_serial = fopen("/dev/comIMU", "r");
    if (imu_serial == NULL) {
        printf("imu readout init: ERROR could not open serial port\n");
        return;
    }
    OSTaskCreateExt(imu_readout_task,
                    NULL,
                    &imu_readout_stk[IMU_TASK_STACKSIZE-1],
                    IMU_TASK_PRIORITY,
                    IMU_TASK_PRIORITY,
                    &imu_readout_stk[0],
                    IMU_TASK_STACKSIZE,
                    NULL, NULL);
}

