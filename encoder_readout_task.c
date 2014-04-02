#include <stddef.h>
#include <stdbool.h>
#include <cvra_dc.h>
#include <ucos_ii.h>
#include <lwip/api.h>
#include <uptime.h>
#include "tasks.h"

struct encoder_datapoint {
    int32_t encoders[3];
    timestamp_t timestamp;
};


#define ENC_SAMPLE_PERIOD 10000 // [us]

#define ENC_BUFFER_SIZE 100000
static struct encoder_datapoint enc_buffer[ENC_BUFFER_SIZE];
static int enc_buffer_index = 0;
static bool enc_buffer_en = false;
static OS_EVENT *enc_buffer_mutex;

OS_STK encoder_readout_stk[ENCODER_TASK_STACKSIZE];

void encoder_readout_task(void *pdata)
{
    printf("encoder readout task started\n");
    timestamp_t last_iteration = uptime_get();
    enc_buffer_index = 0;
    while (1) {
        INT8U err;
        OSMutexPend(enc_buffer_mutex, 0, &err);
        timestamp_t now = uptime_get();
        if (enc_buffer_index < ENC_BUFFER_SIZE && enc_buffer_en) {
            //printf("enc %d\n", enc_buffer_index);
            enc_buffer[enc_buffer_index].encoders[0] = hw_get_wheel_0_encoder();
            enc_buffer[enc_buffer_index].encoders[1] = hw_get_wheel_1_encoder();
            enc_buffer[enc_buffer_index].encoders[2] = hw_get_wheel_2_encoder();
            enc_buffer[enc_buffer_index].timestamp = now;
            enc_buffer_index++;
        }
        OSMutexPost(enc_buffer_mutex);
        int32_t prev_period = now - last_iteration;
        last_iteration = now;
        int32_t delay = ENC_SAMPLE_PERIOD * 2 - prev_period;
        if (delay > 0)
            OSTimeDly((int64_t)OS_TICKS_PER_SEC*delay/1000000);
    }
}

void encoder_readout_start(void)
{
    INT8U err;
    OSMutexPend(enc_buffer_mutex, 0, &err);
    enc_buffer_index = 0;
    enc_buffer_en = true;
    OSMutexPost(enc_buffer_mutex);

}

void encoder_readout_stop(void)
{
    INT8U err;
    OSMutexPend(enc_buffer_mutex, 0, &err);
    enc_buffer_en = false;
    OSMutexPost(enc_buffer_mutex);
}


void encoder_readout_send(struct netconn *conn)
{
    printf("%d encoder values read\n", enc_buffer_index);
    int i;
    printf("< encoder readout\n");
    static char sendbuf[300];
    sprintf(sendbuf, "timestamp, encoder0, encoder1, encoder2\n");
    netconn_write(conn, sendbuf, strlen(sendbuf), NETCONN_COPY);
    for (i = 0; i < enc_buffer_index - 1; i++) {
        snprintf(sendbuf, sizeof(sendbuf), "%d, %d, %d, %d\n", enc_buffer[i].timestamp, enc_buffer[i].encoders[0], enc_buffer[i].encoders[1], enc_buffer[i].encoders[2]);
        netconn_write(conn, sendbuf, strlen(sendbuf), NETCONN_COPY);
        // printf("%s\n", sendbuf);
    }
    // sprintf(sendbuf, "enc end\n");
    // netconn_write(conn, sendbuf, strlen(sendbuf), NETCONN_COPY);
    printf("encoder readout >\n");
}

void encoder_readout_init(void)
{
    OSTaskCreateExt(encoder_readout_task,
                    NULL,
                    &encoder_readout_stk[ENCODER_TASK_STACKSIZE-1],
                    ENCODER_TASK_PRIORITY,
                    ENCODER_TASK_PRIORITY,
                    &encoder_readout_stk[0],
                    ENCODER_TASK_STACKSIZE,
                    NULL, NULL);
}
