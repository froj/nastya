
#include <ucos_ii.h>
#include <uptime.h>
#include "tasks.h"

struct encoder_datapoint {
    int32_t encoders[3];
    timestamp_t timestamp;
};


#define ENC_SAMPLE_PERIOD 1000 // [us]

#define ENC_BUFFER_SIZE 100000
static struct encoder_datapoint enc_buffer[ENC_BUFFER_SIZE];
static int enc_buffer_index = 0;
static bool enc_buffer_en = false;
static OS_EVENT *enc_buffer_mutex;

OS_STK encoder_readout_stk[ENCODER_TASK_STACKSIZE];

void encoder_readout_task(void *pdata)
{
    timestamp_t last_iteration = uptime_get();
    enc_buffer_index = 0;
    while (1) {
        INT8U err;
        OSMutexPend(enc_buffer_mutex, 0, &err);
        timestamp_t now = uptime_get();
        if (enc_buffer_index < ENC_BUFFER_SIZE && enc_buffer_en) {
            enc_buffer[enc_buffer_index].encoders[0] = cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE);
            enc_buffer[enc_buffer_index].encoders[1] = cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE);
            enc_buffer[enc_buffer_index].encoders[2] = cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE);
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

void encoder_readout_send(void)
{
    int i;
    for (i = 0; i < enc_buffer_index - 1; i++) {
        // todo
    }
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
