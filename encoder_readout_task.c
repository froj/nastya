
#include <ucos_ii.h>
#include <uptime.h>
#include "tasks.h"

struct encoder_datapoint {
    int32_t encoders[3];
    timestamp_t timestamp;
};


#define ENC_SAMPLE_PERIOD 1000 // [us]

#define ENC_BUFFER_SIZE 100000
struct encoder_datapoint enc_buffer[ENC_BUFFER_SIZE];

OS_STK encoder_readout_stk[ENCODER_TASK_STACKSIZE];

void encoder_readout_task(void *pdata)
{
    timestamp_t last_iteration = uptime_get();
    int i = 0;
    while (i < ENC_BUFFER_SIZE) {
        timestamp_t now = uptime_get();
        enc_buffer[i].encoders[0] = cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE);
        enc_buffer[i].encoders[1] = cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE); // TODO
        enc_buffer[i].encoders[2] = cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE);
        enc_buffer[i].timestamp = now;
        i++;
        int32_t prev_period = now - last_iteration;
        last_iteration = now;
        int32_t delay = ENC_SAMPLE_PERIOD * 2 - prev_period;
        if (delay > 0)
            OSTimeDly((int64_t)OS_TICKS_PER_SEC*delay/1000000);
    }
}

void encoder_readout_start(void)
{

}

void encoder_readout_stop(void)
{

}

void encoder_readout_send(void)
{

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
