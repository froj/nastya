
#include <ucos_ii.h>
#include <uptime.h>

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
        enc_buffer[i].encoders[0] = 0;
        enc_buffer[i].encoders[1] = 0; // TODO
        enc_buffer[i].encoders[2] = 0;
        enc_buffer[i].timestamp = now;
        i++;
        int32_t prev_period = now - last_iteration;
        last_iteration = now;
        // DELAY (ENC_SAMPLE_PERIOD * 2 - prev_period) // TODO
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
