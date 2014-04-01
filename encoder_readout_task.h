#ifndef ENCODER_READOUT_H
#define ENCODER_READOUT_H

#include <lwip/api.h>

void encoder_readout_init(void);
void encoder_readout_start(void);
void encoder_readout_stop(void);
void encoder_readout_send(struct netconn *conn);

#endif // ENCODER_READOUT_H
