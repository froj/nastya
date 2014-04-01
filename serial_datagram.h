
#ifndef SERIAL_DATAGRAM_H
#define SERIAL_DATAGRAM_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    int bufflen;
    int writep;
    bool esc;
    uint8_t err;
    void (*callback_fn)(uint8_t *pkg, int len);
} serial_datagram_rcv_buffer_t;


void serial_datagram_send_reset(void (*sendfn)(uint8_t *p, int len));
void serial_datagram_send(uint8_t *pkg, int len, void (*sendfn)(uint8_t *p, int len));

void serial_datagram_rcv_buffer_init(serial_datagram_rcv_buffer_t *rcvb,
    uint8_t *buffer, int bufflen, void (*callback_fn)(uint8_t *pkg, int len));
void serial_datagram_rcv(serial_datagram_rcv_buffer_t *buf, uint8_t *data, int len);

#endif // SERIAL_DATAGRAM_H
