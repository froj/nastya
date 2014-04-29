
#include <stdint.h>
#include <stdbool.h>

#include "serial_datagram.h"

// [data], [CRC-32], [ESC + END]
// ESC bytes in data & crc are escaped with ESC+DATA_ESC

#define ESC         0x2A
#define DATA_ESC    0x00
#define END         0x10


#define CRC32_RESET 0xffffffff
uint32_t crc32(uint32_t crc, uint8_t *buffer, int len)
{
    // TODO: use real CRC-32, this is only a simple bytewise xor checksum
    uint8_t x = 0xff & crc;
    int i;
    for (i = 0; i < len; i++)
        x ^= buffer[i];
    return x;
}

void serial_datagram_send_reset(void (*sendfn)(uint8_t *p, int len))
{
    uint8_t end[2] = {ESC, END};
    sendfn(end, 2);
}

static int _escape_add(uint8_t *buf, uint8_t val)
{
    if (val == ESC) {
        *buf++ = ESC;
        *buf++ = DATA_ESC;
        return 2;
    } else {
        *buf++ = val;
        return 1;
    }
}

void serial_datagram_send(uint8_t *pkg, int len, void (*sendfn)(uint8_t *p, int len))
{
    // data
    int b = 0, a = 0;
    while (b < len) {
        if (pkg[b] == ESC) {
            sendfn(&pkg[a], b - a);
            uint8_t esc[2] = {ESC, DATA_ESC};
            sendfn(esc, 2);
            a = b + 1;
        }
        b++;
    }
    sendfn(&pkg[a], b-a);
    // crc
    uint32_t crc = crc32(CRC32_RESET, pkg, len);
    uint8_t crc_[8];
    int crc_len = 0;
    crc_len += _escape_add(&crc_[crc_len], 0xFF & (crc >> 24));
    crc_len += _escape_add(&crc_[crc_len], 0xFF & (crc >> 16));
    crc_len += _escape_add(&crc_[crc_len], 0xFF & (crc >> 8));
    crc_len += _escape_add(&crc_[crc_len], 0xFF & crc);
    sendfn(crc_, crc_len);
    uint8_t end[2] = {ESC, END};
    sendfn(end, 2);
}


void serial_datagram_rcv_buffer_init(serial_datagram_rcv_buffer_t *rcvb,
    uint8_t *buffer, int bufflen, void (*callback_fn)(uint8_t *pkg, int len))
{
    rcvb->buffer = buffer;
    rcvb->bufflen = bufflen;

    rcvb->writep = 0;
    rcvb->esc = false;
    rcvb->err = 0;
    rcvb->callback_fn = callback_fn;
}

void serial_datagram_rcv(serial_datagram_rcv_buffer_t *buf, uint8_t *data, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        if (buf->esc) {
            if (data[i] == DATA_ESC) {
                // data: ESC
                buf->buffer[buf->writep++] = ESC;
                buf->esc = false;
            } else if (data[i] == END) {
                // end of packet
                if (!buf->err && buf->writep >= 4) {
                    int32_t c_crc = crc32(CRC32_RESET, buf->buffer, buf->writep - 4);
                    int32_t r_crc = 0;
                    r_crc += buf->buffer[buf->writep - 4] << 24;
                    r_crc += buf->buffer[buf->writep - 3] << 16;
                    r_crc += buf->buffer[buf->writep - 2] << 8;
                    r_crc += buf->buffer[buf->writep - 1];
                    if (c_crc == r_crc) {
                        buf->callback_fn(buf->buffer, buf->writep - 4);
                    } else {
                        // printf("serial_datagram_rcv: crc error %x %x\n", c_crc, r_crc);
                    }
                } else {
                    // printf("serial_datagram_rcv: frame error\n");
                }
                buf->writep = 0;
                buf->esc = false;
                buf->err = 0;
            } else if (data[i] == ESC) {
                // error, set esc
                buf->err = 1;
            } else {
                // error, clear esc
                buf->err = 1;
                buf->esc = false;
            }
        } else {
            if (data[i] == ESC) {
                buf->esc = true;
            } else {
                if (!buf->err && buf->writep < buf->bufflen) {
                    buf->buffer[buf->writep++] = data[i];
                } else {
                    buf->err = 1;
                }
            }
        }
    }
}


