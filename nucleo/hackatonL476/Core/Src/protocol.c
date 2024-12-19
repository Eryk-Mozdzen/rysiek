#include "protocol.h"

#define MAGIC1 0x1E
#define MAGIC2 0xE1
#define MAGIC3 0x69

uint8_t checksum(const uint8_t *buffer, const uint8_t size) {
    uint8_t sum = 0;
    for(uint8_t i=0; i<size; i++) {
        sum +=buffer[i];
    }
    return sum;
}

bool protocol_consume(protocol_t *protocol, const uint8_t byte) {
    if(protocol->counter==0) {
        if(byte==MAGIC1) {
            protocol->buffer[protocol->counter] = byte;
            protocol->counter++;
            return false;
        }
    }

    if(protocol->counter==1) {
        if(byte==MAGIC2) {
            protocol->buffer[protocol->counter] = byte;
            protocol->counter++;
            return false;
        }
    }

    if(protocol->counter>=2 && protocol->counter<=5) {
        protocol->buffer[protocol->counter] = byte;
        protocol->counter++;
        return false;
    }

    if(protocol->counter==6) {
        if(byte==MAGIC3) {
            protocol->buffer[protocol->counter] = byte;
            protocol->counter++;
            return false;
        }
    }

    if(protocol->counter==7) {
        return (protocol->frame.checksum==checksum(protocol->buffer, protocol->counter));
    }

    protocol->counter = 0;
    return false;
}
