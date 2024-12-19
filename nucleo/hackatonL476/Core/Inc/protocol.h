#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PROTOCOL_MATRIX_EMPTY = 0,
    PROTOCOL_MATRIX_FULL = 1,
    PROTOCOL_MATRIX_SMILE = 2,
    PROTOCOL_MATRIX_HEART = 3,
    PROTOCOL_MATRIX_KONAR = 4,
} PROTOCOL_matrix_t;

typedef struct {
    uint8_t magic1;
    uint8_t magic2;
    uint8_t checksum;
    uint8_t matrix;
    int8_t motor_left;
    int8_t motor_right;
    uint8_t magic3;
} protocol_frame_t;

typedef struct {
    union {
        protocol_frame_t frame;
        uint8_t buffer[sizeof(protocol_frame_t)];
    };
    uint8_t counter;
} protocol_t;

bool protocol_consume(protocol_t *protocol, const uint8_t byte);

#endif
