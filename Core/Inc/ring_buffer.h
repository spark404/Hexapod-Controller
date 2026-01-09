//
// Created by Hugo Trippaers on 09/01/2026.
//

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    size_t   size;   // must be > 1
    size_t   head;
    size_t   tail;
} ringbuf_t;

void   ringbuf_init(ringbuf_t *rb, uint8_t *buf, size_t size);

size_t ringbuf_available(const ringbuf_t *rb);
size_t ringbuf_free(const ringbuf_t *rb);

bool   ringbuf_push(ringbuf_t *rb, uint8_t byte);
bool   ringbuf_pop(ringbuf_t *rb, uint8_t *byte);

size_t ringbuf_read(ringbuf_t *rb, uint8_t *dst, size_t len);
size_t ringbuf_write(ringbuf_t *rb, const uint8_t *src, size_t len);

void   ringbuf_reset(ringbuf_t *rb);

#endif //RING_BUFFER_H
