//
// Created by Hugo Trippaers on 09/01/2026.
//
#include "ring_buffer.h"

void ringbuf_init(ringbuf_t *rb, uint8_t *buf, size_t size)
{
    rb->buffer = buf;
    rb->size   = size;
    rb->head   = 0;
    rb->tail   = 0;
}

size_t ringbuf_available(const ringbuf_t *rb)
{
    size_t head = rb->head;
    size_t tail = rb->tail;

    if (head >= tail) {
        return head - tail;
    }

    return rb->size - tail + head;
}

size_t ringbuf_free(const ringbuf_t *rb)
{
    return rb->size - 1 - ringbuf_available(rb);
}

bool ringbuf_push(ringbuf_t *rb, uint8_t byte)
{
    size_t next = (rb->head + 1) % rb->size;

    if (next == rb->tail) {
        return false;  // full
    }

    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

bool ringbuf_pop(ringbuf_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) {
        return false;  // empty
    }

    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return true;
}

size_t ringbuf_read(ringbuf_t *rb, uint8_t *dst, size_t len)
{
    size_t count = 0;

    while ((count < len) && ringbuf_pop(rb, &dst[count])) {
        count++;
    }

    return count;
}

size_t ringbuf_write(ringbuf_t *rb, const uint8_t *src, size_t len)
{
    size_t count = 0;

    while ((count < len) && ringbuf_push(rb, src[count])) {
        count++;
    }

    return count;
}

void ringbuf_reset(ringbuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}
