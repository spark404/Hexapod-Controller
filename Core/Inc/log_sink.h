/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Non-blocking log output.
 *
 * printf() output is copied into a ring buffer and a DMA transfer drains it, so
 * the caller never waits for the serial line. Messages are accepted or dropped
 * as a whole: when the buffer cannot hold a complete message the message is
 * counted and dropped, and the next message that fits is preceded by a
 * "[log] N message(s) dropped" notice. Lines are never cut in half.
 *
 * This file has no HAL, RTOS or libc-stdio dependency, so it builds on the host
 * (tests/log_sink_test.c) and can be used in other firmware: give it a buffer
 * and a port (a lock, and a function that starts a transmission), call
 * log_sink_tx_done() when that transmission ends. log_sink_port.c is the port
 * for this firmware (USART1 with DMA).
 *
 * Threading: log_sink_write() can be called from any task and from interrupt
 * handlers; log_sink_tx_done() from the transmit complete interrupt. All state
 * is protected by the port's lock, which must be usable from both (a PRIMASK
 * lock is). The lock is held for the copy of one message and for starting the
 * transmission, a few microseconds.
 */

#ifndef LOG_SINK_H
#define LOG_SINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /* Enter a critical section, returns a token for unlock(). Must work from
     * tasks and interrupt handlers, and nest. */
    uint32_t (*lock)(void);
    void (*unlock)(uint32_t token);

    /* Start transmitting len bytes (len >= 1) from data and return at once.
     * The data stays valid and untouched until log_sink_tx_done() is called.
     * Return false if the transmission could not be started (it is tried again
     * at the next write or log_sink_kick()). */
    bool (*start_tx)(const uint8_t *data, size_t len);
} log_sink_port_t;

typedef struct {
    uint32_t written_bytes;      /* bytes accepted into the buffer */
    uint32_t dropped_messages;   /* messages refused because the buffer was full */
    uint32_t dropped_bytes;      /* their size, plus bytes lost to a failed transmission */
    uint32_t high_water;         /* most bytes ever waiting in the buffer */
} log_sink_stats_t;

/* buf must stay valid for the life of the program. size must be at least 2 and
 * is limited by what the port's start_tx accepts in one go (the UART DMA takes
 * 65535). One byte of the buffer is never used. */
void log_sink_init(uint8_t *buf, size_t size, const log_sink_port_t *port);

/* Queue len bytes as one message. Returns len if it was queued, 0 if it was
 * dropped (no room, or larger than the buffer). Never blocks. */
size_t log_sink_write(const void *data, size_t len);

/* The transmission started by start_tx() ended. ok = false when it failed: its
 * bytes are discarded and counted as dropped, so a broken line cannot stall the
 * log. Call from the transmit complete / error interrupt. */
void log_sink_tx_done(bool ok);

/* Try to start a transmission if data is waiting and none is running. Only
 * needed when start_tx() failed and no new message is coming to retry it. */
void log_sink_kick(void);

/* Bytes waiting in the buffer, including the ones being transmitted. */
size_t log_sink_pending(void);

/* Bytes handed to the running transmission, 0 when none is running. */
size_t log_sink_inflight(void);

/* For a fatal error, with interrupts off and the transmission already stopped
 * by the caller: send everything still waiting with put() and leave the sink
 * empty. skip is how much of the running transmission already went out. Takes
 * no lock and does not need interrupts. */
void log_sink_drain_polled(void (*put)(uint8_t byte), size_t skip);

void log_sink_get_stats(log_sink_stats_t *out);

#endif /* LOG_SINK_H */
