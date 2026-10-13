/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "log_sink.h"

#include <string.h>

/* "[log] 4294967295 message(s) dropped\r\n" is 37 bytes */
#define NOTICE_MAX 40U

static struct {
    uint8_t *buf;
    size_t size;
    size_t head;          /* next byte to write */
    size_t tail;          /* oldest byte not yet transmitted */
    size_t inflight;      /* bytes at tail handed to start_tx() */
    bool active;          /* a transmission is running */
    const log_sink_port_t *port;
    log_sink_stats_t stats;
    uint32_t unreported;  /* dropped messages not announced yet */
} s;

static size_t used(void)
{
    return (s.head + s.size - s.tail) % s.size;
}

static size_t room(void)
{
    return s.size - 1U - used();
}

static void put(const uint8_t *src, size_t len)
{
    size_t first = s.size - s.head;
    if (first > len) {
        first = len;
    }
    memcpy(s.buf + s.head, src, first);
    memcpy(s.buf, src + first, len - first);
    s.head = (s.head + len) % s.size;
}

static size_t format_notice(char *dst, uint32_t count)
{
    static const char head[] = "[log] ";
    static const char tail[] = " message(s) dropped\r\n";
    char digits[10];
    size_t n = 0;

    for (uint32_t v = count; v != 0U; v /= 10U) {
        digits[n++] = (char) ('0' + (v % 10U));
    }

    size_t len = 0;
    memcpy(dst, head, sizeof(head) - 1U);
    len += sizeof(head) - 1U;
    while (n > 0U) {
        dst[len++] = digits[--n];
    }
    memcpy(dst + len, tail, sizeof(tail));   /* with the terminator, len not counted */
    return len + sizeof(tail) - 1U;
}

/* Called with the lock held. */
static void start_locked(void)
{
    if (s.active) {
        return;
    }

    size_t n = used();
    if (n == 0U) {
        return;
    }

    /* One contiguous run: the transfer cannot wrap around the end. */
    size_t run = s.size - s.tail;
    if (n > run) {
        n = run;
    }

    if (s.port->start_tx(s.buf + s.tail, n)) {
        s.active = true;
        s.inflight = n;
    }
}

void log_sink_init(uint8_t *buf, size_t size, const log_sink_port_t *port)
{
    memset(&s, 0, sizeof(s));
    s.buf = buf;
    s.size = size;
    s.port = port;
}

size_t log_sink_write(const void *data, size_t len)
{
    if (len == 0U) {
        return 0;
    }

    uint32_t token = s.port->lock();

    char notice[NOTICE_MAX];
    size_t notice_len = 0;
    if (s.unreported != 0U) {
        notice_len = format_notice(notice, s.unreported);
    }

    size_t result = 0;
    if (len <= room() && notice_len <= room() - len) {
        if (notice_len != 0U) {
            put((const uint8_t *) notice, notice_len);
            s.stats.written_bytes += (uint32_t) notice_len;
            s.unreported = 0;
        }
        put((const uint8_t *) data, len);
        s.stats.written_bytes += (uint32_t) len;

        size_t now = used();
        if (now > s.stats.high_water) {
            s.stats.high_water = (uint32_t) now;
        }
        result = len;
    } else {
        s.stats.dropped_messages++;
        s.stats.dropped_bytes += (uint32_t) len;
        s.unreported++;
    }

    start_locked();
    s.port->unlock(token);
    return result;
}

void log_sink_tx_done(bool ok)
{
    uint32_t token = s.port->lock();

    if (s.active) {
        s.tail = (s.tail + s.inflight) % s.size;
        if (!ok) {
            s.stats.dropped_bytes += (uint32_t) s.inflight;
        }
        s.inflight = 0;
        s.active = false;
        start_locked();
    }

    s.port->unlock(token);
}

void log_sink_kick(void)
{
    uint32_t token = s.port->lock();
    start_locked();
    s.port->unlock(token);
}

size_t log_sink_pending(void)
{
    uint32_t token = s.port->lock();
    size_t n = used();
    s.port->unlock(token);
    return n;
}

size_t log_sink_inflight(void)
{
    uint32_t token = s.port->lock();
    size_t n = s.active ? s.inflight : 0U;
    s.port->unlock(token);
    return n;
}

void log_sink_drain_polled(void (*put)(uint8_t byte), size_t skip)
{
    size_t n = used();
    if (skip > n) {
        skip = n;
    }

    size_t at = (s.tail + skip) % s.size;
    for (size_t i = skip; i < n; i++) {
        put(s.buf[at]);
        at = (at + 1U) % s.size;
    }

    s.tail = s.head;
    s.inflight = 0;
    s.active = false;
}

void log_sink_get_stats(log_sink_stats_t *out)
{
    uint32_t token = s.port->lock();
    *out = s.stats;
    s.port->unlock(token);
}
