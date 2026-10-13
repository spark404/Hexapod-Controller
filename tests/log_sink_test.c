//
// Host test for the non-blocking log output, see Core/Src/log_sink.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc \
//      tests/log_sink_test.c Core/Src/log_sink.c -o /tmp/log_sink_test && /tmp/log_sink_test
//
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log_sink.h"

#define BUF 64U

static uint8_t g_buf[BUF];

/* The fake line: start_tx records the chunk, the test ends it with tx_done() */
static const uint8_t *g_chunk;
static size_t g_chunk_len;
static bool g_start_ok;
static unsigned g_starts;
static unsigned g_lock_depth;
static unsigned g_lock_max;

static char g_out[4096];
static size_t g_out_len;

static uint32_t t_lock(void)
{
    g_lock_depth++;
    if (g_lock_depth > g_lock_max) {
        g_lock_max = g_lock_depth;
    }
    return 0xA5;
}

static void t_unlock(uint32_t token)
{
    assert(token == 0xA5);
    assert(g_lock_depth > 0);
    g_lock_depth--;
}

static bool t_start(const uint8_t *data, size_t len)
{
    assert(g_lock_depth == 1);        /* started with the lock held, once */
    assert(g_chunk == NULL);          /* never two transmissions at once */
    assert(len >= 1);
    assert(data >= g_buf && data + len <= g_buf + BUF);   /* contiguous, inside the buffer */
    if (!g_start_ok) {
        return false;
    }
    g_chunk = data;
    g_chunk_len = len;
    g_starts++;
    return true;
}

static const log_sink_port_t port = {t_lock, t_unlock, t_start};

static void reset(void)
{
    log_sink_init(g_buf, BUF, &port);
    g_chunk = NULL;
    g_chunk_len = 0;
    g_start_ok = true;
    g_starts = 0;
    g_lock_depth = 0;
    g_out_len = 0;
    g_out[0] = '\0';
}

/* End the running transmission and keep what was sent */
static void finish(bool ok)
{
    assert(g_chunk != NULL);
    if (ok) {
        assert(g_out_len + g_chunk_len < sizeof(g_out));
        memcpy(g_out + g_out_len, g_chunk, g_chunk_len);
        g_out_len += g_chunk_len;
        g_out[g_out_len] = '\0';
    }
    g_chunk = NULL;
    log_sink_tx_done(ok);
}

/* Let the line run until nothing is left */
static void drain(void)
{
    while (g_chunk != NULL) {
        finish(true);
    }
}

static size_t put_str(const char *s)
{
    return log_sink_write(s, strlen(s));
}

static void test_write_and_transmit(void)
{
    reset();
    assert(put_str("hello\r\n") == 7);
    assert(g_starts == 1 && g_chunk_len == 7);       /* started at the first write */
    assert(log_sink_pending() == 7);

    assert(put_str("world\r\n") == 7);
    assert(g_starts == 1);                           /* still busy, only queued */

    finish(true);                                    /* the next chunk follows at once */
    assert(g_starts == 2 && g_chunk_len == 7);
    drain();

    assert(strcmp(g_out, "hello\r\nworld\r\n") == 0);
    assert(log_sink_pending() == 0);
    assert(put_str("") == 0 && g_starts == 2);       /* empty write does nothing */
}

static void test_wrap_around_is_split_in_two_chunks(void)
{
    reset();
    char msg[40];
    memset(msg, 'a', 39);
    msg[39] = '\0';
    assert(put_str(msg) == 39);
    drain();                                         /* tail and head at 39 */

    memset(msg, 'b', 39);
    assert(put_str(msg) == 39);                      /* wraps: 25 bytes + 14 bytes */
    assert(g_chunk_len == BUF - 39);
    finish(true);
    assert(g_chunk != NULL && g_chunk_len == 39 - (BUF - 39));
    drain();

    assert(g_out_len == 78);
    assert(strspn(g_out + 39, "b") == 39);
}

static void test_full_buffer_drops_whole_messages_and_announces(void)
{
    reset();
    char msg[BUF];
    memset(msg, 'x', sizeof(msg));

    /* the first chunk is in flight, so its bytes stay in the buffer */
    assert(log_sink_write(msg, 40) == 40);
    assert(log_sink_write(msg, 30) == 0);            /* 23 left: dropped, not cut */
    assert(log_sink_write(msg, 24) == 0);
    assert(log_sink_write(msg, BUF) == 0);           /* larger than the buffer */

    log_sink_stats_t st;
    log_sink_get_stats(&st);
    assert(st.dropped_messages == 3 && st.dropped_bytes == 30 + 24 + BUF);
    assert(st.written_bytes == 40);
    assert(st.high_water == 40);

    drain();

    /* now there is room: the notice comes first, then the message */
    assert(log_sink_write("after\r\n", 7) == 7);
    drain();
    assert(strstr(g_out, "[log] ") != NULL);
    assert(strstr(g_out, " message(s) dropped\r\n") != NULL);
    assert(strstr(g_out, "after\r\n") > strstr(g_out, "dropped"));
    assert(strncmp(g_out, "xxxxxxxx", 8) == 0);      /* what was queued is untouched */

    /* only announced once */
    size_t before = g_out_len;
    assert(put_str("next\r\n") == 6);
    drain();
    assert(g_out_len - before == 6);
}

static void test_notice_counts(void)
{
    reset();
    char msg[BUF];
    memset(msg, 'x', sizeof(msg));
    assert(log_sink_write(msg, BUF - 1) == BUF - 1);   /* exactly full */
    for (int i = 0; i < 12; i++) {
        assert(log_sink_write("y", 1) == 0);
    }
    drain();
    g_out_len = 0;
    assert(put_str("ok\r\n") == 4);
    drain();
    assert(strcmp(g_out, "[log] 12 message(s) dropped\r\nok\r\n") == 0);
}

static void test_failed_start_is_retried(void)
{
    reset();
    g_start_ok = false;
    assert(put_str("one\r\n") == 5);                 /* queued, no transmission */
    assert(g_chunk == NULL && log_sink_pending() == 5);

    g_start_ok = true;
    log_sink_kick();
    assert(g_chunk != NULL);
    drain();
    assert(strcmp(g_out, "one\r\n") == 0);

    /* and by the next write */
    g_start_ok = false;
    assert(put_str("two\r\n") == 5);
    g_start_ok = true;
    assert(put_str("three\r\n") == 7);
    drain();
    assert(strcmp(g_out, "one\r\ntwo\r\nthree\r\n") == 0);
}

static void test_failed_transmission_does_not_stall(void)
{
    reset();
    assert(put_str("lost\r\n") == 6);
    assert(put_str("kept\r\n") == 6);
    finish(false);                                   /* the first chunk is gone */
    drain();
    assert(strcmp(g_out, "kept\r\n") == 0);

    log_sink_stats_t st;
    log_sink_get_stats(&st);
    assert(st.dropped_bytes == 6 && st.dropped_messages == 0);

    log_sink_tx_done(true);                          /* spurious: ignored */
    assert(put_str("again\r\n") == 7);
    drain();
    assert(strcmp(g_out, "kept\r\nagain\r\n") == 0);
}

static void test_stream_is_intact_under_random_load(void)
{
    reset();
    srand(12345);
    static char expect[32768];
    size_t expect_len = 0;
    size_t sent = 0;

    for (int i = 0; i < 400; i++) {
        char msg[32];
        int n = snprintf(msg, sizeof(msg), "%d:%.*s\r\n", i, rand() % 12, "abcdefghijkl");
        size_t w = log_sink_write(msg, (size_t) n);
        assert(w == (size_t) n);                      /* the line keeps up, nothing is dropped */
        memcpy(expect + expect_len, msg, (size_t) n);
        expect_len += (size_t) n;

        /* the line is slow and bursty: finish 0 to 2 chunks per message */
        for (int k = rand() % 3; k > 0 && g_chunk != NULL; k--) {
            finish(true);
        }
        if (log_sink_pending() > 3 * BUF / 4) {
            drain();
        }
        if (g_out_len > sizeof(g_out) - 200) {        /* make room in the capture */
            assert(memcmp(g_out, expect + sent, g_out_len) == 0);
            sent += g_out_len;
            g_out_len = 0;
        }
    }
    drain();
    assert(sent + g_out_len == expect_len);
    assert(memcmp(g_out, expect + sent, g_out_len) == 0);

    log_sink_stats_t st;
    log_sink_get_stats(&st);
    assert(st.dropped_messages == 0 && st.written_bytes == expect_len);
    assert(st.high_water <= BUF - 1);
    assert(g_lock_depth == 0 && g_lock_max == 1);    /* no nesting, always released */
}

static char g_polled[256];
static size_t g_polled_len;

static void polled_put(uint8_t b)
{
    g_polled[g_polled_len++] = (char) b;
    g_polled[g_polled_len] = '\0';
}

static void test_polled_drain_after_a_stopped_transmission(void)
{
    reset();
    g_polled_len = 0;
    assert(put_str("0123456789") == 10);             /* in flight */
    assert(put_str("abcdef") == 6);                  /* queued behind it */
    assert(log_sink_inflight() == 10);

    /* the DMA was stopped with 4 of the 10 bytes left */
    log_sink_drain_polled(polled_put, 6);
    assert(strcmp(g_polled, "6789abcdef") == 0);
    assert(log_sink_pending() == 0 && log_sink_inflight() == 0);

    /* the sink is usable again */
    g_chunk = NULL;
    assert(put_str("x") == 1 && g_chunk != NULL);
}

static void test_polled_drain_wraps_and_clamps(void)
{
    reset();
    char msg[40];
    memset(msg, 'a', 39);
    msg[39] = '\0';
    assert(put_str(msg) == 39);
    drain();
    assert(put_str("0123456789") == 10);             /* first part ends at the end of the buffer */
    assert(put_str("ABCDEFGHIJKLMNOP") == 16);       /* wraps */

    g_polled_len = 0;
    log_sink_drain_polled(polled_put, 0);
    assert(strcmp(g_polled, "0123456789ABCDEFGHIJKLMNOP") == 0);

    reset();
    g_polled_len = 0;
    assert(put_str("abc") == 3);
    log_sink_drain_polled(polled_put, 99);           /* more than was queued */
    assert(g_polled_len == 0);
}

int main(void)
{
    test_write_and_transmit();
    test_wrap_around_is_split_in_two_chunks();
    test_full_buffer_drops_whole_messages_and_announces();
    test_notice_counts();
    test_failed_start_is_retried();
    test_failed_transmission_does_not_stall();
    test_stream_is_intact_under_random_load();
    test_polled_drain_after_a_stopped_transmission();
    test_polled_drain_wraps_and_clamps();
    printf("log_sink_test: all tests passed\n");
    return 0;
}
