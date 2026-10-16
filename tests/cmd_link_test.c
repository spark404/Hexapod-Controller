//
// Host test for the shared command link state, see Core/Src/cmd_link.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc \
//      tests/cmd_link_test.c Core/Src/cmd_link.c Core/Src/cmd_frame.c -o /tmp/cmd_link_test \
//      && /tmp/cmd_link_test
//
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "cmd_link.h"

static unsigned depth;
static unsigned max_depth;

static uint32_t t_lock(void)
{
    depth++;
    if (depth > max_depth) {
        max_depth = depth;
    }
    return 0x5A;
}

static void t_unlock(uint32_t token)
{
    assert(token == 0x5A);
    assert(depth > 0);
    depth--;
}

static const cmd_link_port_t port = {t_lock, t_unlock};

static void frame(uint8_t *f, uint8_t seq, float vx)
{
    cmd_frame_t c = {.seq = seq, .vx = vx, .height = 100.0f};
    cmd_frame_build(&c, f);
}

static void test_take_gives_each_command_once(void)
{
    cmd_link_init(&port);
    uint8_t f[CMD_FRAME_LEN];
    cmd_frame_t c;
    uint32_t ms;

    assert(!cmd_link_take(&c, &ms));                        /* nothing yet */

    frame(f, 1, 10.0f);
    assert(cmd_link_on_frame(f, CMD_FRAME_LEN, 1000) == CMD_RX_NEW);
    assert(cmd_link_take(&c, &ms));
    assert(c.seq == 1 && c.vx == 10.0f && ms == 1000);
    assert(!cmd_link_take(&c, &ms));                        /* taken */

    /* only the latest of several is delivered */
    frame(f, 2, 20.0f);
    cmd_link_on_frame(f, CMD_FRAME_LEN, 1040);
    frame(f, 3, 30.0f);
    cmd_link_on_frame(f, CMD_FRAME_LEN, 1080);
    assert(cmd_link_take(&c, &ms));
    assert(c.seq == 3 && c.vx == 30.0f && ms == 1080);
    assert(!cmd_link_take(&c, &ms));
}

static void test_repeat_does_not_refresh_the_link(void)
{
    cmd_link_init(&port);
    uint8_t f[CMD_FRAME_LEN];
    cmd_frame_t c;
    uint32_t ms;

    frame(f, 5, 10.0f);
    cmd_link_on_frame(f, CMD_FRAME_LEN, 100);
    assert(cmd_link_take(&c, &ms) && ms == 100);

    assert(cmd_link_on_frame(f, CMD_FRAME_LEN, 140) == CMD_RX_DUPLICATE);   /* same seq again */
    assert(!cmd_link_take(&c, &ms));
}

static void test_bad_frames_change_nothing_but_the_reply_flag(void)
{
    cmd_link_init(&port);
    uint8_t f[CMD_FRAME_LEN];
    uint8_t reply[CMD_FRAME_LEN];
    status_frame_t st;
    cmd_frame_t c;
    uint32_t ms;

    frame(f, 1, 10.0f);
    cmd_link_on_frame(f, CMD_FRAME_LEN, 100);
    cmd_link_take(&c, &ms);

    f[6] ^= 0x10;                                           /* damaged */
    assert(cmd_link_on_frame(f, CMD_FRAME_LEN, 140) == CMD_RX_BAD);
    assert(!cmd_link_take(&c, &ms));
    cmd_link_get_reply(reply);
    assert(status_frame_parse(reply, CMD_FRAME_LEN, &st) == CMD_OK);
    assert((st.flags & STATUS_FLAG_CMD_REJECTED) != 0);

    frame(f, 2, 10.0f);                                     /* the next good one clears it */
    assert(cmd_link_on_frame(f, CMD_FRAME_LEN, 180) == CMD_RX_NEW);
    cmd_link_get_reply(reply);
    assert(status_frame_parse(reply, CMD_FRAME_LEN, &st) == CMD_OK);
    assert((st.flags & STATUS_FLAG_CMD_REJECTED) == 0);

    cmd_rx_t stats;
    cmd_link_get_stats(&stats);
    assert(stats.accepted == 2 && stats.crc_errors == 1);
}

static void test_reply_carries_the_published_status(void)
{
    cmd_link_init(&port);
    uint8_t reply[CMD_FRAME_LEN];
    status_frame_t st;

    /* before the control task published anything the reply is still a valid frame */
    cmd_link_get_reply(reply);
    assert(status_frame_parse(reply, CMD_FRAME_LEN, &st) == CMD_OK);
    assert(st.state == 0 && st.flags == 0 && st.yaw == 0.0f);

    status_frame_t pub = {.seq = 9, .state = 3, .fault = 0, .flags = STATUS_FLAG_LINK_OK | STATUS_FLAG_LIMITED,
                          .contact = 0, .yaw = 1.25f};
    cmd_link_publish_status(&pub);
    cmd_link_get_reply(reply);
    assert(status_frame_parse(reply, CMD_FRAME_LEN, &st) == CMD_OK);
    assert(st.seq == 9 && st.state == 3 && st.yaw == 1.25f);
    assert(st.flags == (STATUS_FLAG_LINK_OK | STATUS_FLAG_LIMITED));
}

int main(void)
{
    test_take_gives_each_command_once();
    test_repeat_does_not_refresh_the_link();
    test_bad_frames_change_nothing_but_the_reply_flag();
    test_reply_carries_the_published_status();
    assert(depth == 0 && max_depth == 1);                   /* always released, never nested */
    printf("cmd_link_test: all tests passed\n");
    return 0;
}
