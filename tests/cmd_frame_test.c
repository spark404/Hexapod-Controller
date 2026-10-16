//
// Host test for the SPI command and status frames, see Core/Src/cmd_frame.c
//
// The golden frames were produced by an independent implementation (Python,
// struct + a bitwise CRC-16/IBM-3740) and are meant to be reused by the Pi side
// tests, so both ends agree on the bytes.
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc \
//      tests/cmd_frame_test.c Core/Src/cmd_frame.c -o /tmp/cmd_frame_test && /tmp/cmd_frame_test
//
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cmd_frame.h"

/* seq 1, no flags, vx 50, height 100, everything else 0 */
static const uint8_t GOLDEN_WALK[CMD_FRAME_LEN] = {
    0xA5, 0x01, 0x01, 0x00, 0x00, 0x00, 0x48, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xC8, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0xAE};

/* seq 255, flags stop + fault clear, vx -12.5, vy 40, wz -0.1, height 120, tilt 0.05 / -0.05 */
static const uint8_t GOLDEN_MIXED[CMD_FRAME_LEN] = {
    0xA5, 0x01, 0xFF, 0x05, 0x00, 0x00, 0x48, 0xC1, 0x00, 0x00, 0x20, 0x42, 0xCD, 0xCC, 0xCC, 0xBD,
    0x00, 0x00, 0xF0, 0x42, 0xCD, 0xCC, 0x4C, 0x3D, 0xCD, 0xCC, 0x4C, 0xBD, 0x00, 0x3E, 0xE2};

/* seq 7, state 4, fault 0, flags link ok + contact valid, contact legs 0 and 2, yaw -1.5 */
static const uint8_t GOLDEN_STATUS[CMD_FRAME_LEN] = {
    0x5A, 0x01, 0x0C, 0x07, 0x04, 0x00, 0x05, 0x05, 0x00, 0x00, 0xC0, 0xBF, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xEC};

/* Rewrite the CRC after a test edited a frame */
static void reseal(uint8_t *f)
{
    const uint16_t crc = cmd_crc16(f, CMD_FRAME_LEN - 2U);
    f[CMD_FRAME_LEN - 2U] = (uint8_t) crc;
    f[CMD_FRAME_LEN - 1U] = (uint8_t) (crc >> 8);
}

static void test_crc_check_value(void)
{
    assert(cmd_crc16((const uint8_t *) "123456789", 9) == 0x29B1);   /* the catalogued check value */
    assert(cmd_crc16(NULL, 0) == 0xFFFF);                             /* initial value */
}

static void test_build_matches_golden(void)
{
    uint8_t f[CMD_FRAME_LEN];

    cmd_frame_t walk = {.seq = 1, .vx = 50.0f, .height = 100.0f};
    cmd_frame_build(&walk, f);
    assert(memcmp(f, GOLDEN_WALK, CMD_FRAME_LEN) == 0);

    cmd_frame_t mixed = {.seq = 255,
                         .flags = CMD_FLAG_STOP | CMD_FLAG_FAULT_CLEAR,
                         .vx = -12.5f,
                         .vy = 40.0f,
                         .wz = -0.1f,
                         .height = 120.0f,
                         .tilt_r = 0.05f,
                         .tilt_p = -0.05f,
                         .gait = CMD_GAIT_TRIPOD};
    cmd_frame_build(&mixed, f);
    assert(memcmp(f, GOLDEN_MIXED, CMD_FRAME_LEN) == 0);

    status_frame_t st = {.seq = 7,
                         .state = 4,
                         .flags = STATUS_FLAG_LINK_OK | STATUS_FLAG_CONTACT_VALID,
                         .contact = 0x05,
                         .yaw = -1.5f};
    status_frame_build(&st, f);
    assert(memcmp(f, GOLDEN_STATUS, CMD_FRAME_LEN) == 0);
}

static void test_parse_golden(void)
{
    cmd_frame_t c;
    assert(cmd_frame_parse(GOLDEN_MIXED, CMD_FRAME_LEN, &c) == CMD_OK);
    assert(c.seq == 255 && c.flags == 0x05 && c.gait == 0);
    assert(c.vx == -12.5f && c.vy == 40.0f && c.height == 120.0f);
    assert(fabsf(c.wz + 0.1f) < 1e-7f && fabsf(c.tilt_r - 0.05f) < 1e-7f && fabsf(c.tilt_p + 0.05f) < 1e-7f);

    status_frame_t s;
    assert(status_frame_parse(GOLDEN_STATUS, CMD_FRAME_LEN, &s) == CMD_OK);
    assert(s.seq == 7 && s.state == 4 && s.fault == 0 && s.contact == 0x05);
    assert(s.flags == (STATUS_FLAG_LINK_OK | STATUS_FLAG_CONTACT_VALID));
    assert(s.yaw == -1.5f);
}

static void test_round_trip_values(void)
{
    const float samples[] = {0.0f, -0.0f, 1.0f, -1.0f, 3.14159274f, -1e-30f, 123456.789f, 1e30f};
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
        cmd_frame_t in = {.seq = (uint8_t) i, .flags = 0x07, .vx = samples[i], .vy = -samples[i], .wz = samples[i],
                          .height = samples[i], .tilt_r = samples[i], .tilt_p = samples[i], .gait = 3};
        uint8_t f[CMD_FRAME_LEN];
        cmd_frame_t out;
        cmd_frame_build(&in, f);
        assert(cmd_frame_parse(f, CMD_FRAME_LEN, &out) == CMD_OK);
        assert(memcmp(&in, &out, sizeof(in)) == 0 || (in.vx == out.vx && in.vy == out.vy));   /* -0.0 compares equal */
        assert(out.seq == in.seq && out.flags == in.flags && out.gait == 3);
        assert(out.vx == in.vx && out.height == in.height && out.tilt_p == in.tilt_p);
    }
}

static void test_rejects_damage(void)
{
    uint8_t f[CMD_FRAME_LEN];
    cmd_frame_t c;

    assert(cmd_frame_parse(GOLDEN_WALK, CMD_FRAME_LEN - 1U, &c) == CMD_ERR_LENGTH);
    assert(cmd_frame_parse(GOLDEN_WALK, CMD_FRAME_LEN + 1U, &c) == CMD_ERR_LENGTH);
    assert(cmd_frame_parse(GOLDEN_WALK, 0, &c) == CMD_ERR_LENGTH);

    /* a status frame is not a command */
    assert(cmd_frame_parse(GOLDEN_STATUS, CMD_FRAME_LEN, &c) == CMD_ERR_MAGIC);

    /* every single bit flip in the frame is caught: by the CRC, or by the magic for byte 0 */
    for (size_t byte = 0; byte < CMD_FRAME_LEN; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            memcpy(f, GOLDEN_MIXED, CMD_FRAME_LEN);
            f[byte] ^= (uint8_t) (1U << bit);
            assert(cmd_frame_parse(f, CMD_FRAME_LEN, &c) != CMD_OK);
        }
    }

    /* a version we do not know, intact: reported as such, not as a CRC error */
    memcpy(f, GOLDEN_WALK, CMD_FRAME_LEN);
    f[1] = 2;
    reseal(f);
    assert(cmd_frame_parse(f, CMD_FRAME_LEN, &c) == CMD_ERR_VERSION);

    /* NaN and infinity with a good CRC */
    const uint8_t nan_le[4] = {0x00, 0x00, 0xC0, 0x7F};
    const uint8_t inf_le[4] = {0x00, 0x00, 0x80, 0x7F};
    for (size_t field = 4; field <= 24; field += 4) {
        memcpy(f, GOLDEN_WALK, CMD_FRAME_LEN);
        memcpy(&f[field], nan_le, 4);
        reseal(f);
        assert(cmd_frame_parse(f, CMD_FRAME_LEN, &c) == CMD_ERR_VALUE);

        memcpy(f, GOLDEN_WALK, CMD_FRAME_LEN);
        memcpy(&f[field], inf_le, 4);
        reseal(f);
        assert(cmd_frame_parse(f, CMD_FRAME_LEN, &c) == CMD_ERR_VALUE);
    }
}

static void test_status_is_forward_compatible(void)
{
    status_frame_t s;
    uint8_t f[CMD_FRAME_LEN];

    /* a later version: a longer length and a field behind the known ones, version 2 */
    memcpy(f, GOLDEN_STATUS, CMD_FRAME_LEN);
    f[1] = 2;
    f[2] = 20;
    f[16] = 0x99;   /* a new field the reader does not know */
    reseal(f);
    assert(status_frame_parse(f, CMD_FRAME_LEN, &s) == CMD_OK);
    assert(s.seq == 7 && s.yaw == -1.5f);

    /* shorter than the fields it must have, or longer than the frame has room for */
    f[2] = STATUS_FRAME_V1_LENGTH - 1U;
    reseal(f);
    assert(status_frame_parse(f, CMD_FRAME_LEN, &s) == CMD_ERR_VALUE);
    f[2] = 27;
    reseal(f);
    assert(status_frame_parse(f, CMD_FRAME_LEN, &s) == CMD_ERR_VALUE);

    /* version 0 does not exist */
    memcpy(f, GOLDEN_STATUS, CMD_FRAME_LEN);
    f[1] = 0;
    reseal(f);
    assert(status_frame_parse(f, CMD_FRAME_LEN, &s) == CMD_ERR_VERSION);

    /* a command is not a status */
    assert(status_frame_parse(GOLDEN_WALK, CMD_FRAME_LEN, &s) == CMD_ERR_MAGIC);
}

static void send(cmd_rx_t *rx, uint8_t seq, cmd_rx_result_t expect)
{
    uint8_t f[CMD_FRAME_LEN];
    cmd_frame_t in = {.seq = seq, .vx = 10.0f}, out;
    cmd_frame_build(&in, f);
    assert(cmd_rx_process(rx, f, CMD_FRAME_LEN, &out) == expect);
    if (expect == CMD_RX_NEW) {
        assert(out.seq == seq && out.vx == 10.0f);
    }
}

static void test_rx_sequence_handling(void)
{
    cmd_rx_t rx;
    cmd_rx_init(&rx);

    send(&rx, 5, CMD_RX_NEW);            /* the first frame is accepted whatever its seq */
    send(&rx, 5, CMD_RX_DUPLICATE);      /* a repeat is not new input */
    send(&rx, 6, CMD_RX_NEW);
    send(&rx, 9, CMD_RX_NEW);            /* 7 and 8 were lost */
    assert(rx.gaps == 2 && rx.accepted == 3 && rx.duplicates == 1);

    /* wraps through 255 to 0 without counting a gap */
    uint32_t gaps = rx.gaps;
    send(&rx, 255, CMD_RX_NEW);
    assert(rx.gaps == gaps + 245U);      /* 9 -> 255 skipped 245 commands */
    send(&rx, 0, CMD_RX_NEW);
    send(&rx, 1, CMD_RX_NEW);
    assert(rx.gaps == gaps + 245U);      /* 255 -> 0 -> 1 skipped none */

    /* damaged frames change nothing but the counters */
    uint8_t f[CMD_FRAME_LEN];
    cmd_frame_t out;
    memcpy(f, GOLDEN_WALK, CMD_FRAME_LEN);
    f[10] ^= 0x40;
    assert(cmd_rx_process(&rx, f, CMD_FRAME_LEN, &out) == CMD_RX_BAD);
    assert(rx.crc_errors == 1 && rx.last_error == CMD_ERR_CRC);
    assert(cmd_rx_process(&rx, GOLDEN_WALK, 7, &out) == CMD_RX_BAD);
    assert(rx.rejected == 1 && rx.last_error == CMD_ERR_LENGTH);
    send(&rx, 2, CMD_RX_NEW);            /* last accepted seq is still 1 */
    assert(rx.last_error == CMD_OK);

    /* the Pi restarted and starts over at seq 0: accepted, the "gap" is only a statistic */
    send(&rx, 0, CMD_RX_NEW);
}

int main(void)
{
    test_crc_check_value();
    test_build_matches_golden();
    test_parse_golden();
    test_round_trip_values();
    test_rejects_damage();
    test_status_is_forward_compatible();
    test_rx_sequence_handling();
    printf("cmd_frame_test: all tests passed\n");
    return 0;
}
