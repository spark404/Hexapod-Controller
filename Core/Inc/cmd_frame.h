/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Command and status frames of the SPI link between the Raspberry Pi (master)
 * and the motion controller (slave), see docs/design.md section 3.
 *
 * Every transfer is CMD_FRAME_LEN bytes in both directions: the Pi clocks out a
 * command frame and clocks in the status frame the controller prepared one
 * transfer earlier. All multi-byte values are little-endian, floats are IEEE
 * 754 binary32. The CRC is CRC-16/IBM-3740 (CCITT-FALSE: polynomial 0x1021,
 * initial value 0xFFFF, no reflection, no final xor) over bytes 0 to 28, stored
 * little-endian in bytes 29 and 30.
 *
 * Command frame (Pi to controller):
 *
 *   0       magic    0xA5
 *   1       version  1
 *   2       seq      increments per command, wraps at 255
 *   3       flags    CMD_FLAG_*
 *   4..7    vx       mm/s, forward, level frame
 *   8..11   vy       mm/s, left
 *   12..15  wz       rad/s, counter-clockwise
 *   16..19  height   mm
 *   20..23  tilt_r   rad, body roll offset
 *   24..27  tilt_p   rad, body pitch offset
 *   28      gait     CMD_GAIT_*
 *   29..30  crc16
 *
 * Status frame (controller to Pi):
 *
 *   0       magic    0x5A
 *   1       version  1
 *   2       length   bytes 3 onwards that the sender defines (12 in version 1);
 *                    the rest up to the CRC is zero. A reader uses the fields it
 *                    knows and ignores the others.
 *   3       seq      seq of the last command applied
 *   4       state    controller state (controller_state_t): 0 boot, 1 syncing,
 *                    2 standup, 3 standing, 4 walking, 5 rotating, 6 powerdown
 *   5       fault    fault code, 0 = none
 *   6       flags    STATUS_FLAG_*
 *   7       contact  bit n set = leg n touches the ground
 *   8..11   yaw      rad, estimated body yaw
 *   12..28  reserved (zero in version 1)
 *   29..30  crc16
 *
 * This file has no HAL or RTOS dependency and builds on the host
 * (tests/cmd_frame_test.c).
 */

#ifndef CMD_FRAME_H
#define CMD_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CMD_FRAME_LEN 31U
#define CMD_FRAME_MAGIC 0xA5U
#define STATUS_FRAME_MAGIC 0x5AU
#define CMD_FRAME_VERSION 1U
#define STATUS_FRAME_V1_LENGTH 12U

/* Command flags */
#define CMD_FLAG_STOP 0x01U           /* force zero twist, stay standing */
#define CMD_FLAG_POWERDOWN 0x02U      /* request power down */
#define CMD_FLAG_FAULT_CLEAR 0x04U    /* clear a latched fault */

/* Gait ids */
#define CMD_GAIT_TRIPOD 0U

/* Status flags */
#define STATUS_FLAG_LINK_OK 0x01U        /* a valid command arrived within the link timeout */
#define STATUS_FLAG_LIMITED 0x02U        /* the controller could not do all of the command */
#define STATUS_FLAG_CONTACT_VALID 0x04U  /* the contact bits are measured, not a placeholder */
#define STATUS_FLAG_CMD_REJECTED 0x08U   /* the last frame failed validation */

typedef struct {
    uint8_t seq;
    uint8_t flags;
    float vx;
    float vy;
    float wz;
    float height;
    float tilt_r;
    float tilt_p;
    uint8_t gait;
} cmd_frame_t;

typedef struct {
    uint8_t seq;       /* seq of the last command applied */
    uint8_t state;
    uint8_t fault;
    uint8_t flags;
    uint8_t contact;
    float yaw;
} status_frame_t;

typedef enum {
    CMD_OK = 0,
    CMD_ERR_LENGTH,    /* not CMD_FRAME_LEN bytes */
    CMD_ERR_MAGIC,
    CMD_ERR_CRC,
    CMD_ERR_VERSION,   /* intact frame of a version this code does not know */
    CMD_ERR_VALUE      /* a float is NaN or infinite, or a length field is too short */
} cmd_result_t;

uint16_t cmd_crc16(const uint8_t *data, size_t len);

/* Fill out[CMD_FRAME_LEN] with the frame for cmd, CRC included. */
void cmd_frame_build(const cmd_frame_t *cmd, uint8_t *out);
cmd_result_t cmd_frame_parse(const uint8_t *buf, size_t len, cmd_frame_t *out);

void status_frame_build(const status_frame_t *status, uint8_t *out);
cmd_result_t status_frame_parse(const uint8_t *buf, size_t len, status_frame_t *out);

/*
 * Receive side bookkeeping: validates a frame and drops a repeat of the last
 * seq. A repeat is not new input, so it does not count as a command (the link
 * timeout only restarts on CMD_RX_NEW).
 */
typedef enum {
    CMD_RX_NEW,        /* valid, apply it */
    CMD_RX_DUPLICATE,  /* valid, same seq as the previous one: ignore */
    CMD_RX_BAD         /* failed validation: ignore, counted */
} cmd_rx_result_t;

typedef struct {
    bool have_last;
    uint8_t last_seq;
    uint32_t accepted;
    uint32_t duplicates;
    uint32_t gaps;          /* commands skipped between two accepted ones, as seen in seq */
    uint32_t crc_errors;
    uint32_t rejected;      /* length, magic, version or value errors */
    cmd_result_t last_error;
} cmd_rx_t;

void cmd_rx_init(cmd_rx_t *rx);
cmd_rx_result_t cmd_rx_process(cmd_rx_t *rx, const uint8_t *buf, size_t len, cmd_frame_t *out);

#endif /* CMD_FRAME_H */
