/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cmd_frame.h"

#include <math.h>
#include <string.h>

#define CRC_OFFSET (CMD_FRAME_LEN - 2U)

uint16_t cmd_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t) ((uint16_t) data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000U) != 0U ? (uint16_t) ((crc << 1) ^ 0x1021U) : (uint16_t) (crc << 1);
        }
    }
    return crc;
}

static void put_f32(uint8_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    dst[0] = (uint8_t) bits;
    dst[1] = (uint8_t) (bits >> 8);
    dst[2] = (uint8_t) (bits >> 16);
    dst[3] = (uint8_t) (bits >> 24);
}

static float get_f32(const uint8_t *src)
{
    const uint32_t bits = (uint32_t) src[0] | ((uint32_t) src[1] << 8) | ((uint32_t) src[2] << 16) |
                          ((uint32_t) src[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void seal(uint8_t *frame)
{
    const uint16_t crc = cmd_crc16(frame, CRC_OFFSET);
    frame[CRC_OFFSET] = (uint8_t) crc;
    frame[CRC_OFFSET + 1U] = (uint8_t) (crc >> 8);
}

/* Length, magic and CRC, in that order. */
static cmd_result_t check(const uint8_t *buf, size_t len, uint8_t magic)
{
    if (len != CMD_FRAME_LEN) {
        return CMD_ERR_LENGTH;
    }
    if (buf[0] != magic) {
        return CMD_ERR_MAGIC;
    }

    const uint16_t crc = (uint16_t) ((uint16_t) buf[CRC_OFFSET] | ((uint16_t) buf[CRC_OFFSET + 1U] << 8));
    if (crc != cmd_crc16(buf, CRC_OFFSET)) {
        return CMD_ERR_CRC;
    }
    return CMD_OK;
}

void cmd_frame_build(const cmd_frame_t *cmd, uint8_t *out)
{
    memset(out, 0, CMD_FRAME_LEN);
    out[0] = CMD_FRAME_MAGIC;
    out[1] = CMD_FRAME_VERSION;
    out[2] = cmd->seq;
    out[3] = cmd->flags;
    put_f32(&out[4], cmd->vx);
    put_f32(&out[8], cmd->vy);
    put_f32(&out[12], cmd->wz);
    put_f32(&out[16], cmd->height);
    put_f32(&out[20], cmd->tilt_r);
    put_f32(&out[24], cmd->tilt_p);
    out[28] = cmd->gait;
    seal(out);
}

cmd_result_t cmd_frame_parse(const uint8_t *buf, size_t len, cmd_frame_t *out)
{
    const cmd_result_t res = check(buf, len, CMD_FRAME_MAGIC);
    if (res != CMD_OK) {
        return res;
    }
    if (buf[1] != CMD_FRAME_VERSION) {
        return CMD_ERR_VERSION;
    }

    cmd_frame_t cmd;
    cmd.seq = buf[2];
    cmd.flags = buf[3];
    cmd.vx = get_f32(&buf[4]);
    cmd.vy = get_f32(&buf[8]);
    cmd.wz = get_f32(&buf[12]);
    cmd.height = get_f32(&buf[16]);
    cmd.tilt_r = get_f32(&buf[20]);
    cmd.tilt_p = get_f32(&buf[24]);
    cmd.gait = buf[28];

    if (!isfinite(cmd.vx) || !isfinite(cmd.vy) || !isfinite(cmd.wz) || !isfinite(cmd.height) ||
        !isfinite(cmd.tilt_r) || !isfinite(cmd.tilt_p)) {
        return CMD_ERR_VALUE;
    }

    *out = cmd;
    return CMD_OK;
}

void status_frame_build(const status_frame_t *status, uint8_t *out)
{
    memset(out, 0, CMD_FRAME_LEN);
    out[0] = STATUS_FRAME_MAGIC;
    out[1] = CMD_FRAME_VERSION;
    out[2] = STATUS_FRAME_V1_LENGTH;
    out[3] = status->seq;
    out[4] = status->state;
    out[5] = status->fault;
    out[6] = status->flags;
    out[7] = status->contact;
    put_f32(&out[8], status->yaw);
    seal(out);
}

cmd_result_t status_frame_parse(const uint8_t *buf, size_t len, status_frame_t *out)
{
    const cmd_result_t res = check(buf, len, STATUS_FRAME_MAGIC);
    if (res != CMD_OK) {
        return res;
    }
    if (buf[1] < CMD_FRAME_VERSION) {
        return CMD_ERR_VERSION;
    }
    /* A later version only adds fields behind these, the known ones stay put */
    if (buf[2] < STATUS_FRAME_V1_LENGTH || buf[2] > CRC_OFFSET - 3U) {
        return CMD_ERR_VALUE;
    }

    status_frame_t status;
    status.seq = buf[3];
    status.state = buf[4];
    status.fault = buf[5];
    status.flags = buf[6];
    status.contact = buf[7];
    status.yaw = get_f32(&buf[8]);
    if (!isfinite(status.yaw)) {
        return CMD_ERR_VALUE;
    }

    *out = status;
    return CMD_OK;
}

void cmd_rx_init(cmd_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

cmd_rx_result_t cmd_rx_process(cmd_rx_t *rx, const uint8_t *buf, size_t len, cmd_frame_t *out)
{
    cmd_frame_t cmd;
    const cmd_result_t res = cmd_frame_parse(buf, len, &cmd);

    if (res != CMD_OK) {
        if (res == CMD_ERR_CRC) {
            rx->crc_errors++;
        } else {
            rx->rejected++;
        }
        rx->last_error = res;
        return CMD_RX_BAD;
    }
    rx->last_error = CMD_OK;

    if (rx->have_last && cmd.seq == rx->last_seq) {
        rx->duplicates++;
        return CMD_RX_DUPLICATE;
    }

    if (rx->have_last) {
        rx->gaps += (uint8_t) (cmd.seq - rx->last_seq - 1U);
    }
    rx->have_last = true;
    rx->last_seq = cmd.seq;
    rx->accepted++;
    *out = cmd;
    return CMD_RX_NEW;
}
