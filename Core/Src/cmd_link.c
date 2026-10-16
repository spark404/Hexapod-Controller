/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cmd_link.h"

static struct {
    const cmd_link_port_t *port;
    cmd_rx_t rx;
    cmd_frame_t cmd;
    uint32_t cmd_ms;
    bool fresh;          /* a command that cmd_link_take() has not returned yet */
    bool last_rejected;  /* the last frame failed validation */
    status_frame_t status;
} s;

void cmd_link_init(const cmd_link_port_t *port)
{
    s.port = port;
    cmd_rx_init(&s.rx);
    s.cmd = (cmd_frame_t) {0};
    s.cmd_ms = 0;
    s.fresh = false;
    s.last_rejected = false;
    s.status = (status_frame_t) {0};
}

cmd_rx_result_t cmd_link_on_frame(const uint8_t *rx, size_t len, uint32_t now_ms)
{
    cmd_frame_t cmd;
    const uint32_t token = s.port->lock();

    const cmd_rx_result_t res = cmd_rx_process(&s.rx, rx, len, &cmd);
    s.last_rejected = res == CMD_RX_BAD;
    if (res == CMD_RX_NEW) {
        s.cmd = cmd;
        s.cmd_ms = now_ms;
        s.fresh = true;
    }

    s.port->unlock(token);
    return res;
}

void cmd_link_get_reply(uint8_t *out)
{
    const uint32_t token = s.port->lock();
    status_frame_t status = s.status;
    if (s.last_rejected) {
        status.flags |= STATUS_FLAG_CMD_REJECTED;
    }
    s.port->unlock(token);

    status_frame_build(&status, out);
}

bool cmd_link_take(cmd_frame_t *cmd, uint32_t *rx_ms)
{
    const uint32_t token = s.port->lock();
    const bool fresh = s.fresh;
    if (fresh) {
        *cmd = s.cmd;
        *rx_ms = s.cmd_ms;
        s.fresh = false;
    }
    s.port->unlock(token);
    return fresh;
}

void cmd_link_publish_status(const status_frame_t *status)
{
    const uint32_t token = s.port->lock();
    s.status = *status;
    s.port->unlock(token);
}

void cmd_link_get_stats(cmd_rx_t *out)
{
    const uint32_t token = s.port->lock();
    *out = s.rx;
    s.port->unlock(token);
}
