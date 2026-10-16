/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shared state between the SPI task and the control task for the command link
 * (backlog T7.5): the latest command received, and the status frame that is
 * clocked out in answer to the next one.
 *
 *   SPI task:      cmd_link_on_frame()   after every transfer
 *                  cmd_link_get_reply()  before arming the next transfer
 *   control task:  cmd_link_take()       once per tick
 *                  cmd_link_publish_status()
 *
 * All access goes through the port's lock (a critical section on the target),
 * held only for copying a few dozen bytes. This replaces the unprotected
 * updated_velocity / updated_heading / updated_height globals.
 *
 * Only the latest command is kept: if two arrive between control ticks the
 * first is overwritten, which is fine for a state command (the flags are levels,
 * not events). No HAL or RTOS dependency, host test in tests/cmd_link_test.c.
 */

#ifndef CMD_LINK_H
#define CMD_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cmd_frame.h"

typedef struct {
    uint32_t (*lock)(void);
    void (*unlock)(uint32_t token);
} cmd_link_port_t;

void cmd_link_init(const cmd_link_port_t *port);

/* SPI task: a transfer of len bytes ended at now_ms (ms, free running 32 bit counter). */
cmd_rx_result_t cmd_link_on_frame(const uint8_t *rx, size_t len, uint32_t now_ms);

/* SPI task: the CMD_FRAME_LEN bytes to clock out during the next transfer. */
void cmd_link_get_reply(uint8_t *out);

/* Control task: true when a new command arrived since the last call. A repeat of
 * the previous seq is not new, so it does not keep the link alive. */
bool cmd_link_take(cmd_frame_t *cmd, uint32_t *rx_ms);

/* Control task: what to report in the replies from now on. The command rejected
 * flag is added by the link itself. */
void cmd_link_publish_status(const status_frame_t *status);

void cmd_link_get_stats(cmd_rx_t *out);

#endif /* CMD_LINK_H */
