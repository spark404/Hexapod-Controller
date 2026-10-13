/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Non-blocking printf() on USART1 (the debug console) with DMA.
 *
 * Needs the USART1_TX DMA request enabled in CubeMX (normal mode, memory
 * increment, byte data width) with its interrupt. Wiring, all in USER CODE:
 *
 *   main():                       log_sink_uart_init();   after MX_USART1_UART_Init()
 *   HAL_UART_TxCpltCallback():    if (log_sink_uart_tx_cplt(huart)) return;
 *   HAL_UART_ErrorCallback():     if (log_sink_uart_error(huart)) return;
 *
 * Until log_sink_uart_init() is called, and for the early boot messages, printf()
 * keeps using the polled __io_putchar().
 */

#ifndef LOG_SINK_PORT_H
#define LOG_SINK_PORT_H

#include <stdbool.h>

#include "main.h"

#ifndef LOG_SINK_BUF_SIZE
#define LOG_SINK_BUF_SIZE 2048U   /* 180 ms of output at 115200 baud */
#endif

void log_sink_uart_init(void);

/* For a fatal error, call it right after __disable_irq(): stops the DMA, sends
 * what is still queued by polling and switches printf() back to polled output,
 * so the report of the error is not stuck in the buffer. */
void log_sink_uart_panic(void);

/* Return true when huart is the log UART (the event was handled or ignored). */
bool log_sink_uart_tx_cplt(UART_HandleTypeDef *huart);
bool log_sink_uart_error(UART_HandleTypeDef *huart);

#endif /* LOG_SINK_PORT_H */
