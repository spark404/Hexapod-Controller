/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "log_sink_port.h"

#include "log_sink.h"

extern UART_HandleTypeDef huart1;
extern int __io_putchar(int ch);

#define LOG_UART (&huart1)

_Static_assert(LOG_SINK_BUF_SIZE >= 2U && LOG_SINK_BUF_SIZE <= 65535U, "UART DMA transfers are at most 65535 bytes");

static uint8_t s_buf[LOG_SINK_BUF_SIZE];
static volatile bool s_ready;

static uint32_t port_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void port_unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static bool port_start_tx(const uint8_t *data, size_t len)
{
    return HAL_UART_Transmit_DMA(LOG_UART, data, (uint16_t) len) == HAL_OK;
}

static const log_sink_port_t s_port = {
    .lock = port_lock,
    .unlock = port_unlock,
    .start_tx = port_start_tx,
};

void log_sink_uart_init(void)
{
    log_sink_init(s_buf, sizeof(s_buf), &s_port);
    s_ready = true;
}

bool log_sink_uart_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart != LOG_UART) {
        return false;
    }

    log_sink_tx_done(true);
    return true;
}

bool log_sink_uart_error(UART_HandleTypeDef *huart)
{
    if (huart != LOG_UART) {
        return false;
    }

    /* Only a failed DMA transfer ends the transmission; the other errors (RX
     * side, overrun) leave it running. */
    if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U) {
        log_sink_tx_done(false);
    }
    return true;
}

static void panic_put(uint8_t byte)
{
    (void) __io_putchar(byte);
}

void log_sink_uart_panic(void)
{
    if (!s_ready) {
        return;
    }

    /* How much of the running transfer went out: the DMA counts down. A stream
     * that is off with a transfer pending has finished it. */
    size_t skip = log_sink_inflight();
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *) LOG_UART->hdmatx->Instance;
    if ((stream->CR & DMA_SxCR_EN) != 0U) {
        skip -= (skip < stream->NDTR) ? skip : stream->NDTR;
    }

    __HAL_DMA_DISABLE(LOG_UART->hdmatx);
    CLEAR_BIT(LOG_UART->Instance->CR3, USART_CR3_DMAT);

    s_ready = false;
    log_sink_drain_polled(panic_put, skip);
}

/* Replaces the weak _write() of syscalls.c, which printf() ends up in. */
int _write(int file, char *ptr, int len)
{
    (void) file;

    if (len <= 0) {
        return 0;
    }

    if (!s_ready) {
        for (int i = 0; i < len; i++) {
            __io_putchar(ptr[i]);
        }
    } else {
        /* A dropped message is counted and announced by the sink, the caller
         * is told it was written either way: stdio would retry or give up. */
        (void) log_sink_write(ptr, (size_t) len);
    }

    return len;
}
