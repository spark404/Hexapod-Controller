/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Servo bus diagnostics.
 *
 * Counts failed and successful Dynamixel bus transactions, transport and UART
 * level errors, and measures the round-trip time of successful transactions.
 * The firmware prints a windowed summary periodically (see bus_diag_format).
 *
 * This module has no HAL or RTOS dependency. All counters are updated with
 * relaxed atomics, so they can be bumped from tasks and interrupt handlers
 * alike. bus_diag_record() tracks a failure streak and must only be called from
 * one context (the servo task).
 */

#ifndef BUS_DIAG_H
#define BUS_DIAG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Servos on the bus. Failures are counted per servo index (leg * 3 + joint). */
#define BUS_DIAG_MAX_SERVOS 18

/* The first servo replies one Return Delay Time (20 us) after a request; the receiver has to be
 * armed before that. A turnaround above this limit is counted as slow (T7.17). */
#define BUS_DIAG_TURNAROUND_LIMIT_US 15U

/* Transactions that are timed and counted per operation */
typedef enum {
    BUS_OP_READ_STATE = 0,  /* Sync Read of Present Load, Velocity and Position */
    BUS_OP_WRITE_GOAL,      /* Sync Write of Goal Position */
    BUS_OP_COUNT
} bus_op_t;

/* Outcome of a transaction, derived from the dynamixel result code */
typedef enum {
    BUS_RES_OK = 0,
    BUS_RES_CRC,            /* DNM_RECV_CRC_FAIL */
    BUS_RES_NOT_STATUS,     /* DNM_RECV_NOT_STATUS */
    BUS_RES_LL_ERR,         /* DNM_LL_ERR */
    BUS_RES_TIMEOUT,        /* DNM_LL_TIMEOUT */
    BUS_RES_API,            /* DNM_API_ERR */
    BUS_RES_ID,             /* DNM_RECV_ID_MISMATCH */
    BUS_RES_LENGTH,         /* DNM_RECV_LENGTH_MISMATCH */
    BUS_RES_OTHER,          /* any other non-zero code */
    BUS_RES_COUNT
} bus_res_t;

/* Events below the protocol layer, and recovery actions */
typedef enum {
    BUS_EV_RD_MUTEX_TMO = 0, /* usart_read: reader mutex not acquired */
    BUS_EV_RD_EMPTY,         /* usart_read: timeout with no data */
    BUS_EV_RD_PARTIAL,       /* usart_read: timeout with fewer bytes than asked */
    BUS_EV_WR_MUTEX_TMO,     /* usart_write: writer mutex not acquired */
    BUS_EV_WR_TXQ_FULL,      /* usart_write: TX queue full */
    BUS_EV_WR_TX_TMO,        /* usart_write: TX complete not signalled */
    BUS_EV_UART_ORE,         /* UART overrun */
    BUS_EV_UART_FE,          /* UART framing error */
    BUS_EV_UART_NE,          /* UART noise error */
    BUS_EV_UART_PE,          /* UART parity error */
    BUS_EV_UART_DMA,         /* UART DMA error */
    BUS_EV_RX_DROP,          /* received bytes dropped, RX ring full (counts bytes) */
    BUS_EV_RESET_READ_FAIL,  /* RX DMA reset after a failed servo read */
    BUS_EV_RESET_UART_ERR,   /* RX DMA reset after a UART error */
    BUS_EV_SERVO_ALERT,      /* servo reported a hardware alert in a read (the data is valid) */
    BUS_EV_LOOP_OVERRUN,     /* servo loop iteration took longer than its period */
    BUS_EV_RX_STALE,         /* bytes in the receiver when a request was sent, cleared (counts bytes) */
    BUS_EV_WR_SKIP_BUSY,     /* goal write skipped: the bus did not go quiet after a failed read */
    BUS_EV_TURNAROUND_SLOW,  /* receiver armed more than BUS_DIAG_TURNAROUND_LIMIT_US after a transmit */
    BUS_EV_COUNT
} bus_event_t;

/* Point-in-time copy of the counters. Window statistics (min, max, streak)
 * cover the time since the previous snapshot taken with reset_window set. */
typedef struct {
    uint32_t res[BUS_OP_COUNT][BUS_RES_COUNT];
    uint32_t time_sum_us[BUS_OP_COUNT];     /* successful transactions only */
    uint32_t time_min_us[BUS_OP_COUNT];     /* UINT32_MAX when none */
    uint32_t time_max_us[BUS_OP_COUNT];
    uint32_t events[BUS_EV_COUNT];
    uint32_t servo_fail[BUS_DIAG_MAX_SERVOS]; /* reads without data, per servo index */
    uint32_t turnaround_count;              /* transmits whose turnaround was measured */
    uint32_t turnaround_sum_us;
    uint32_t turnaround_min_us;             /* UINT32_MAX when none */
    uint32_t turnaround_max_us;
    uint32_t max_streak;                    /* longest run of failed transactions */
    uint32_t last_error;                    /* last non-zero dynamixel result code */
} bus_diag_snapshot_t;

/* Text of a periodic report. Empty strings mean "nothing to print". */
typedef struct {
    char summary[224];      /* always filled */
    char protocol[224];     /* per result code detail, only with errors in the window */
    char transport[384];    /* transport and UART detail, only with errors in the window */
    char servos[160];       /* failed reads per servo, only when a servo failed in the window */
    char boot[160];         /* totals since boot, only when any error happened */
    bool window_errors;     /* any failure or event in the window */
} bus_diag_report_t;

void bus_diag_init(void);

/* Record one finished transaction. dnm_result is the dynamixel result code
 * (0 = ok). duration_us is only used for successful transactions. */
void bus_diag_record(bus_op_t op, uint8_t dnm_result, uint32_t duration_us);

/* Count a read that gave no data for servo number servo_index (0..17). For a
 * timeout, only the servo that did not answer should be counted, not the ones
 * behind it that could not answer either. */
void bus_diag_servo_failure(unsigned servo_index);

/* Add n occurrences of an event */
void bus_diag_event(bus_event_t ev, uint32_t n);

/* Record the time from the end of a transmit to the receiver being armed again. Safe from an
 * interrupt handler. Above BUS_DIAG_TURNAROUND_LIMIT_US it also counts BUS_EV_TURNAROUND_SLOW. */
void bus_diag_turnaround(uint32_t us);

/* Copy the counters. With reset_window the window statistics (min, max,
 * streak, turnaround min and max) are cleared after being copied. */
void bus_diag_snapshot(bus_diag_snapshot_t *out, bool reset_window);

/* Build the report for the window between prev and cur. Counter differences
 * use unsigned arithmetic, so they stay correct when a counter wraps. */
void bus_diag_format(const bus_diag_snapshot_t *prev, const bus_diag_snapshot_t *cur,
                     uint32_t window_s, bus_diag_report_t *out);

#endif /* BUS_DIAG_H */
