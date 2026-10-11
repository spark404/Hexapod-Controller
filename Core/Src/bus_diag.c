//
// Servo bus diagnostics, see bus_diag.h
//
#include "bus_diag.h"

#include <stdio.h>
#include <string.h>

#include "dynamixel/dynamixel_error.h"

#define RELAXED __ATOMIC_RELAXED

typedef struct {
    uint32_t res[BUS_OP_COUNT][BUS_RES_COUNT];
    uint32_t time_sum_us[BUS_OP_COUNT];
    uint32_t time_min_us[BUS_OP_COUNT];
    uint32_t time_max_us[BUS_OP_COUNT];
    uint32_t events[BUS_EV_COUNT];
    uint32_t servo_fail[BUS_DIAG_MAX_SERVOS];
    uint32_t max_streak;
    uint32_t last_error;
} bus_diag_state_t;

static bus_diag_state_t g_diag;

/* Failed transactions since the last success. Single writer: the servo task. */
static uint32_t g_streak;

static void atomic_add(uint32_t *p, const uint32_t v)
{
    __atomic_fetch_add(p, v, RELAXED);
}

static void atomic_max(uint32_t *p, const uint32_t v)
{
    uint32_t cur = __atomic_load_n(p, RELAXED);
    while (v > cur && !__atomic_compare_exchange_n(p, &cur, v, true, RELAXED, RELAXED)) {
    }
}

static void atomic_min(uint32_t *p, const uint32_t v)
{
    uint32_t cur = __atomic_load_n(p, RELAXED);
    while (v < cur && !__atomic_compare_exchange_n(p, &cur, v, true, RELAXED, RELAXED)) {
    }
}

static bus_res_t map_result(const uint8_t dnm_result)
{
    switch (dnm_result) {
        case DNM_OK:              return BUS_RES_OK;
        case DNM_RECV_CRC_FAIL:   return BUS_RES_CRC;
        case DNM_RECV_NOT_STATUS: return BUS_RES_NOT_STATUS;
        case DNM_LL_ERR:          return BUS_RES_LL_ERR;
        case DNM_LL_TIMEOUT:      return BUS_RES_TIMEOUT;
        case DNM_API_ERR:         return BUS_RES_API;
        case DNM_RECV_ID_MISMATCH:     return BUS_RES_ID;
        case DNM_RECV_LENGTH_MISMATCH: return BUS_RES_LENGTH;
        default:                  return BUS_RES_OTHER;
    }
}

void bus_diag_init(void)
{
    memset(&g_diag, 0, sizeof(g_diag));
    for (int op = 0; op < BUS_OP_COUNT; op++) {
        g_diag.time_min_us[op] = UINT32_MAX;
    }
    g_streak = 0;
}

void bus_diag_record(const bus_op_t op, const uint8_t dnm_result, const uint32_t duration_us)
{
    if ((unsigned) op >= (unsigned) BUS_OP_COUNT) {
        return;
    }

    const bus_res_t res = map_result(dnm_result);
    atomic_add(&g_diag.res[op][res], 1);

    if (res == BUS_RES_OK) {
        atomic_add(&g_diag.time_sum_us[op], duration_us);
        atomic_min(&g_diag.time_min_us[op], duration_us);
        atomic_max(&g_diag.time_max_us[op], duration_us);
        g_streak = 0;
    } else {
        g_streak++;
        atomic_max(&g_diag.max_streak, g_streak);
        __atomic_store_n(&g_diag.last_error, dnm_result, RELAXED);
    }
}

void bus_diag_servo_failure(const unsigned servo_index)
{
    if (servo_index >= BUS_DIAG_MAX_SERVOS) {
        return;
    }
    atomic_add(&g_diag.servo_fail[servo_index], 1);
}

void bus_diag_event(const bus_event_t ev, const uint32_t n)
{
    if ((unsigned) ev >= (unsigned) BUS_EV_COUNT) {
        return;
    }
    atomic_add(&g_diag.events[ev], n);
}

void bus_diag_snapshot(bus_diag_snapshot_t *out, const bool reset_window)
{
    for (int op = 0; op < BUS_OP_COUNT; op++) {
        for (int r = 0; r < BUS_RES_COUNT; r++) {
            out->res[op][r] = __atomic_load_n(&g_diag.res[op][r], RELAXED);
        }
        out->time_sum_us[op] = __atomic_load_n(&g_diag.time_sum_us[op], RELAXED);
        if (reset_window) {
            out->time_min_us[op] = __atomic_exchange_n(&g_diag.time_min_us[op], UINT32_MAX, RELAXED);
            out->time_max_us[op] = __atomic_exchange_n(&g_diag.time_max_us[op], 0U, RELAXED);
        } else {
            out->time_min_us[op] = __atomic_load_n(&g_diag.time_min_us[op], RELAXED);
            out->time_max_us[op] = __atomic_load_n(&g_diag.time_max_us[op], RELAXED);
        }
    }
    for (int e = 0; e < BUS_EV_COUNT; e++) {
        out->events[e] = __atomic_load_n(&g_diag.events[e], RELAXED);
    }
    for (int s = 0; s < BUS_DIAG_MAX_SERVOS; s++) {
        out->servo_fail[s] = __atomic_load_n(&g_diag.servo_fail[s], RELAXED);
    }
    if (reset_window) {
        out->max_streak = __atomic_exchange_n(&g_diag.max_streak, 0U, RELAXED);
    } else {
        out->max_streak = __atomic_load_n(&g_diag.max_streak, RELAXED);
    }
    out->last_error = __atomic_load_n(&g_diag.last_error, RELAXED);
}

/* Counter difference, correct across a 32-bit wrap */
static uint32_t delta(const uint32_t cur, const uint32_t prev)
{
    return cur - prev;
}

static uint32_t res_total(const bus_diag_snapshot_t *s, const bus_op_t op)
{
    uint32_t sum = 0;
    for (int r = 0; r < BUS_RES_COUNT; r++) {
        sum += s->res[op][r];
    }
    return sum;
}

static uint32_t res_failed(const bus_diag_snapshot_t *s, const bus_op_t op)
{
    return res_total(s, op) - s->res[op][BUS_RES_OK];
}

static uint32_t events_total(const bus_diag_snapshot_t *s, const bus_event_t first, const bus_event_t last)
{
    uint32_t sum = 0;
    for (int e = first; e <= (int) last; e++) {
        sum += s->events[e];
    }
    return sum;
}

/* "min/avg/max" in microseconds of the successful transactions of one window */
static void format_timing(char *buf, const size_t size, const bus_diag_snapshot_t *prev,
                          const bus_diag_snapshot_t *cur, const bus_op_t op)
{
    const uint32_t ok = delta(cur->res[op][BUS_RES_OK], prev->res[op][BUS_RES_OK]);
    if (ok == 0) {
        snprintf(buf, size, "-");
        return;
    }
    const uint32_t avg = delta(cur->time_sum_us[op], prev->time_sum_us[op]) / ok;
    snprintf(buf, size, "%lu/%lu/%lu",
             (unsigned long) cur->time_min_us[op], (unsigned long) avg, (unsigned long) cur->time_max_us[op]);
}

#define D(field) ((unsigned long) delta(cur->field, prev->field))
#define DR(op, r) ((unsigned long) delta(cur->res[op][r], prev->res[op][r]))
#define DE(e) ((unsigned long) delta(cur->events[e], prev->events[e]))

void bus_diag_format(const bus_diag_snapshot_t *prev, const bus_diag_snapshot_t *cur,
                     const uint32_t window_s, bus_diag_report_t *out)
{
    memset(out, 0, sizeof(*out));

    char rd_us[32];
    char wr_us[32];
    format_timing(rd_us, sizeof(rd_us), prev, cur, BUS_OP_READ_STATE);
    format_timing(wr_us, sizeof(wr_us), prev, cur, BUS_OP_WRITE_GOAL);

    const uint32_t rd_n = delta(res_total(cur, BUS_OP_READ_STATE), res_total(prev, BUS_OP_READ_STATE));
    const uint32_t wr_n = delta(res_total(cur, BUS_OP_WRITE_GOAL), res_total(prev, BUS_OP_WRITE_GOAL));
    const uint32_t rd_fail = delta(res_failed(cur, BUS_OP_READ_STATE), res_failed(prev, BUS_OP_READ_STATE));
    const uint32_t wr_fail = delta(res_failed(cur, BUS_OP_WRITE_GOAL), res_failed(prev, BUS_OP_WRITE_GOAL));

    uint32_t events_in_window = 0;
    for (int e = 0; e < BUS_EV_COUNT; e++) {
        events_in_window += delta(cur->events[e], prev->events[e]);
    }
    out->window_errors = (rd_fail + wr_fail + events_in_window) != 0;

    snprintf(out->summary, sizeof(out->summary),
             "[Bus] %lus: rd %lu ok %lu (us %s) | wr %lu ok %lu (us %s) | max fail streak %lu | loop overruns %lu",
             (unsigned long) window_s,
             (unsigned long) rd_n, (unsigned long) (rd_n - rd_fail), rd_us,
             (unsigned long) wr_n, (unsigned long) (wr_n - wr_fail), wr_us,
             (unsigned long) cur->max_streak, DE(BUS_EV_LOOP_OVERRUN));

    if (out->window_errors) {
        snprintf(out->protocol, sizeof(out->protocol),
                 "[Bus] rd err: crc %lu notstatus %lu ll %lu timeout %lu api %lu id %lu len %lu other %lu"
                 " | wr err: crc %lu notstatus %lu ll %lu timeout %lu api %lu other %lu | last code %lu",
                 DR(BUS_OP_READ_STATE, BUS_RES_CRC), DR(BUS_OP_READ_STATE, BUS_RES_NOT_STATUS),
                 DR(BUS_OP_READ_STATE, BUS_RES_LL_ERR), DR(BUS_OP_READ_STATE, BUS_RES_TIMEOUT),
                 DR(BUS_OP_READ_STATE, BUS_RES_API), DR(BUS_OP_READ_STATE, BUS_RES_ID),
                 DR(BUS_OP_READ_STATE, BUS_RES_LENGTH), DR(BUS_OP_READ_STATE, BUS_RES_OTHER),
                 DR(BUS_OP_WRITE_GOAL, BUS_RES_CRC), DR(BUS_OP_WRITE_GOAL, BUS_RES_NOT_STATUS),
                 DR(BUS_OP_WRITE_GOAL, BUS_RES_LL_ERR), DR(BUS_OP_WRITE_GOAL, BUS_RES_TIMEOUT),
                 DR(BUS_OP_WRITE_GOAL, BUS_RES_API), DR(BUS_OP_WRITE_GOAL, BUS_RES_OTHER),
                 (unsigned long) cur->last_error);

        snprintf(out->transport, sizeof(out->transport),
                 "[Bus] io: rd mutex %lu empty %lu partial %lu | wr mutex %lu txq %lu txtmo %lu"
                 " | uart ore %lu fe %lu ne %lu pe %lu dma %lu | rx dropped %lu B"
                 " | dma resets: after read %lu, after uart err %lu | servo alerts %lu",
                 DE(BUS_EV_RD_MUTEX_TMO), DE(BUS_EV_RD_EMPTY), DE(BUS_EV_RD_PARTIAL),
                 DE(BUS_EV_WR_MUTEX_TMO), DE(BUS_EV_WR_TXQ_FULL), DE(BUS_EV_WR_TX_TMO),
                 DE(BUS_EV_UART_ORE), DE(BUS_EV_UART_FE), DE(BUS_EV_UART_NE), DE(BUS_EV_UART_PE),
                 DE(BUS_EV_UART_DMA), DE(BUS_EV_RX_DROP),
                 DE(BUS_EV_RESET_READ_FAIL), DE(BUS_EV_RESET_UART_ERR), DE(BUS_EV_SERVO_ALERT));
    }

    /* Which servos did not give data in this window, only worth a line when one failed */
    size_t used = 0;
    for (int s = 0; s < BUS_DIAG_MAX_SERVOS; s++) {
        const uint32_t d = delta(cur->servo_fail[s], prev->servo_fail[s]);
        if (d == 0 || used >= sizeof(out->servos)) {
            continue;
        }
        if (used == 0) {
            used = (size_t) snprintf(out->servos, sizeof(out->servos),
                                     "[Bus] reads without data per servo (leg*3+joint):");
        }
        const int written = snprintf(out->servos + used, sizeof(out->servos) - used, " %d:%lu", s, (unsigned long) d);
        if (written > 0) {
            used += (size_t) written;
        }
    }
    if (used >= sizeof(out->servos)) {
        out->servos[sizeof(out->servos) - 1] = '\0';
    }
    if (out->servos[0] != '\0') {
        out->window_errors = true;
    }

    /* Totals since boot, only worth printing once something has gone wrong */
    const uint32_t total_rd_fail = res_failed(cur, BUS_OP_READ_STATE);
    const uint32_t total_wr_fail = res_failed(cur, BUS_OP_WRITE_GOAL);
    const uint32_t uart_errs = events_total(cur, BUS_EV_UART_ORE, BUS_EV_UART_DMA);
    const uint32_t resets = events_total(cur, BUS_EV_RESET_READ_FAIL, BUS_EV_RESET_UART_ERR);
    if (total_rd_fail + total_wr_fail + uart_errs + resets + cur->events[BUS_EV_RX_DROP] != 0) {
        snprintf(out->boot, sizeof(out->boot),
                 "[Bus] since boot: rd failed %lu/%lu, wr failed %lu/%lu, uart errors %lu, rx dropped %lu B, dma resets %lu",
                 (unsigned long) total_rd_fail, (unsigned long) res_total(cur, BUS_OP_READ_STATE),
                 (unsigned long) total_wr_fail, (unsigned long) res_total(cur, BUS_OP_WRITE_GOAL),
                 (unsigned long) uart_errs, (unsigned long) cur->events[BUS_EV_RX_DROP],
                 (unsigned long) resets);
    }
}
