#ifndef APP_WCET_H_
#define APP_WCET_H_

/*
 * Measured WCET per task, for the RTA (Part 3). Each task calls
 *     uint32_t t0 = wcet_start();  ...body...  wcet_record(WCET_xxx, t0);
 * from AFTER its blocking wait returns until just BEFORE it blocks again, so a
 * sample is the CPU time of one job. It is wall time, though: if a higher-
 * priority thread or an ISR preempts the body, that time is included, so a
 * sample is an upper bound on C (say so in the report). Max over the whole run.
 *
 * Cycle counter: SysTick at CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC (84 MHz, ~12 ns).
 */

#include <stdint.h>
#include <zephyr/kernel.h>

enum wcet_id {
    WCET_CAN_RX,    /* rx_cb: dispatch of one frame (driver thread) */
    WCET_HB_TX,
    WCET_HB_MON,
    WCET_TEST_BTN,
    WCET_BLINK,
    WCET_DRIVE,     /* N3 */
    WCET_PID,       /* N3 */
    WCET_STEER,     /* N2 */
    WCET_FORCE_TX,  /* N2 */
    WCET_RELAY,     /* N2 */
    WCET_COUNT,
};

static inline uint32_t wcet_start(void) {
    return k_cycle_get_32();
}

/* Keep the max for this id. Safe with several writers (CAN_RX has two). */
void wcet_record(enum wcet_id id, uint32_t start);

/* main: print every task's max so far, in microseconds (tasks never run show 0). */
void wcet_print(void);

#endif /* APP_WCET_H_ */
