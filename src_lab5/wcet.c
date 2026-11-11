#include "wcet.h"
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

/* Max cycles per task. atomic_t: the task writes, main reads. */
static atomic_t max_cyc[WCET_COUNT];

static const char *const names[WCET_COUNT] = {
    [WCET_CAN_RX] = "CAN_RX",
    [WCET_HB_TX] = "HB_TX",
    [WCET_HB_MON] = "HB_MON",
    [WCET_TEST_BTN] = "TEST_BTN",
    [WCET_BLINK] = "BLINK",
    [WCET_DRIVE] = "DRIVE",
    [WCET_PID] = "PID",
    [WCET_STEER] = "STEER",
    [WCET_FORCE_TX] = "FORCE_TX",
    [WCET_RELAY] = "RELAY",
};

void wcet_record(enum wcet_id id, uint32_t start) {
    uint32_t d = k_cycle_get_32() - start; /* unsigned: correct across wrap */
    atomic_val_t old;

    /* CAN_RX has two writers (one driver thread per bus): compare-and-swap so a
     * bigger value can never be overwritten by a smaller one racing it */
    do {
        old = atomic_get(&max_cyc[id]);
        if (d <= (uint32_t)old) {
            return;
        }
    } while (!atomic_cas(&max_cyc[id], old, (atomic_val_t)d));
}

void wcet_print(void) {
    printk("WCET us:");
    for (int i = 0; i < WCET_COUNT; i++) {
        uint32_t c = (uint32_t)atomic_get(&max_cyc[i]);
        if (c != 0) {
            printk(" %s %u", names[i], k_cyc_to_us_ceil32(c));
        }
    }
    printk("\n");
}
