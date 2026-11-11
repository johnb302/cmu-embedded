#include "relay.h"
#include "config.h"
#include "can_bus.h"
#include "wcet.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

/* RELAY_TX test point (PA9): toggles each time a frame forwarded A -> B has been sent on bus B */
static const struct gpio_dt_spec relay_tp = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), relay_tx_gpios);

struct relay_job {
    struct can_frame frame;
    int to_bus;
};

/* Deeper than CTRL's 1: every relayed frame is a different message that must get across */
K_MSGQ_DEFINE_STATIC(relay_q, sizeof(struct relay_job), 8, 4);

static atomic_t relayed;

/* Who sent this frame? (the zone whose reachability decides the relay) */
static int sender_of(const struct can_frame *f) {
    switch (f->id) {
        case ID_CTRL:
        case ID_BLINK:
            return ZONE_N1;
        case ID_HB_N1:
            return ZONE_N1;
        case ID_HB_N2:
            return ZONE_N2;
        case ID_HB_N3:
            return ZONE_N3;
        case ID_FAULT:
            return f->data[1]; /* b1 = the failing zone = the sender */
        default:
            return -1;
    }
}

/* Is any zone other than `except` (and us) heard on this bus? */
static bool someone_else_on(int bus, int except) {
    for (int z = ZONE_N1; z <= ZONE_N3; z++) {
        if (z == NODE_ID || z == except) {
            continue;
        }
        if (can_io_hb_age_ms(z, bus) < HB_TIMEOUT_MS) {
            return true;
        }
    }
    return false;
}

void relay_offer(const struct can_frame *f, int from_bus) {
    if (f->data[0] & FLAG_RELAYED) {
        return; /* already a relayed copy: never relay twice (no loops) */
    }

    int sender = sender_of(f);
    if (sender < ZONE_N1 || sender > ZONE_N3 || sender == NODE_ID) {
        return;
    }

    int to_bus = 1 - from_bus;
    if (can_io_hb_age_ms(sender, to_bus) < RELAY_AFTER_MS) {
        return; /* the sender reaches the other bus itself */
    }
    if (!someone_else_on(to_bus, sender)) {
        return; /* nobody on the other bus to relay to */
    }

    struct relay_job job = { .frame = *f, .to_bus = to_bus };
    (void)k_msgq_put(&relay_q, &job, K_NO_WAIT); /* full = 8 queued: drop, the next one follows */
}

static void relay_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        struct relay_job job;

        k_msgq_get(&relay_q, &job, K_FOREVER);
        uint32_t t0 = wcet_start();
        if (can_io_forward(job.to_bus, &job.frame) == 0) {
            atomic_inc(&relayed);
        }
        /* After the send on bus B returns (TA clarification, measurement #6: RELAY C,
         * CAN_RX_A -> RELAY_TX). Only A -> B forwards toggle, so every edge pairs with a CAN_RX_A edge. */
        if (job.to_bus == 1) {
            gpio_pin_toggle_dt(&relay_tp);
        }
        wcet_record(WCET_RELAY, t0);
    }
}

K_THREAD_DEFINE(relay_tid, TASK_STACK_SIZE, relay_thread, NULL, NULL, NULL, RELAY_PRIO, 0, SYS_FOREVER_MS);

int relay_start(void) {
    if (!gpio_is_ready_dt(&relay_tp)) {
        printk("RELAY_TX isn't ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&relay_tp, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    k_thread_start(relay_tid);
    return 0;
}

uint32_t relay_take_count(void) {
    return (uint32_t)atomic_clear(&relayed);
}
