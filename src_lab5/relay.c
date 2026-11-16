#include "relay.h"
#include "config.h"
#include "can_bus.h"
#include "netwatch.h"
#include "raft.h"
#include "time_node.h"
#include "wcet.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

/* RELAY_TX test point (PA9): toggles each time a frame forwarded A -> B has been sent on
 * bus B. N2 only: on N3, PA9 is PWM_SET. */
#if NODE_ID == ZONE_N2
static const struct gpio_dt_spec relay_tp = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), relay_tx_gpios);
#endif

struct relay_job {
    struct can_frame frame;
    int64_t rx_us; /* when it arrived: a REF's time in the relay is added to its master time */
    int to_bus;
};

/* Deeper than CTRL's 1: every relayed frame is a different message that must get across */
K_MSGQ_DEFINE_STATIC(relay_q, sizeof(struct relay_job), 8, 4);

static atomic_t relayed;

/* The routing table from the graph (RAFT thread, every 10 ms): bit (S * 2 + X) set =
 * forward S's frames heard on bus X */
static atomic_t route_bits;

void relay_set_routes(const struct route_table *t) {
    atomic_val_t bits = 0;
    for (int s = 1; s <= RAFT_N; s++) {
        for (int b = 0; b < 2; b++) {
            if (t->relay[s][b]) {
                bits |= BIT(s * 2 + b);
            }
        }
    }
    atomic_set(&route_bits, bits);
}

/* Who sent this Lab 3 frame? (the zone whose reachability decides the relay) */
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

/* The routing rule on what we hear ourselves (netwatch.h), for the time before the graph
 * has the link change (Part 5.2). The sender counts as off the other bus after
 * RELAY_AFTER_MS, before the receivers' 50 ms heartbeat timeout. */
static bool observed_route(int sender, int from_bus) {
    struct netwatch_obs o = {0};
    for (int z = ZONE_N1; z <= ZONE_N3; z++) {
        int64_t limit = (z == sender) ? RELAY_AFTER_MS : HB_TIMEOUT_MS;
        for (int b = 0; b < 2; b++) {
            o.direct[z][b] = z != NODE_ID && can_io_hb_direct_age_ms(z, b) < limit;
            o.fresh[z][b] = o.direct[z][b];
        }
    }
    return route_relay_observed(&o, NODE_ID, (uint8_t)sender, (uint8_t)from_bus);
}

void relay_offer(const struct can_frame *f, int from_bus, int64_t rx_us) {
    bool raft = raft_is_frame(f->id);
    uint8_t flag = raft ? RAFT_FLAG_RELAYED : FLAG_RELAYED;
    if (f->data[0] & flag) {
        return; /* already a relayed copy: never relay twice (no loops) */
    }

    int sender = raft ? RAFT_ID_SENDER(f->id) : sender_of(f);
    if (sender < ZONE_N1 || sender > ZONE_N3 || sender == NODE_ID) {
        return;
    }
    bool by_graph = atomic_test_bit(&route_bits, sender * 2 + from_bus);
    if (!by_graph && !observed_route(sender, from_bus)) {
        return;
    }

    struct relay_job job = {.frame = *f, .rx_us = rx_us, .to_bus = 1 - from_bus};
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
        bool raft = raft_is_frame(job.frame.id);
        if (raft && RAFT_ID_TYPE(job.frame.id) == RAFT_ID_REF) {
            /* Transparent clock: add the first hop and our time in the queue, stamped as
             * late as possible (timesync.h) */
            time_relay_ref(RAFT_ID_SENDER(job.frame.id), job.frame.data, job.rx_us);
        }
        if (can_io_forward(job.to_bus, &job.frame, raft ? RAFT_FLAG_RELAYED : FLAG_RELAYED) == 0) {
            atomic_inc(&relayed);
        }
#if NODE_ID == ZONE_N2
        /* After the send on bus B returns (TA clarification, measurement #6: RELAY C,
         * CAN_RX_A -> RELAY_TX). Only A -> B forwards toggle, so every edge pairs with a CAN_RX_A edge. */
        if (job.to_bus == 1) {
            gpio_pin_toggle_dt(&relay_tp);
        }
#endif
        wcet_record(WCET_RELAY, t0);
    }
}

K_THREAD_DEFINE(relay_tid, TASK_STACK_SIZE, relay_thread, NULL, NULL, NULL, RELAY_PRIO, 0, SYS_FOREVER_MS);

int relay_start(void) {
#if NODE_ID == ZONE_N2
    if (!gpio_is_ready_dt(&relay_tp)) {
        printk("RELAY_TX isn't ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&relay_tp, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }
#endif

    k_thread_start(relay_tid);
    return 0;
}

uint32_t relay_take_count(void) {
    return (uint32_t)atomic_clear(&relayed);
}
