#include "blink.h"
#include "config.h"
#include "can_bus.h"
#include "time_node.h"
#include "wcet.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>

/* PC11 left, PC10 right on both nodes (FL/FR on N2, RL/RR on N3) */
static const struct gpio_dt_spec lamp_left = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_left_gpios);
static const struct gpio_dt_spec lamp_right = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_right_gpios);

/* Same pattern as CTRL: one deep, keep the newest */
K_MSGQ_DEFINE_STATIC(blink_q, sizeof(struct can_frame), 1, 4);

void blink_post(const struct can_frame *f) {
    if (k_msgq_put(&blink_q, f, K_NO_WAIT) != 0) {
        k_msgq_purge(&blink_q);
        (void)k_msgq_put(&blink_q, f, K_NO_WAIT);
    }
}

/* Failure mode (step 8): the cockpit is gone, no heartbeat on either bus (relayed copies
 * count). The same 60 ms detection that puts the zone in ERROR and brakes the motor. */
static bool cockpit_lost(void) {
    return can_io_hb_age_ms(ZONE_N1, 0) >= HB_TIMEOUT_MS && can_io_hb_age_ms(ZONE_N1, 1) >= HB_TIMEOUT_MS;
}

#define POLL_MS 20 /* re-check failure mode at least this often */

/* The last lamp edges, for `blink` in the shell: when (global µs) and to what */
#define EDGE_LOG 8
struct edge {
    int64_t global_us;
    uint8_t lamps; /* bit 0 left, bit 1 right */
};
static struct edge edges[EDGE_LOG];
static atomic_t edge_count;
static atomic_t shown_state;   /* the state now shown (BLINK_*) */
static atomic_t shown_half_ms; /* its half period */

static void set_lamps(bool left, bool right, int64_t g) {
    static uint8_t last = 0xFF;
    uint8_t lamps = (left ? 1 : 0) | (right ? 2 : 0);
    if (lamps != last) {
        gpio_pin_set_dt(&lamp_left, left);
        gpio_pin_set_dt(&lamp_right, right);
        atomic_val_t n = atomic_inc(&edge_count);
        edges[n % EDGE_LOG] = (struct edge){g, lamps};
        last = lamps;
    }
}

/*
 * BLINK_F (N2) / BLINK_R (N3), Lab 5: the Pi says which lamps blink (BLINK b1); when they
 * light comes from global time, so every lamp on every node switches at the same instant.
 * A lamp is on for the first half of each period: on while (global / half) is even, with
 * the edges at multiples of the half period. In failure mode the STM32s blink hazards on
 * their own, at 4 Hz, from the same clock, so they stay in step through the election.
 */
static void blink_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint8_t from_pi = BLINK_OFF;

    while (1) {
        uint32_t t0 = wcet_start();
        bool failsafe = cockpit_lost();
        uint8_t st = failsafe ? BLINK_HAZARD : from_pi;
        int64_t half_us = 1000LL * (failsafe ? BLINK_FAILSAFE_HALF_MS
                                             : (st == BLINK_HAZARD ? BLINK_HAZARD_HALF_MS : BLINK_TURN_HALF_MS));

        int64_t g = global_gettime();
        bool on = st != BLINK_OFF && (g / half_us) % 2 == 0;
        set_lamps(on && (st == BLINK_LEFT || st == BLINK_HAZARD), on && (st == BLINK_RIGHT || st == BLINK_HAZARD), g);
        atomic_set(&shown_state, st);
        atomic_set(&shown_half_ms, (atomic_val_t)(half_us / 1000));
        wcet_record(WCET_BLINK, t0);

        /* Sleep until just before the next edge (or POLL_MS, or a new BLINK frame) */
        int64_t edge = (g / half_us + 1) * half_us;
        int64_t wait_us = time_local_for_global(edge) - local_us() - 300;
        if (st == BLINK_OFF || wait_us > POLL_MS * 1000) {
            wait_us = POLL_MS * 1000;
        }
        struct can_frame frame;
        if (k_msgq_get(&blink_q, &frame, K_USEC(wait_us > 0 ? wait_us : 0)) == 0) {
            if (frame.data[1] <= BLINK_HAZARD) {
                from_pi = frame.data[1];
            }
            continue;
        }
        if (st != BLINK_OFF && time_local_for_global(edge) - local_us() < 1000) {
            time_wait_global(edge); /* the last ~200 µs spun: the edge on time */
        }
    }
}

K_THREAD_DEFINE(blink_tid, TASK_STACK_SIZE, blink_thread, NULL, NULL, NULL, BLINK_PRIO, 0, SYS_FOREVER_MS);

int blink_start(void) {
    const struct gpio_dt_spec *lamps[] = {&lamp_left, &lamp_right};

    for (size_t i = 0; i < ARRAY_SIZE(lamps); i++) {
        if (!gpio_is_ready_dt(lamps[i])) {
            printk("Blinker %zu isn't ready\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(lamps[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("Blinker %zu can't be configured.\n", i);
            return ret;
        }
    }

    k_thread_start(blink_tid);
    return 0;
}

/* blink: what the lamps show, and their last edges in global time */
static int cmd_blink(const struct shell *sh, size_t argc, char **argv) {
    static const char *const names[] = {"off", "left", "right", "hazard"};
    atomic_val_t st = atomic_get(&shown_state);
    atomic_val_t n = atomic_get(&edge_count);
    shell_print(sh, "N%d lamps: %s, half period %ld ms%s", NODE_ID, st <= BLINK_HAZARD ? names[st] : "?",
                (long)atomic_get(&shown_half_ms), cockpit_lost() ? " (failure mode: cockpit lost)" : "");
    for (atomic_val_t i = n > EDGE_LOG ? n - EDGE_LOG : 0; i < n; i++) {
        struct edge e = edges[i % EDGE_LOG];
        shell_print(sh, "  global %lld.%06lld s  L%d R%d", (long long)(e.global_us / 1000000),
                    (long long)(e.global_us % 1000000), e.lamps & 1, (e.lamps >> 1) & 1);
    }
    return 0;
}

SHELL_CMD_REGISTER(blink, NULL, "Blinker state and its last edges (global time)", cmd_blink);
