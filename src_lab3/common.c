#include "common.h"
#include "config.h"
#include "can_bus.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>

static const struct gpio_dt_spec gpios[] = {GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), deg_a_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), deg_b_gpios)};

/* Peer nodes */
static const uint8_t peers[] = {ZONE_N1, ZONE_N2 + ZONE_N3 - NODE_ID};

/* Per-bus atomic value (health bit) */
static atomic_t health_a = 1;
static atomic_t health_b = 1;

static void hb_mon_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next = k_uptime_get();

    while(1) {
        int count_A = 0;
        int count_B = 0;
        /* Loop over the zones and check the age on each bus */
        for (int i = 0; i < ARRAY_SIZE(peers); i++) {
            /* Check bus A */
            int64_t zone_age_A = can_io_hb_age_ms(peers[i], 0);
            if (zone_age_A >= HB_TIMEOUT_MS) {
                count_A++;
            }

            int64_t zone_age_B = can_io_hb_age_ms(peers[i], 1);
            if (zone_age_B >= HB_TIMEOUT_MS) {
                count_B++;
            }

            /* Check if a zone is lost */
            if (zone_age_A >= HB_TIMEOUT_MS && zone_age_B >= HB_TIMEOUT_MS) {
                printk("Zone %d is lost.\n", i);
            }
        }

        /* If all nodes on BUS A miss heartbeat, bus is degraded */
        if (count_A == 2) {
            atomic_set(&health_a, 1);
            gpio_pin_set_dt(&gpios[0], 1);
        } else {
            atomic_set(&health_a, 0);
            gpio_pin_set_dt(&gpios[0], 0);
        }

        /* If all nodes on BUS B miss heartbeat, bus is degraded */
        if (count_B == 2) {
            atomic_set(&health_b, 1);
            gpio_pin_set_dt(&gpios[1], 1);
        } else {
            atomic_set(&health_b, 0);
            gpio_pin_set_dt(&gpios[1], 0);
        }

        next += HB_INTERVAL_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(hb_mon_tid, TASK_STACK_SIZE, hb_mon_thread, NULL, NULL, NULL, HB_MON_PRIO, 0, SYS_FOREVER_MS);

static void hb_tx_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next = k_uptime_get();

    uint16_t id = 0;
    if (NODE_ID == ZONE_N2) {
        id = ID_HB_N2;
    } else {
        id = ID_HB_N3;
    }

    while(1) {
        uint8_t data[FRAME_DLC] = {0};
        data[0] = 0;
        data[1] = HB_STATE_ERROR;
        /* Set flags to show both buses healthy */
        atomic_val_t deg_a = atomic_get(&health_a);
        atomic_val_t deg_b = atomic_get(&health_b);
        data[2] = (deg_a ? 0 : HB_HEALTH_A) | (deg_b ? 0 : HB_HEALTH_B);
        /* Set local fault */
        data[3] = 0;

        can_io_send(id, data);
        next += HB_INTERVAL_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(hb_tid, TASK_STACK_SIZE, hb_tx_thread, NULL, NULL, NULL, HB_TX_PRIO, 0, SYS_FOREVER_MS);

/* Start the thread */
int common_start(void) {
    /* configure deg_A and deg_B gpios */
    for (size_t i = 0; i < ARRAY_SIZE(gpios); i++) {
        if (!gpio_is_ready_dt(&gpios[i])) {
            printk("GPIO %zu isn't ready\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&gpios[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("GPIO %zu can't be configured.\n", i);
            return ret;
        }
    }

    k_thread_start(hb_tid);
    k_thread_start(hb_mon_tid);

    return 0;
}
