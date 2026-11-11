#include "can_bus.h"
#include "state.h"
#include "ctrl.h"
#include "blink.h"
#include "relay.h"
#include "wcet.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

/* Acquire buses and tx pins from device tree */
static const struct device *devs[] = {DEVICE_DT_GET(DT_ALIAS(can_a)), DEVICE_DT_GET(DT_ALIAS(can_b))};
static const struct gpio_dt_spec tx_pins[] = {GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_tx_a_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_tx_b_gpios)};
static const struct gpio_dt_spec rx_pins[] = {GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_rx_a_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_rx_b_gpios)};

/* Create SEQ counters for each ID */
static atomic_t seq_counters[] = {0, 0, 0, 0, 0, 0, 0};

/* Array of potential IDs */
static const uint16_t ids[] = {ID_CTRL, ID_FAULT, ID_HB_N1, ID_HB_N2, ID_HB_N3, ID_BLINK};

/* RX counts per bus */
static atomic_t rx_counts[] = {0, 0};

/* Table of time data from nodes last heard from */
static int64_t last_heard[ZONE_N3 + 1][2] = {{-100000, -100000}, {-100000, -100000}, {-100000, -100000}, {-100000, -100000}};
static K_MUTEX_DEFINE(heard_lock);

/* Struct of last heard message on either bus */
struct last_rx {
    uint8_t seq;
    int64_t time;
};
static struct last_rx last_rx[10] = {{.seq = 0, .time=-100000}, {.seq = 0, .time=-100000}, {.seq = 0, .time=-100000}, {.seq = 0, .time=-100000},
                                     {.seq = 0, .time=-100000}, {.seq = 0, .time=-100000}, {.seq = 0, .time=-100000}, {.seq = 0, .time=-100000},
                                     {.seq = 0, .time=-100000}, {.seq = 0, .time=-100000}};

/* Lock for editing shared variable last_rx */
static K_MUTEX_DEFINE(dedup_lock);

/* Frame counters */
static atomic_t rx_accepted = 0;
static atomic_t rx_dropped = 0;

/* Function definition for get_counter in rx_cb */
static int get_counter(uint16_t id);

static void rx_dispatch(const struct device *dev, struct can_frame *frame, void *user_data) {
    /* Recover bus */
    size_t i = (size_t)(uintptr_t)user_data;

    int zone = 0;
    switch(frame->id) {
        case ID_HB_N1:
            zone = ZONE_N1;
            break;
        case ID_HB_N2:
            zone = ZONE_N2;
            break;
        case ID_HB_N3:
            zone = ZONE_N3;
            break;
        default:
            break;
    }

    if (zone != 0) {
        /* Record heartbeat's heard @ time */
        k_mutex_lock(&heard_lock, K_FOREVER);
        last_heard[zone][i] = k_uptime_get();
        k_mutex_unlock(&heard_lock);
    }

    /* Increment count */
    atomic_inc(&rx_counts[i]);

    /* If the frame id is fault, it could come from many places */
    int slot = 0;
    if (frame->id == ID_FAULT) {
        uint8_t sender = frame->data[1];
        if (sender < 1 || sender > 3) {return;}
        slot = 6 + sender;
    } else {
        slot = get_counter(frame->id);
        if (slot < 0) {return;}
    }

    k_mutex_lock(&dedup_lock, K_FOREVER);
    /* If this frame has the same seq as the sender and is within the dedup window, drop it*/
    if (last_rx[slot].seq == (frame->data[0] & SEQ_MASK) && k_uptime_get() < last_rx[slot].time + DEDUP_WINDOW_MS) {
        k_mutex_unlock(&dedup_lock);
        atomic_inc(&rx_dropped);
        return;
    }
    last_rx[slot].seq = (frame->data[0] & SEQ_MASK);
    last_rx[slot].time = k_uptime_get();
    k_mutex_unlock(&dedup_lock);
    atomic_inc(&rx_accepted);

    /* Let state machine update */
    switch(frame->id) {
        case ID_CTRL:
            /* DRIVE (N3) / STEER (N2) validates it at its own priority */
            ctrl_post(frame);
            break;
        case ID_HB_N1:
        case ID_HB_N2:
        case ID_HB_N3:
            state_peer_report(zone, frame->data[3] != 0);
            break;
        case ID_FAULT:
            state_remote_fault(frame->data[1]);
            break;
        case ID_BLINK:
            blink_post(frame);
            break;
        default:
            break;
    }

#if NODE_ID == ZONE_N2
    /* RELAY decides whether the other bus needs this frame (relay.c) */
    relay_offer(frame, i);
#endif
}

/* The driver's RX callback (its thread, after the SPI read): CAN_RX_A/B toggles
 * here, the moment our code sees the frame (handout test-point definition). */
static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data) {
    size_t bus = (size_t)(uintptr_t)user_data;
    uint32_t t0 = wcet_start();

    gpio_pin_toggle_dt(&rx_pins[bus]);
    rx_dispatch(dev, frame, user_data);
    wcet_record(WCET_CAN_RX, t0);
}

int can_io_get_rx(int bus) {
    atomic_val_t count = atomic_clear(&rx_counts[bus]);
    return count;
}

int can_io_get_dropped(void) {
    return atomic_clear(&rx_dropped);
}

int can_io_get_accepted(void) {
    return atomic_clear(&rx_accepted);
}

int can_io_init(void) {
    /* Loop over and initialize buses */
    for (size_t i = 0; i < ARRAY_SIZE(devs); i++) {
        if (!gpio_is_ready_dt(&tx_pins[i])) {
            printk("TX on CAN %zu not ready.\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&tx_pins[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("TX on CAN %zu could not be configured.\n", i);
            return ret;
        }

        /* RX test point: must be ready before can_start() lets rx_cb run */
        if (!gpio_is_ready_dt(&rx_pins[i])) {
            printk("RX on CAN %zu not ready.\n", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&rx_pins[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("RX on CAN %zu could not be configured.\n", i);
            return ret;
        }

        if (!device_is_ready(devs[i])) {
            printk("CAN %zu is not ready.\n", i);
            return -ENODEV;
        }

        /* Creating CAN filters by looping over all possible IDs */
        for (size_t j = 0; j < ARRAY_SIZE(ids); j++) {
            /* Filter should accept frame when filter.id and frame.id match */
            struct can_filter filter = {
                .mask = CAN_STD_ID_MASK,
                .id = ids[j],
            };

            int filt_ret = can_add_rx_filter(devs[i], rx_cb, (void *)(uintptr_t)i, &filter);
            if (filt_ret < 0) {
                printk("Unable to add filter to CAN %zu.\n", i);
                return filt_ret;
            }
        }

        int start_ret = can_start(devs[i]);
        if (start_ret < 0) {
            printk("CAN %zu could not start.\n", i);
            return start_ret;
        }
    }

    return 0;
}

static void tx_cb(const struct device *dev, int error, void *user_data) {
    return;
}

static int get_counter(uint16_t id) {
    switch(id) {
        case ID_CTRL:
            return 0;
        case ID_FAULT:
            return 1;
        case ID_HB_N1:
            return 2;
        case ID_HB_N2:
            return 3;
        case ID_HB_N3:
            return 4;
        case ID_BLINK:
            return 5;
        case ID_FORCE:
            return 6;
        default:
            return -1;
    }
}

int can_io_send(uint16_t id, const uint8_t data[FRAME_DLC]) {
    /* Build the frame */
    struct can_frame frame = {
        .id = id,
        .dlc = FRAME_DLC,
        .flags = 0,
    };
    memcpy(frame.data, data, FRAME_DLC);

    
    int counter = get_counter(id);
    if (counter < 0) {
        printk("ID not in the known list.\n");
        return counter;
    }

    /* Clear the bottom half of byte 0, then OR with SEQ */
    atomic_val_t prev = atomic_inc(&seq_counters[counter]);
    frame.data[0] = (frame.data[0] & ~SEQ_MASK) | (SEQ_MASK & prev);

    int failed = 0;
    int ret;
    for (size_t i = 0; i < ARRAY_SIZE(devs); i++) {
        /* Toggle the TX pin */
        gpio_pin_toggle_dt(&tx_pins[i]);

        /* Send CAN frame */
        ret = can_send(devs[i], &frame, K_MSEC(CAN_TX_WAIT_MS), tx_cb, NULL);
        if (ret < 0) {
            failed++;
        }
    }

    if (failed == ARRAY_SIZE(devs)) {
        printk("Failed to send frame on both buses.\n");
        return ret;
    }

    return 0;
}

/* RELAY: re-send a received frame on ONE bus, unchanged except for the relayed
 * flag. The seq is kept, so receivers dedup it against any other copy. */
int can_io_forward(int bus, const struct can_frame *f) {
    struct can_frame frame = *f;
    frame.data[0] |= FLAG_RELAYED;

    gpio_pin_toggle_dt(&tx_pins[bus]);
    return can_send(devs[bus], &frame, K_MSEC(CAN_TX_WAIT_MS), tx_cb, NULL);
}

int64_t can_io_hb_age_ms(uint8_t zone, int bus) {
    k_mutex_lock(&heard_lock, K_FOREVER);
    int64_t heard = last_heard[zone][bus];
    k_mutex_unlock(&heard_lock);

    return (k_uptime_get() - heard);
}
