#include "can_bus.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

/* Acquire buses and tx pins from device tree */
static const struct device *devs[] = {DEVICE_DT_GET(DT_ALIAS(can_a)), DEVICE_DT_GET(DT_ALIAS(can_b))};
static const struct gpio_dt_spec tx_pins[] = {GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_tx_a_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_tx_b_gpios)};

/* Create SEQ counters for each ID */
static atomic_t seq_counters[] = {0, 0, 0, 0, 0, 0, 0};

/* Array of potential IDs */
static const uint16_t ids[] = {ID_CTRL, ID_FAULT, ID_HB_N1, ID_HB_N2, ID_HB_N3, ID_BLINK};

/* RX counts per bus */
static atomic_t rx_counts[] = {0, 0};

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data) {
    /* Recover bus */
    size_t i = (size_t)(uintptr_t)user_data;

    /* Increment count */
    atomic_inc(&rx_counts[i]);
}

int can_io_get_rx(int bus) {
    atomic_val_t count = atomic_clear(&rx_counts[bus]);
    return count;
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
