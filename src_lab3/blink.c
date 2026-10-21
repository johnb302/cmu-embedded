#include "blink.h"
#include "config.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* PC11 left, PC10 right on both nodes */
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

static void blink_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        struct can_frame frame;

        k_msgq_get(&blink_q, &frame, K_FOREVER);

        uint8_t st = frame.data[1];
        bool on = (frame.data[2] & 0x01) != 0;
        if (st > BLINK_HAZARD) {
            continue; /* not a state we know: ignore it, keep the last lamps */
        }

        bool left = on && (st == BLINK_LEFT || st == BLINK_HAZARD);
        bool right = on && (st == BLINK_RIGHT || st == BLINK_HAZARD);
        gpio_pin_set_dt(&lamp_left, left);
        gpio_pin_set_dt(&lamp_right, right);
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
