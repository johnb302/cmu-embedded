/* Main file for Lab3 of 18649 */
#include "config.h"
#include "can_bus.h"
#include "common.h"
#include "state.h"
#include "ctrl.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>

int main(void) {
    printk("lab3: node %d booted.\n", NODE_ID);

    int st_ret = state_init();
    if (st_ret < 0) {
        printk("State not initialized.\n");
        return st_ret;
    }

    struct gpio_dt_spec leds[] = {GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_left_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_right_gpios)};

    for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(&leds[i])) {
            printk("GPIO %zu isn't ready\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("GPIO %zu can't be configured.\n", i);
            return ret;
        }
    }

    int ret = can_io_init();
    if (ret < 0) {
        printk("CAN buses could not be initialized.\n");
        return ret;
    }

    common_start();
    if (common_wdg_start() < 0) {
        printk("watchdog not ready.\n");
        return -ENODEV;
    }

    k_thread_priority_set(k_current_get(), MAIN_PRIO);

    while (1) {
        k_msleep(1000);
        printk("CAN A counts: %d\n", can_io_get_rx(0));
        printk("CAN B counts: %d\n", can_io_get_rx(1));
        printk("CAN accepted counts: %d\n", can_io_get_accepted());
        printk("CAN dropped counts: %d\n", can_io_get_dropped());
    }

    return 0;
}