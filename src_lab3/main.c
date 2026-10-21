/* Main file for Lab3 of 18649 */

#include "config.h"
#include "can_bus.h"
#include "common.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

int main(void) {
    printk("lab3: node %d booted.\n", NODE_ID);

    struct gpio_dt_spec leds[] = {GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), deg_a_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), deg_b_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), err_gpios),
                                GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_left_gpios), GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_right_gpios)};

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

    int count = 0;
    while (1) {
        gpio_pin_set_dt(&leds[count], 1);
        k_msleep(200);
        gpio_pin_set_dt(&leds[count++], 0);
        if (count == ARRAY_SIZE(leds)) {count = 0;}

        k_msleep(800);
        printk("CAN A counts: %d\n", can_io_get_rx(0));
        printk("CAN B counts: %d\n", can_io_get_rx(1));
        printk("CAN accepted counts: %d\n", can_io_get_accepted());
        printk("CAN dropped counts: %d\n", can_io_get_dropped());
        printk("Zone 1 Age on CAN A: %lld\n", can_io_hb_age_ms(ZONE_N1, 0));
        printk("Zone 1 Age on CAN B: %lld\n", can_io_hb_age_ms(ZONE_N1, 1));
        if (NODE_ID == ZONE_N2) {
            printk("Zone 3 Age on CAN A: %lld\n", can_io_hb_age_ms(ZONE_N3, 0));
            printk("Zone 3 Age on CAN B: %lld\n", can_io_hb_age_ms(ZONE_N3, 1));
        } else {
            printk("Zone 2 Age on CAN A: %lld\n", can_io_hb_age_ms(ZONE_N2, 0));
            printk("Zone 2 Age on CAN B: %lld\n", can_io_hb_age_ms(ZONE_N2, 1));
        }
    }

    return 0;
}