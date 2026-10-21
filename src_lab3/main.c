/* Main file for Lab3 of 18649 */

#include "config.h"
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

    int count = 0;
    while (1) {
        gpio_pin_set_dt(&leds[count], 1);
        k_msleep(200);
        gpio_pin_set_dt(&leds[count++], 0);
        if (count == ARRAY_SIZE(leds)) {count = 0;}
    }

    return 0;
}