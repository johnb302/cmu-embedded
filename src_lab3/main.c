/* Main file for Lab3 of 18649 */
#include "config.h"
#include "can_bus.h"
#include "common.h"
#include "state.h"
#include "ctrl.h"
#include "n3.h"
#include "n2.h"
#include "relay.h"
#include "blink.h"
#include "button.h"
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

#if NODE_ID == ZONE_N3
    /* Before CAN starts: outputs braked, DRIVE waiting for its first CTRL */
    int n3_ret = n3_start();
    if (n3_ret < 0) {
        printk("N3 tasks not started.\n");
        return n3_ret;
    }
#endif

    /* The blinker lamps belong to the BLINK task now */
    int bl_ret = blink_start();
    if (bl_ret < 0) {
        printk("BLINK not started.\n");
        return bl_ret;
    }

    int ret = can_io_init();
    if (ret < 0) {
        printk("CAN buses could not be initialized.\n");
        return ret;
    }

    common_start();

#if NODE_ID == ZONE_N2
    /* After CAN, unlike n3_start(): FORCE_TX starts sending right away */
    int n2_ret = n2_start();
    if (n2_ret < 0) {
        printk("N2 tasks not started.\n");
        return n2_ret;
    }
    int rl_ret = relay_start();
    if (rl_ret < 0) {
        printk("RELAY not started.\n");
        return rl_ret;
    }
#endif

    /* After CAN: a press sends FAULT */
    int btn_ret = button_start();
    if (btn_ret < 0) {
        printk("TEST_BTN not started.\n");
        return btn_ret;
    }

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
#if NODE_ID == ZONE_N2
        printk("Relayed: %u\n", relay_take_count());
#endif
    }

    return 0;
}