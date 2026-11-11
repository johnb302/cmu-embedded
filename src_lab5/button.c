#include "button.h"
#include "config.h"
#include "state.h"
#include "wcet.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* B1 from the board's devicetree (alias sw0). ACTIVE_LOW there, so _dt calls read 1 = pressed. */
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/* The driver keeps a POINTER to this struct (unlike can_add_rx_filter, which copies),
 * so it must outlive this function: static, never a local. */
static struct gpio_callback btn_cb;

/* ISR -> thread hand-off */
static K_SEM_DEFINE(btn_sem, 0, 1);

/* ISR context: no mutex, no CAN, no printk. Just wake TEST_BTN. */
static void on_press(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&btn_sem);
}

/* Debounce AFTER acting: the press is handled on its first edge (R2: local ERR within
 * 10 ms), then we wait until B1 has been released for BUTTON_DEBOUNCE_MS and drop
 * every bounce edge that piled up meanwhile. A bouncing release can't count as a press. */
static void wait_released(void) {
    int64_t released_since = -1;

    while (1) {
        k_msleep(5);
        int64_t now = k_uptime_get();
        if (gpio_pin_get_dt(&btn) == 0) {
            if (released_since < 0) {
                released_since = now;
            } else if (now - released_since >= BUTTON_DEBOUNCE_MS) {
                break;
            }
        } else {
            released_since = -1;
        }
    }
    k_sem_reset(&btn_sem);
}

static void button_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Only this thread sets or clears the test latch, so it can track it here */
    bool latched = false;
    int64_t last_press = -100000;

    while (1) {
        k_sem_take(&btn_sem, K_FOREVER);
        uint32_t t0 = wcet_start();
        int64_t now = k_uptime_get();

        if (!latched) {
            state_report_fault(FAULT_TEST_BUTTON);
            latched = true;
            last_press = -100000; /* the latching press doesn't count toward a double */
        } else if (now - last_press <= SELF_TEST_EXIT_WINDOW_MS) {
            state_clear_test();
            latched = false;
        } else {
            last_press = now; /* first press of a possible double */
        }
        wcet_record(WCET_TEST_BTN, t0); /* N3: includes the 2 ms FAULT delay on a latch */

        wait_released();
    }
}

K_THREAD_DEFINE(button_tid, TASK_STACK_SIZE, button_thread, NULL, NULL, NULL, TEST_BTN_PRIO, 0, SYS_FOREVER_MS);

int button_start(void) {
    if (!gpio_is_ready_dt(&btn)) {
        printk("B1 isn't ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        return ret;
    }
    gpio_init_callback(&btn_cb, on_press, BIT(btn.pin));
    ret = gpio_add_callback(btn.port, &btn_cb);
    if (ret < 0) {
        return ret;
    }

    k_thread_start(button_tid);
    return 0;
}
