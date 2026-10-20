/*
 * TEST_BTN: the Nucleo's blue B1 button (PC13, the board's sw0 alias).
 * Single press: local fail within 10 ms (R2.5) and FAULT to every zone.
 * Double press (two presses within DOUBLE_MS while failed): clear it.
 *
 * Concepts:
 *  - GPIO interrupt: gpio_pin_interrupt_configure_dt(..., EDGE_TO_ACTIVE)
 *    plus gpio_init_callback()/gpio_add_callback(). The callback runs in
 *    the EXTI ISR, so it only does k_sem_give(): no mutex, no CAN, no
 *    printing (Part 5 rule: no SPI, logging or allocation inside an ISR).
 *  - Deferred work: the TEST_BTN thread waits in k_sem_take() and does the
 *    real work at its own priority. Response = ISR + one context switch +
 *    the thread, comfortably inside 10 ms.
 *  - Leading-edge debounce: act on the first edge, then ignore edges for
 *    DEBOUNCE_MS. A "wait until it stops bouncing" debounce would add its
 *    whole interval to the 10 ms budget.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "button.h"
#include "can_bus.h"
#include "tasks.h"
#include "zone_state.h"

#define DEBOUNCE_MS 50
#define DOUBLE_MS   500

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback btn_cb; /* must outlive the call: the driver keeps a pointer */

static K_SEM_DEFINE(btn_sem, 0, 1);

/* ISR context. */
static void on_press(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_sem_give(&btn_sem);
}

static void button_thread(void *p1, void *p2, void *p3)
{
	int64_t last_edge = -DEBOUNCE_MS;
	int64_t clear_press = -1; /* first press of a possible double, while latched */

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&btn_sem, K_FOREVER);

		int64_t now = k_uptime_get();

		if (now - last_edge < DEBOUNCE_MS) {
			continue; /* contact bounce */
		}
		last_edge = now;

		if (!zone_state_test_latched()) {
			zone_state_report_fault(FAULT_TEST_BUTTON);
			clear_press = -1;
		} else if (clear_press >= 0 && now - clear_press <= DOUBLE_MS) {
			zone_state_clear_test();
			clear_press = -1;
		} else {
			clear_press = now;
		}
	}
}

K_THREAD_DEFINE(button_tid, TASK_STACK_SIZE, button_thread, NULL, NULL, NULL,
		TEST_BTN_PRIO, 0, SYS_FOREVER_MS);

int button_start(void)
{
	int ret;

	if (!gpio_is_ready_dt(&btn)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}
	gpio_init_callback(&btn_cb, on_press, BIT(btn.pin));
	ret = gpio_add_callback(btn.port, &btn_cb);
	if (ret != 0) {
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		return ret;
	}
	k_thread_start(button_tid);
	return 0;
}
