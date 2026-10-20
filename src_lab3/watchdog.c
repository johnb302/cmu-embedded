/*
 * Independent watchdog (R4.5).
 *
 * Concepts:
 *  - The IWDG runs from its own 32 kHz RC oscillator, so it still resets
 *    the chip if the main clock or the whole scheduler dies. Once started
 *    it can't be stopped: only a reset turns it off.
 *  - Zephyr watchdog API: wdt_install_timeout() sets the timeout,
 *    wdt_setup() starts it, wdt_feed() restarts the countdown.
 *  - What to feed on: feeding from a low-priority thread proves the CPU
 *    isn't starved, and checking heartbeat_progress() proves HB_TX and
 *    HB_MON are still looping. If either stops, the feeds stop and the
 *    IWDG resets the node within WDT_TIMEOUT_MS.
 *  - WDT_OPT_PAUSE_HALTED_BY_DBG: don't reset while a debugger has the
 *    core halted.
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>

#include "heartbeat.h"
#include "tasks.h"
#include "watchdog.h"

#define WDT_TIMEOUT_MS 500
#define WDT_FEED_MS    100

static const struct device *const wdt = DEVICE_DT_GET(DT_NODELABEL(iwdg));
static int wdt_channel = -1;

static void wdt_thread(void *p1, void *p2, void *p3)
{
	uint32_t last = heartbeat_progress();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sleep(K_MSEC(WDT_FEED_MS));

		uint32_t now = heartbeat_progress();

		if (now != last) {
			wdt_feed(wdt, wdt_channel);
		}
		last = now;
	}
}

K_THREAD_DEFINE(wdt_tid, TASK_STACK_SIZE, wdt_thread, NULL, NULL, NULL,
		WDT_PRIO, 0, SYS_FOREVER_MS);

int watchdog_start(void)
{
	struct wdt_timeout_cfg cfg = {
		.window = { .min = 0, .max = WDT_TIMEOUT_MS },
		.callback = NULL, /* IWDG can't call back: it just resets */
		.flags = WDT_FLAG_RESET_SOC,
	};
	int ret;

	if (!device_is_ready(wdt)) {
		return -ENODEV;
	}
	wdt_channel = wdt_install_timeout(wdt, &cfg);
	if (wdt_channel < 0) {
		return wdt_channel;
	}
	ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret != 0) {
		return ret;
	}
	k_thread_start(wdt_tid);
	return 0;
}
