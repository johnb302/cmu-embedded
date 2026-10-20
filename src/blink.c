/*
 * BLINK_F / BLINK_R.
 *
 * Concepts:
 *  - Same handler -> k_msgq -> task pattern as CTRL.
 *  - k_msgq_get() with a timeout: the task wakes either for a frame or
 *    every BLINK_POLL_MS, which is how it notices the cockpit went quiet.
 *  - Sync by message, not by clock: each STM32 has its own crystal, so two
 *    local 500 ms timers drift apart (you saw the phase drift between the
 *    two boards' 1 s loops). Following the Pi's BLINK edge keeps front and
 *    rear within the receive-path difference, not the clock difference.
 *
 * Fallback: if this zone is in ERROR and no BLINK has arrived for
 * BLINK_LOST_MS (cockpit dead or unreachable), flash hazards locally so
 * the node is still in its fail-safe state (checkoff step 8, isolation).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "blink.h"
#include "can_bus.h"
#include "tasks.h"
#include "zone_state.h"

#define ZEPHYR_USER   DT_PATH(zephyr_user)
#define BLINK_POLL_MS 100
#define BLINK_LOST_MS 1200 /* > 2 x the Pi's 500 ms toggle */
#define LOCAL_HALF_MS 500  /* local hazard: 1 Hz */

static const struct gpio_dt_spec left = GPIO_DT_SPEC_GET(ZEPHYR_USER, blink_left_gpios);
static const struct gpio_dt_spec right = GPIO_DT_SPEC_GET(ZEPHYR_USER, blink_right_gpios);

K_MSGQ_DEFINE(blink_q, sizeof(struct can_frame), 1, 4);

static void on_blink(const struct can_frame *frame, enum can_bus_id bus)
{
	ARG_UNUSED(bus);

	if (k_msgq_put(&blink_q, frame, K_NO_WAIT) != 0) {
		k_msgq_purge(&blink_q);
		(void)k_msgq_put(&blink_q, frame, K_NO_WAIT);
	}
}

static void show(uint8_t state, bool phase)
{
	gpio_pin_set_dt(&left, phase && (state == BLINK_LEFT || state == BLINK_HAZARD));
	gpio_pin_set_dt(&right, phase && (state == BLINK_RIGHT || state == BLINK_HAZARD));
}

static void blink_thread(void *p1, void *p2, void *p3)
{
	struct can_frame frame;
	int64_t last_rx = k_uptime_get();
	int64_t last_local = 0;
	bool local_phase = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (k_msgq_get(&blink_q, &frame, K_MSEC(BLINK_POLL_MS)) == 0) {
			last_rx = k_uptime_get();
			show(frame.data[1], (frame.data[2] & 0x01) != 0);
			continue;
		}

		/* Timed out: is the cockpit gone while we're in ERROR? */
		int64_t now = k_uptime_get();

		if (zone_state_get() == ZONE_ERROR && now - last_rx > BLINK_LOST_MS &&
		    now - last_local >= LOCAL_HALF_MS) {
			local_phase = !local_phase;
			last_local = now;
			show(BLINK_HAZARD, local_phase);
		}
	}
}

K_THREAD_DEFINE(blink_tid, TASK_STACK_SIZE, blink_thread, NULL, NULL, NULL,
		BLINK_PRIO, 0, SYS_FOREVER_MS);

void blink_init(void)
{
	can_bus_set_handler(CAN_ID_BLINK, on_blink);
}

int blink_start(void)
{
	int ret = gpio_pin_configure_dt(&left, GPIO_OUTPUT_INACTIVE);

	ret |= gpio_pin_configure_dt(&right, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return -EIO;
	}
	k_thread_start(blink_tid);
	return 0;
}
