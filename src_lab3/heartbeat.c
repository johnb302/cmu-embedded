/*
 * HB_TX and HB_MON.
 *
 * Concepts:
 *  - K_THREAD_DEFINE: a statically created thread with a fixed priority
 *    (tasks.h). Created at boot but not started (SYS_FOREVER_MS); started
 *    by heartbeat_start() after the CAN layer is up.
 *  - Periodic release without drift: k_sleep(K_MSEC(20)) after the work
 *    makes the period 20 ms + work + preemption, and the error accumulates.
 *    Sleeping until an absolute time does not:
 *        next += PERIOD; k_sleep(K_TIMEOUT_ABS_MS(next));
 *    That's how HB_TX holds R4.5's 20 ms +/- 10 % under load.
 *  - Publishing results with atomics: HB_MON writes degraded[] with
 *    atomic_set, HB_TX and the console read it with atomic_get. One word,
 *    one writer: no lock needed.
 *
 * DEG rule (R4.7): bus X is degraded on this node when NO other zone's
 * heartbeat has arrived on X for HB_TIMEOUT_MS. "No zone", not "any zone":
 * if only N2 is pulled off A, N1 and N3 still hear each other on A, so
 * only N2 lights DEG_A (checkoff step 5).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "can_bus.h"
#include "heartbeat.h"
#include "tasks.h"
#include "zone_state.h"

#define ZEPHYR_USER DT_PATH(zephyr_user)

/* The zones this node listens to: N1 and the other STM32. */
#if NODE_ID == ZONE_N2
static const uint8_t peers[] = { ZONE_N1, ZONE_N3 };
#else
static const uint8_t peers[] = { ZONE_N1, ZONE_N2 };
#endif

static atomic_t degraded[BUS_COUNT];
static atomic_t progress;

static const struct gpio_dt_spec deg_led[BUS_COUNT] = {
	[BUS_A] = GPIO_DT_SPEC_GET(ZEPHYR_USER, deg_a_gpios),
	[BUS_B] = GPIO_DT_SPEC_GET(ZEPHYR_USER, deg_b_gpios),
};

static void hb_tx_thread(void *p1, void *p2, void *p3)
{
	int64_t next = k_uptime_get();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		uint8_t data[CAN_BUS_DLC] = { 0 };

		data[1] = (uint8_t)zone_state_get();
		data[2] = (heartbeat_bus_degraded(BUS_A) ? 0 : HB_HEALTH_A) |
			  (heartbeat_bus_degraded(BUS_B) ? 0 : HB_HEALTH_B);
		data[3] = zone_state_local_fault() ? 1 : 0;
		can_bus_send(CAN_ID_HB(NODE_ID), data);
		atomic_inc(&progress);

		next += HB_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}

static void hb_mon_thread(void *p1, void *p2, void *p3)
{
	int64_t next = k_uptime_get();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		bool all_alive = true;

		for (size_t b = 0; b < BUS_COUNT; b++) {
			bool any_heard = false;

			for (size_t p = 0; p < ARRAY_SIZE(peers); p++) {
				if (can_bus_hb_age_ms(peers[p], b) < HB_TIMEOUT_MS) {
					any_heard = true;
				}
			}
			atomic_set(&degraded[b], !any_heard);
			gpio_pin_set_dt(&deg_led[b], !any_heard);
		}

		/* R4.5: a zone is failed only when missed on BOTH buses. */
		for (size_t p = 0; p < ARRAY_SIZE(peers); p++) {
			if (can_bus_hb_age_ms(peers[p], BUS_A) >= HB_TIMEOUT_MS &&
			    can_bus_hb_age_ms(peers[p], BUS_B) >= HB_TIMEOUT_MS) {
				all_alive = false;
			}
		}
		zone_state_update_peers(all_alive);
		atomic_inc(&progress);

		next += HB_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}

K_THREAD_DEFINE(hb_tx_tid, TASK_STACK_SIZE, hb_tx_thread, NULL, NULL, NULL,
		HB_TX_PRIO, 0, SYS_FOREVER_MS);
K_THREAD_DEFINE(hb_mon_tid, TASK_STACK_SIZE, hb_mon_thread, NULL, NULL, NULL,
		HB_MON_PRIO, 0, SYS_FOREVER_MS);

/* Driver-thread context: pass the peer's reported state on, nothing else. */
static void on_peer_hb(const struct can_frame *frame, enum can_bus_id bus)
{
	ARG_UNUSED(bus);

	zone_state_peer_report(frame->id - CAN_ID_HB_N1 + ZONE_N1, frame->data[3] != 0);
}

void heartbeat_init(void)
{
	for (size_t p = 0; p < ARRAY_SIZE(peers); p++) {
		can_bus_set_handler(CAN_ID_HB(peers[p]), on_peer_hb);
	}
}

int heartbeat_start(void)
{
	for (size_t b = 0; b < BUS_COUNT; b++) {
		int ret = gpio_pin_configure_dt(&deg_led[b], GPIO_OUTPUT_INACTIVE);

		if (ret != 0) {
			return ret;
		}
	}

	/* Started here, not at boot: before can_bus_init() there's no CAN to send on. */
	k_thread_start(hb_tx_tid);
	k_thread_start(hb_mon_tid);
	return 0;
}

bool heartbeat_bus_degraded(enum can_bus_id bus)
{
	return atomic_get(&degraded[bus]) != 0;
}

uint32_t heartbeat_progress(void)
{
	return (uint32_t)atomic_get(&progress);
}
