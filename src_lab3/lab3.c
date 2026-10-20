/*
 * Lab 3 node firmware (N2 steering / N3 drivetrain, chosen by -DNODE_ID).
 * main() wires the modules together in dependency order, then drops to a
 * low priority and prints a once-a-second status line.
 *
 * Init order matters:
 *  1. zone_state first: ERR on, and every module can report faults.
 *  2. *_init(): register CAN handlers BEFORE can_bus_init() starts the
 *     controllers, so no frame arrives at an empty handler table.
 *  3. can_bus_init(): filters, controllers started, stall recovery.
 *  4. *_start(): configure outputs and start the task threads. They're
 *     K_THREAD_DEFINE'd with SYS_FOREVER_MS, so none runs before this.
 *  5. watchdog last, so a slow init can't trip it.
 */

#include <stdio.h>

#include <zephyr/kernel.h>

#include "blink.h"
#include "button.h"
#include "can_bus.h"
#include "control.h"
#include "heartbeat.h"
#include "n2_tasks.h"
#include "tasks.h"
#include "watchdog.h"
#include "zone_state.h"

static const char *const reason_name[] = {
	[0] = "-", [FAULT_TEST_BUTTON] = "test button", [FAULT_BAD_CTRL] = "bad CTRL",
	[FAULT_HB_LOSS] = "HB loss", [FAULT_REMOTE] = "remote FAULT",
};

static void report(void)
{
	uint8_t reason = zone_state_last_reason();

	for (uint8_t z = ZONE_N1; z <= ZONE_N3; z++) {
		int64_t a = can_bus_hb_age_ms(z, BUS_A);
		int64_t b = can_bus_hb_age_ms(z, BUS_B);

		if (z == NODE_ID) {
			continue;
		}
		printf("HB_N%u age A %s%lld ms, B %s%lld ms\n", z,
		       (a == INT64_MAX) ? "never " : "", (a == INT64_MAX) ? 0 : a,
		       (b == INT64_MAX) ? "never " : "", (b == INT64_MAX) ? 0 : b);
	}
	printf("zone %s (last: %s) | DEG A %d B %d | CTRL ok %u bad %u | relayed %u\n",
	       (zone_state_get() == ZONE_OK) ? "OK" : "ERROR",
	       (reason < ARRAY_SIZE(reason_name)) ? reason_name[reason] : "?",
	       heartbeat_bus_degraded(BUS_A), heartbeat_bus_degraded(BUS_B),
	       control_take_ok(), control_take_bad(), relay_take_count());
	printf("tx fail A %u B %u | restart A %u B %u\n",
	       can_bus_take_tx_fail(BUS_A), can_bus_take_tx_fail(BUS_B),
	       can_bus_take_restarts(BUS_A), can_bus_take_restarts(BUS_B));
}

#define CHECK(call)                                                                        \
	do {                                                                               \
		int _ret = (call);                                                         \
		if (_ret != 0) {                                                           \
			printf("%s failed [%d]\n", #call, _ret);                           \
			return 0;                                                          \
		}                                                                          \
	} while (0)

int main(void)
{
	CHECK(zone_state_init());

	heartbeat_init();
	control_init();
	blink_init();
	n2_tasks_init();

	CHECK(can_bus_init());

	CHECK(heartbeat_start());
	CHECK(control_start());
	CHECK(blink_start());
	CHECK(button_start());
	CHECK(n2_tasks_start());

	int wdt = watchdog_start();

	printf("Node %d up (watchdog %s)\n", NODE_ID, (wdt == 0) ? "on" : "FAILED");

	/* main runs at priority 0 by default: drop below every task before printing. */
	k_thread_priority_set(k_current_get(), REPORT_PRIO);

	while (1) {
		k_sleep(K_SECONDS(1));
		report();
	}
}
