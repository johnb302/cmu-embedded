/*
 * FORCE_TX and RELAY (steering zone only).
 *
 * RELAY design (Part 4 questions):
 *  - Who relays: only N2, the middle node. N1 and N3 never forward.
 *  - When: forward a frame from sender S, received on bus X, onto bus Y
 *    only if S's heartbeat hasn't arrived on Y for HB_TIMEOUT_MS (S can't
 *    reach Y itself) and some other zone IS heard on Y (Y is worth sending
 *    on). In scenario (c), N1 is off B and N3 off A, so N2 forwards N1's
 *    CTRL/BLINK/HB_N1/FAULT from A to B and N3's HB_N3/FAULT from B to A.
 *    With both buses healthy it forwards nothing: no extra bus load.
 *  - Cost of detecting first: the first relayed frame waits for the 60 ms
 *    detection (R4.7). The alternative, always relaying, needs no detection
 *    but doubles N2's transmit load and the receivers' interrupts all the
 *    time. Detecting first is cheaper and the 60 ms is inside every
 *    deadline except the brake chain's, which the report has to discuss.
 *  - Loops: a relayed frame carries CAN_FLAG_RELAYED and RELAY ignores
 *    marked frames, and only N2 relays, so nothing can bounce back.
 *  - Duplicates: the relayed copy keeps the original seq, so a receiver
 *    that also got the original drops it in dedup like any second copy.
 *  - Priority: RELAY is on the relayed brake path, so it sits above STEER
 *    and BLINK_F (tasks.h). The tap only queues; the thread sends.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "can_bus.h"
#include "n2_tasks.h"
#include "tasks.h"

static atomic_t relayed;

#if NODE_ID == ZONE_N2

#define FORCE_PERIOD_MS 50
#define RELAY_QUEUE_LEN 8

/* ------------------------------ FORCE_TX ------------------------------ */

static void force_thread(void *p1, void *p2, void *p3)
{
	int64_t next = k_uptime_get();
	int8_t counter = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		uint8_t data[CAN_BUS_DLC] = { [1] = (uint8_t)counter++ };

		can_bus_send(CAN_ID_FORCE, data);

		next += FORCE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}

K_THREAD_DEFINE(force_tid, TASK_STACK_SIZE, force_thread, NULL, NULL, NULL,
		FORCE_TX_PRIO, 0, SYS_FOREVER_MS);

/* -------------------------------- RELAY -------------------------------- */

struct relay_item {
	struct can_frame frame;
	enum can_bus_id from;
};

K_MSGQ_DEFINE(relay_q, sizeof(struct relay_item), RELAY_QUEUE_LEN, 4);

/* The zone that sent this frame, or 0 if we don't relay it. */
static uint8_t sender_of(const struct can_frame *f)
{
	switch (f->id) {
	case CAN_ID_CTRL:
	case CAN_ID_BLINK:
	case CAN_ID_HB_N1:
		return ZONE_N1;
	case CAN_ID_HB_N3:
		return ZONE_N3;
	case CAN_ID_FAULT:
		return f->data[1]; /* FAULT comes from the failing zone itself */
	default:
		return 0;
	}
}

static bool someone_else_on(enum can_bus_id bus, uint8_t sender)
{
	for (uint8_t z = ZONE_N1; z <= ZONE_N3; z++) {
		if (z != NODE_ID && z != sender && can_bus_hb_age_ms(z, bus) < HB_TIMEOUT_MS) {
			return true;
		}
	}
	return false;
}

/* Driver-thread context (called by can_bus after dedup): queue only. */
static void relay_tap(const struct can_frame *frame, enum can_bus_id bus)
{
	struct relay_item item = { .frame = *frame, .from = bus };

	if (CAN_RELAYED(frame->data)) {
		return;
	}
	(void)k_msgq_put(&relay_q, &item, K_NO_WAIT); /* full: drop, it's stale anyway */
}

static void relay_thread(void *p1, void *p2, void *p3)
{
	struct relay_item item;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&relay_q, &item, K_FOREVER);

		uint8_t sender = sender_of(&item.frame);
		enum can_bus_id to = (item.from == BUS_A) ? BUS_B : BUS_A;

		if (sender == 0 || sender == NODE_ID) {
			continue;
		}
		if (can_bus_hb_age_ms(sender, to) < HB_TIMEOUT_MS) {
			continue; /* the sender reaches that bus itself */
		}
		if (!someone_else_on(to, sender)) {
			continue; /* nobody to deliver to over there */
		}
		if (can_bus_forward(to, &item.frame) == 0) {
			atomic_inc(&relayed);
		}
	}
}

K_THREAD_DEFINE(relay_tid, TASK_STACK_SIZE, relay_thread, NULL, NULL, NULL,
		RELAY_PRIO, 0, SYS_FOREVER_MS);

void n2_tasks_init(void)
{
	can_bus_set_relay_tap(relay_tap);
}

int n2_tasks_start(void)
{
	k_thread_start(force_tid);
	k_thread_start(relay_tid);
	return 0;
}

#else /* N3: nothing to do */

void n2_tasks_init(void)
{
}

int n2_tasks_start(void)
{
	return 0;
}

#endif

uint32_t relay_take_count(void)
{
	return (uint32_t)atomic_clear(&relayed);
}
