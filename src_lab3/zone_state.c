/*
 * Error state machine. See zone_state.h for the rules.
 *
 * Concepts:
 *  - Shared state across contexts. The handout requires it be "protected by
 *    something that is not volatile". volatile only stops the compiler
 *    caching a value; it doesn't make check-then-write sequences atomic.
 *  - k_mutex (K_MUTEX_DEFINE, k_mutex_lock/k_mutex_unlock). Everyone who
 *    touches this state runs in a thread: HB_TX, HB_MON, STEER/DRIVE,
 *    TEST_BTN, and the CAN handlers (the MCP2515 driver's thread). A mutex
 *    may block, so it can never be taken from an ISR; the test button's
 *    GPIO callback (an ISR) hands off to a thread with a semaphore.
 *  - Priority inheritance: if HB_MON (low) holds state_lock when the driver
 *    thread (-14) wants it, HB_MON runs at -14 until it unlocks. That bounds
 *    priority inversion to one short critical section: a blocking term in
 *    the RTA.
 *  - k_mutex is recursive for its owner, so update_peers() can call
 *    zone_state_fail() while holding the lock.
 *  - k_msgq: the FAULT frame is built here but sent by its own thread, so
 *    callers never block on CAN and the 20 ms minimum interval is enforced
 *    in one place.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "tasks.h"
#include "zone_state.h"

static K_MUTEX_DEFINE(state_lock);

static const struct gpio_dt_spec err_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), err_gpios);

/* All guarded by state_lock. */
static enum zone_state state = ZONE_ERROR;
static bool ctrl_seen;               /* a valid CTRL since we entered ERROR */
static bool test_latched;            /* test button pressed, not yet double-pressed */
static int64_t hold_until;           /* bad CTRL: earliest exit time */
static int64_t remote_since[ZONE_N3 + 1]; /* per zone: time of its FAULT, -1 = none */
static uint8_t last_reason;

/* Caller holds state_lock. */
static void set_state(enum zone_state s)
{
	state = s;
	gpio_pin_set_dt(&err_pin, s == ZONE_ERROR);
}

/* Caller holds state_lock. */
static bool remote_fault_active(void)
{
	for (int z = ZONE_N1; z <= ZONE_N3; z++) {
		if (remote_since[z] >= 0) {
			return true;
		}
	}
	return false;
}

enum zone_state zone_state_get(void)
{
	enum zone_state s;

	k_mutex_lock(&state_lock, K_FOREVER);
	s = state;
	k_mutex_unlock(&state_lock);
	return s;
}

void zone_state_fail(uint8_t reason)
{
	k_mutex_lock(&state_lock, K_FOREVER);
	set_state(ZONE_ERROR);
	last_reason = reason;
	ctrl_seen = false; /* leaving again needs a fresh valid CTRL */
	if (reason == FAULT_TEST_BUTTON) {
		test_latched = true;
	} else if (reason == FAULT_BAD_CTRL) {
		hold_until = k_uptime_get() + ERROR_HOLD_MS;
	}
	k_mutex_unlock(&state_lock);
}

/* ---- FAULT_TX: one small thread sends every FAULT this zone raises ---- */

K_MSGQ_DEFINE(fault_q, sizeof(uint8_t), 4, 1);

static void fault_tx_thread(void *p1, void *p2, void *p3)
{
	int64_t last_tx = -FAULT_MIN_INTERVAL_MS;
	uint8_t reason;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&fault_q, &reason, K_FOREVER);

		/* Catalog: at most one FAULT per 20 ms from a zone. */
		int64_t wait = last_tx + FAULT_MIN_INTERVAL_MS - k_uptime_get();

		if (wait > 0) {
			k_sleep(K_MSEC(wait));
		}
#if NODE_ID == ZONE_N3
		/*
		 * A bad CTRL reaches N2 and N3 at the same instant and both
		 * answer with FAULT 0x020. Two nodes starting the same ID in the
		 * same bit with different data can't be resolved by arbitration
		 * (bit error, both retry in lockstep). N3 waits a few frame
		 * times so the two FAULTs go out one after the other.
		 */
		k_sleep(K_MSEC(2));
#endif
		uint8_t data[CAN_BUS_DLC] = {
			[1] = NODE_ID, /* failing zone: us */
			[2] = reason,
		};

		can_bus_send(CAN_ID_FAULT, data);
		last_tx = k_uptime_get();
	}
}

K_THREAD_DEFINE(fault_tx_tid, TASK_STACK_SIZE, fault_tx_thread, NULL, NULL, NULL,
		FAULT_TX_PRIO, 0, SYS_FOREVER_MS);

void zone_state_report_fault(uint8_t reason)
{
	zone_state_fail(reason);
	(void)k_msgq_put(&fault_q, &reason, K_NO_WAIT); /* full = 4 already queued: fine */
}

/* ---- inputs from the other tasks ---- */

void zone_state_clear_test(void)
{
	k_mutex_lock(&state_lock, K_FOREVER);
	test_latched = false;
	k_mutex_unlock(&state_lock);
}

bool zone_state_test_latched(void)
{
	bool latched;

	k_mutex_lock(&state_lock, K_FOREVER);
	latched = test_latched;
	k_mutex_unlock(&state_lock);
	return latched;
}

void zone_state_note_valid_ctrl(void)
{
	k_mutex_lock(&state_lock, K_FOREVER);
	ctrl_seen = true;
	k_mutex_unlock(&state_lock);
}

void zone_state_remote_fault(uint8_t zone, uint8_t reason)
{
	ARG_UNUSED(reason);

	if (zone < ZONE_N1 || zone > ZONE_N3 || zone == NODE_ID) {
		return;
	}

	k_mutex_lock(&state_lock, K_FOREVER);
	remote_since[zone] = k_uptime_get();
	set_state(ZONE_ERROR);
	last_reason = FAULT_REMOTE;
	k_mutex_unlock(&state_lock);
}

void zone_state_peer_report(uint8_t zone, bool local_fault)
{
	if (zone < ZONE_N1 || zone > ZONE_N3 || local_fault) {
		return;
	}

	k_mutex_lock(&state_lock, K_FOREVER);
	/*
	 * Wait REMOTE_CLEAR_MS so a heartbeat built just before that zone
	 * failed (and still saying "no fault") can't clear the latch it just
	 * caused.
	 */
	if (remote_since[zone] >= 0 && k_uptime_get() - remote_since[zone] >= REMOTE_CLEAR_MS) {
		remote_since[zone] = -1;
	}
	k_mutex_unlock(&state_lock);
}

void zone_state_update_peers(bool all_peers_alive)
{
	k_mutex_lock(&state_lock, K_FOREVER);

	if (state == ZONE_OK) {
		if (!all_peers_alive) {
			zone_state_fail(FAULT_HB_LOSS); /* recursive lock: fine */
		}
	} else if (all_peers_alive && ctrl_seen && !test_latched &&
		   k_uptime_get() >= hold_until && !remote_fault_active()) {
		set_state(ZONE_OK);
	}

	k_mutex_unlock(&state_lock);
}

/*
 * FAULT (0x020) from another zone: b1 failing zone, b2 reason. Driver-thread
 * context: zone_state_remote_fault() only takes state_lock, which is short.
 */
static void on_fault(const struct can_frame *frame, enum can_bus_id bus)
{
	ARG_UNUSED(bus);

	zone_state_remote_fault(frame->data[1], frame->data[2]);
}

bool zone_state_local_fault(void)
{
	bool local;

	k_mutex_lock(&state_lock, K_FOREVER);
	local = test_latched || k_uptime_get() < hold_until;
	k_mutex_unlock(&state_lock);
	return local;
}

uint8_t zone_state_last_reason(void)
{
	uint8_t r;

	k_mutex_lock(&state_lock, K_FOREVER);
	r = last_reason;
	k_mutex_unlock(&state_lock);
	return r;
}

int zone_state_init(void)
{
	for (int z = 0; z <= ZONE_N3; z++) {
		remote_since[z] = -1;
	}

	/* ACTIVE = high = ERR on: we power up in ERROR (checkoff step 1). */
	int ret = gpio_pin_configure_dt(&err_pin, GPIO_OUTPUT_ACTIVE);

	if (ret != 0) {
		return ret;
	}
	can_bus_set_handler(CAN_ID_FAULT, on_fault); /* before can_bus_init(): see lab3.c */
	k_thread_start(fault_tx_tid);
	return 0;
}
