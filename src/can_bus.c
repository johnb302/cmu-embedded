/*
 * Dual-bus CAN layer: catalog filters on both controllers, one dispatch
 * path with (ID, seq) dedup, and a send helper that queues every frame on
 * A then B with one sequence number. See can_bus.h for the contract.
 *
 * SKELETON. It compiles and runs as is but does nothing useful. Work in
 * TODO order; each step ends with a "Check:" you can run on the boards.
 * Reference solution: reference/stage1/can_bus.c.
 *
 * Docs you'll want open:
 *   CAN API:     https://docs.zephyrproject.org/latest/hardware/peripherals/can/controller.html
 *   GPIO API:    https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html
 *   Devicetree:  https://docs.zephyrproject.org/latest/build/dts/api-usage.html
 *   Mutexes:     https://docs.zephyrproject.org/latest/kernel/services/synchronization/mutexes.html
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "can_bus.h"
#include "tasks.h"


#define ZEPHYR_USER DT_PATH(zephyr_user)

#if NODE_ID == ZONE_N2
#define OTHER_ZONE ZONE_N3
#else
#define OTHER_ZONE ZONE_N2
#endif

/* Everything one controller needs. */
struct bus {
	const struct device *dev;
	struct gpio_dt_spec rx_tp; /* CAN_RX_x: toggles per frame, after the SPI read */
	struct gpio_dt_spec tx_tp; /* CAN_TX_x: toggles per frame queued */
	enum can_bus_id id;
	atomic_t tx_fail;
	atomic_t tx_ok_ms; /* uptime (32-bit ms) of the last frame this bus sent */
	atomic_t restarts;
	atomic_t state;
	struct can_state_change_callback state_cb;
};

/*
 * TODO 1.2 (devicetree): fill in .dev, .rx_tp and .tx_tp for both buses.
 *  - The overlay gives the controllers the aliases can-a and can-b. Look at
 *    DEVICE_DT_GET() and DT_ALIAS(). Note the alias is written can_a in C:
 *    devicetree names turn '-' into '_'.
 *  - The test-point pins are properties of the zephyr,user node
 *    (can-rx-a-gpios, ...). Look at GPIO_DT_SPEC_GET(ZEPHYR_USER, prop).
 * Question: these are resolved at compile time. What happens if you misspell
 * an alias: a runtime error, or a build error?
 */
static struct bus buses[BUS_COUNT] = {
	[BUS_A] = {
		.id = BUS_A,
		.dev = DEVICE_DT_GET(DT_ALIAS(can_a)),
		.rx_tp = GPIO_DT_SPEC_GET(ZEPHYR_USER, can_rx_a_gpios),
		.tx_tp = GPIO_DT_SPEC_GET(ZEPHYR_USER, can_tx_a_gpios),
	},
	[BUS_B] = {
		.id = BUS_B,
		.dev = DEVICE_DT_GET(DT_ALIAS(can_b)),
		.rx_tp = GPIO_DT_SPEC_GET(ZEPHYR_USER, can_rx_b_gpios),
		.tx_tp = GPIO_DT_SPEC_GET(ZEPHYR_USER, can_tx_b_gpios),
	},
};

#if NODE_ID == ZONE_N2
/* RELAY_TX test point (row 12): toggles per forwarded frame. */
static const struct gpio_dt_spec relay_tp = GPIO_DT_SPEC_GET(ZEPHYR_USER, relay_tx_gpios);
#endif

/* IDs this node receives: everything addressed to it in the catalog. */
static const uint16_t rx_ids[] = {
	CAN_ID_CTRL, CAN_ID_FAULT, CAN_ID_HB_N1, CAN_ID_HB(OTHER_ZONE), CAN_ID_BLINK,
};

/*
 * One sequence stream per (sender, ID). FAULT is sent by every zone, so it
 * gets one slot per sending zone, keyed on byte 1.
 */
enum slot {
	SLOT_CTRL,
	SLOT_BLINK,
	SLOT_HB_N1,
	SLOT_HB_N2,
	SLOT_HB_N3,
	SLOT_FORCE,
	SLOT_FAULT_N1,
	SLOT_FAULT_N2,
	SLOT_FAULT_N3,
	SLOT_COUNT,
};

struct seen {
	int64_t at_ms; /* -1: nothing accepted yet */
	uint8_t seq;
};

/*
 * Guards seen[] and hb_seen[]. Taken by rx_frame() (in the MCP2515 driver's
 * thread) and by can_bus_hb_age_ms() (in HB_MON). A k_mutex is allowed here
 * because both are threads; it may never be taken from an ISR. If HB_MON
 * holds it when a frame arrives, the driver thread (prio -14) waits, and
 * priority inheritance raises HB_MON to -14 until it unlocks: keep the
 * critical sections a few lines long. (That wait is a blocking term for
 * CAN_RX in your RTA.)
 */
static K_MUTEX_DEFINE(lock);
static struct seen seen[SLOT_COUNT];
static int64_t hb_seen[ZONE_N3 + 1][BUS_COUNT]; /* index by zone; -1: never */

static atomic_t tx_seq[SLOT_COUNT]; /* next seq per slot this node sends */

static const uint16_t handler_ids[] = {
	CAN_ID_CTRL, CAN_ID_FAULT, CAN_ID_HB_N1, CAN_ID_HB_N2,
	CAN_ID_HB_N3, CAN_ID_BLINK, CAN_ID_FORCE,
};
static can_bus_handler_t handlers[ARRAY_SIZE(handler_ids)];

/*
 * TODO 1.5: map a catalog ID to its slot. For CAN_ID_FAULT, fault_zone
 * (byte 1, 1..3) picks SLOT_FAULT_N1..N3. Unknown ID or zone: -EINVAL.
 */
static int slot_of(uint16_t id, uint8_t fault_zone)
{
	switch(id) {
		case CAN_ID_BLINK:
			return SLOT_BLINK;
		case CAN_ID_CTRL:
			return SLOT_CTRL;
		case CAN_ID_FAULT:
			if (fault_zone < ZONE_N1 || fault_zone > ZONE_N3) {return -EINVAL;}
			return SLOT_FAULT_N1 + (fault_zone - ZONE_N1);
		case CAN_ID_FORCE:
			return SLOT_FORCE;
		case CAN_ID_HB_N1:
			return SLOT_HB_N1;
		case CAN_ID_HB_N2:
			return SLOT_HB_N2;
		case CAN_ID_HB_N3:
			return SLOT_HB_N3;
		default:
			return -EINVAL;
	}
}

static can_bus_handler_t *handler_slot(uint16_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(handler_ids); i++) {
		if (handler_ids[i] == id) {
			return &handlers[i];
		}
	}
	return NULL;
}

/*
 * TODO 1.5 (dedup): return true if this slot accepted the same seq less
 * than DUP_WINDOW_MS ago. Otherwise record (seq, now) and return false.
 * The caller must already hold `lock` (rx_frame() calls k_mutex_lock()
 * before this and k_mutex_unlock() after), so don't lock it in here.
 * Questions for the report: why a time window and not "which bus it came
 * from"? What happens when the sender reboots and its counter restarts at 0?
 */
static bool is_duplicate(struct seen *s, uint8_t seq, int64_t now)
{
	if (s->seq == seq && (now - s->at_ms) < DUP_WINDOW_MS) {
		return true;
	}

	s->at_ms = now;
	s->seq = seq;

	return false;
}

/* Set by relay.c on N2 (stage 6): sees every first copy, after the handler. */
static can_bus_handler_t relay_tap;

/*
 * The one dispatch path. Registered with can_add_rx_filter() in TODO 1.4,
 * so it runs in the MCP2515 interrupt thread of whichever controller got
 * the frame (see the handler rules in can_bus.h).
 *
 * TODO 1.4: toggle this bus's CAN_RX test point (gpio_pin_toggle_dt), then
 *           if the frame is a heartbeat (0x100..0x102), store k_uptime_get()
 *           in hb_seen[zone][bus->id] between
 *           k_mutex_lock(&lock, K_FOREVER) and k_mutex_unlock(&lock).
 * TODO 1.5: drop frames whose DLC isn't CAN_BUS_DLC; find the slot; call
 *           is_duplicate() inside the same k_mutex_lock()/k_mutex_unlock()
 *           pair as the heartbeat store (one lock, one unlock, both
 *           updates). Then unlock, and only after unlocking call the
 *           registered handler (handler_slot()) on a first copy. Why
 *           unlock first? (The handler might take another mutex, or HB_MON
 *           might be waiting for this one.)
 *   Order matters: record the heartbeat BEFORE dedup. Why? (Think about
 *   what HB_MON needs to know about bus B when the A copy arrived first.)
 * Stage 6 (N2 RELAY): first copies also go to the relay tap, which only
 *   queues them (k_msgq_put(..., K_NO_WAIT)); the RELAY thread sends.
 */
static void rx_frame(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct bus *bus = user_data;
	gpio_pin_toggle_dt(&bus->rx_tp);
	/* Drop frames whose DLC is incorrect */
	if (frame->dlc != CAN_BUS_DLC) {return;}

	/* Else find the slot and call is_duplicate */
	int slot = slot_of(frame->id, frame->data[1]);
	if (slot < 0) {return;}

	int64_t now = k_uptime_get();
	k_mutex_lock(&lock, K_FOREVER);
	/* Check if frame is heartbeat */
	if (frame->id == 0x100) {
		hb_seen[ZONE_N1][bus->id] = now;
	} else if (frame->id == 0x101) {
		hb_seen[ZONE_N2][bus->id] = now;
	} else if (frame->id == 0x102) {
		hb_seen[ZONE_N3][bus->id] = now;
	}
	bool dup = is_duplicate(&seen[slot], CAN_SEQ(frame->data), now);
	k_mutex_unlock(&lock);

	if (!dup) {
		can_bus_handler_t *handler = handler_slot(frame->id);
		if (handler == NULL) {return;}
		if (*handler == NULL) {return;}
		(*handler)(frame, bus->id);
	}

	if (!dup && relay_tap != NULL) {
		relay_tap(frame, bus->id);
	}
}

/*
 * TX-done callback, passed to can_send(). error == 0 means the frame was
 * ACKed on the wire, so the bus is alive for us: remember when. Runs in the
 * driver thread: atomics only, no printing.
 */
static void tx_done(const struct device *dev, int error, void *user_data)
{
	struct bus *bus = user_data;

	ARG_UNUSED(dev);

	if (error == 0) {
		atomic_set(&bus->tx_ok_ms, (atomic_val_t)k_uptime_get_32());
	} else {
		atomic_inc(&bus->tx_fail);
	}
}

/*
 * Controller state callback (error-active / warning / passive / bus-off).
 * The driver hands back a pointer to the state_cb inside our struct bus;
 * CONTAINER_OF() subtracts that field's offset to recover the struct bus.
 */
static void state_changed(const struct device *dev, struct can_state_change_callback *cb,
			  enum can_state state, struct can_bus_err_cnt err_cnt)
{
	struct bus *bus = CONTAINER_OF(cb, struct bus, state_cb);

	ARG_UNUSED(dev);
	ARG_UNUSED(err_cnt);

	atomic_set(&bus->state, state);
}

/*
 * Stall recovery (R4.8: back within 100 ms of the jumper going back in).
 * A bus that went bus-off drops its pending frames without a TX-done
 * interrupt, and the Zephyr MCP2515 driver never frees those buffers:
 * every can_send() then returns -EAGAIN forever. Only can_stop() frees
 * them. So: if a bus has had nothing ACKed for TX_STALL_MS, restart it.
 * With a jumper pulled this repeats every TX_STALL_MS, which is harmless.
 * Worst-case recovery after reinsertion: TX_STALL_MS + RECOVER_PERIOD_MS.
 */
#define RECOVER_PERIOD_MS  20
#define RECOVER_STACK_SIZE 1024 /* RECOVER_PRIO is in tasks.h */

static void recover_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sleep(K_MSEC(RECOVER_PERIOD_MS));

		for (size_t i = 0; i < BUS_COUNT; i++) {
			struct bus *bus = &buses[i];
			uint32_t now = k_uptime_get_32();

			/* Unsigned subtraction stays correct across the 49-day wrap. */
			if ((now - (uint32_t)atomic_get(&bus->tx_ok_ms)) < TX_STALL_MS) {
				continue;
			}

			(void)can_stop(bus->dev);  /* aborts pending TX, frees the buffers */
			(void)can_start(bus->dev); /* filters and callbacks survive */
			atomic_set(&bus->tx_ok_ms, (atomic_val_t)k_uptime_get_32());
			atomic_inc(&bus->restarts);
		}
	}
}

/* Created at boot but not started (SYS_FOREVER_MS); can_bus_init() starts it. */
K_THREAD_DEFINE(can_recover_tid, RECOVER_STACK_SIZE, recover_thread, NULL, NULL, NULL,
		RECOVER_PRIO, 0, SYS_FOREVER_MS);

/*
 * TODO 1.3: queue one frame on one controller.
 *  - can_send(dev, frame, K_NO_WAIT, tx_done, bus). Why K_NO_WAIT and a
 *    callback rather than a blocking send? (Pull a jumper and think about
 *    what a blocking send on bus A does to bus B.)
 *  - Toggle bus->tx_tp right before it.
 *  - Wrap the toggle + send in k_sched_lock()/k_sched_unlock(). This is
 *    not a mutex and guards no data: it stops the scheduler switching
 *    threads, so the send runs non-preemptively. That bounds blocking on
 *    the shared SPI bus to one transaction in your RTA. Why does the SPI
 *    bus need that? (The SPI driver's own lock is a k_sem: no priority
 *    inheritance.)
 *  - Count a nonzero return in bus->tx_fail.
 */
static int queue_one(struct bus *bus, const struct can_frame *frame)
{
	/*
	 * The Zephyr MCP2515 driver uses ONE of the chip's three TX buffers
	 * (MCP2515_TX_CNT is 1 in can_mcp2515.h, to keep frames in order), so a
	 * second send while one is still on the wire would fail with
	 * K_NO_WAIT. Wait up to CAN_TX_WAIT_MS for the buffer: long enough for
	 * a few frame times, short enough that a dead bus (frame never ACKed)
	 * costs the sender at most that per send. If can_send() has to wait,
	 * the thread blocks and the scheduler lock is suspended until it runs
	 * again, so waiting doesn't stall other threads.
	 */
	k_sched_lock();
	gpio_pin_toggle_dt(&bus->tx_tp);
	int ret = can_send(bus->dev, frame, K_MSEC(CAN_TX_WAIT_MS), &tx_done, bus);
	if (ret < 0) {
		atomic_add(&bus->tx_fail, 1);
		k_sched_unlock();
		return ret;
	}
	k_sched_unlock();

	return 0;
}

/*
 * TODO 1.3: build a struct can_frame (id, dlc = CAN_BUS_DLC, flags 0 for a
 * standard ID), copy data in, put the next seq for this ID in the low
 * nibble of byte 0 (keep the high nibble), and queue it on A then B.
 * One new seq per message, shared by both copies: atomic_inc() on
 * tx_seq[slot] returns the old value. Return 0 if either bus queued it.
 * Check: on the Pi, `candump -c any` shows 0x101/0x102 on both interfaces
 * with byte 0 counting 0..F, the same on both copies.
 * (Until TODO 1.5, slot_of() returns -EINVAL; temporarily use slot 0.)
 */
int can_bus_send(uint16_t id, const uint8_t data[CAN_BUS_DLC])
{
	struct can_frame frame = {
		.id = id,
		.dlc = CAN_BUS_DLC,
		.flags = 0
	};
	memcpy(frame.data, data, CAN_BUS_DLC);

	/* Put the SEQ in low nibble of byte 0 */
	int slot = slot_of(frame.id, frame.data[1]); /* FAULT: byte 1 is our own zone */
	if (slot < 0) {
		return -EINVAL;
	}
	atomic_val_t prev_seq = atomic_inc(&tx_seq[slot]);
	frame.data[0] = (frame.data[0] & ~CAN_SEQ_MASK) | (prev_seq & CAN_SEQ_MASK);

	int failed = 0;
	int ret;
	for (size_t i = 0; i < BUS_COUNT; i++) {
		ret = queue_one(&buses[i], &frame);
		if (ret < 0) {
			failed++;
		}
	}

	if (failed == BUS_COUNT) {
		return -EIO;
	}

	return 0;
}

/*
 * RELAY (N2): queue a received frame on the other bus with its original
 * seq, marked relayed. Receivers dedup it against the original like any
 * other copy; RELAY never forwards a frame that is already marked.
 */
int can_bus_forward(enum can_bus_id to, const struct can_frame *frame)
{
	struct can_frame copy = *frame;

	if (to >= BUS_COUNT) {
		return -EINVAL;
	}

	copy.data[0] |= CAN_FLAG_RELAYED;
#if NODE_ID == ZONE_N2
	gpio_pin_toggle_dt(&relay_tp);
#endif
	return queue_one(&buses[to], &copy);
}

void can_bus_set_relay_tap(can_bus_handler_t tap)
{
	relay_tap = tap;
}

int can_bus_set_handler(uint16_t id, can_bus_handler_t handler)
{
	can_bus_handler_t *slot = handler_slot(id);

	if (slot == NULL) {
		return -EINVAL;
	}
	*slot = handler;
	return 0;
}

/*
 * TODO 1.4: reject a bad zone/bus first (before locking). Then copy
 * hb_seen[zone][bus] into a local between k_mutex_lock(&lock, K_FOREVER)
 * and k_mutex_unlock(&lock), and do the arithmetic on the copy after
 * unlocking: return INT64_MAX if it's -1 (never), otherwise k_uptime_get()
 * minus it. Every return path after a lock must unlock first.
 */
int64_t can_bus_hb_age_ms(uint8_t zone, enum can_bus_id bus)
{
	if (zone > ZONE_N3 || zone < ZONE_N1 || bus >= BUS_COUNT) {
		return INT64_MAX;
	}

	k_mutex_lock(&lock, K_FOREVER);
	int64_t copy = hb_seen[zone][bus];
	k_mutex_unlock(&lock);

	if (copy == -1) {
		return INT64_MAX;
	}

	return (k_uptime_get() - copy);
}

enum can_state can_bus_state(enum can_bus_id bus)
{
	return (enum can_state)atomic_get(&buses[bus].state);
}

uint32_t can_bus_take_tx_fail(enum can_bus_id bus)
{
	return (uint32_t)atomic_clear(&buses[bus].tx_fail);
}

uint32_t can_bus_take_restarts(enum can_bus_id bus)
{
	return (uint32_t)atomic_clear(&buses[bus].restarts);
}

/* CRC-8, poly 0x07, init 0x00, MSB first: identical to the Pi's crc8(). */
uint8_t can_bus_crc8(const uint8_t *buf, size_t len)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

/*
 * Bring both controllers up.
 *
 * TODO 1.2, for each bus:
 *   1. device_is_ready(bus->dev), else -ENODEV. (The driver's own init ran
 *      at boot. Careful: it never reads a register back, so "ready" does
 *      not prove SPI works. That's how the MISO/MOSI swap hid.)
 *   2. gpio_pin_configure_dt() both test points as GPIO_OUTPUT_INACTIVE.
 *   3. can_init_state_change_callback() + can_add_state_change_callback().
 *   4. can_start(). Then set bus->state to CAN_STATE_ERROR_ACTIVE.
 *   Check: in the shell, `can show can0` / `can1` report error-active, and
 *   `can start can0` now says it's already started (-EALREADY = -120).
 * TODO 1.4, before can_start(): one can_add_rx_filter() per entry in
 *   rx_ids[], callback rx_frame, user data = bus. A filter matches when
 *   (frame.id & mask) == (filter.id & mask); what mask accepts exactly one
 *   standard ID? (Look at CAN_STD_ID_MASK.) Also initialize every
 *   seen[].at_ms and hb_seen[][] to -1 first.
 *   Check: the lab3.c report shows the other STM32's heartbeat age < 20 ms
 *   on both buses.
 * TODO 1.6, at the end: set every tx_ok_ms to now and
 *   k_thread_start(can_recover_tid).
 */
int can_bus_init(void)
{
	/* Initialize seen and hb_seen */
	memset(seen, -1, sizeof(seen));
	memset(hb_seen, -1, sizeof(hb_seen));

	for (size_t i = 0; i < BUS_COUNT; i++) {
		struct bus *bus = &buses[i];

		/* Check if both CAN buses are ready */
		if (!device_is_ready(bus->dev)) {
			printk("CAN %zu is not ready.\n", i);
			return -ENODEV;
		}

		/* Configure both gpio tx and rx */
		int ret = gpio_pin_configure_dt(&bus->rx_tp, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}

		ret = gpio_pin_configure_dt(&bus->tx_tp, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}

		/* Initialize state change call back */
		can_init_state_change_callback(&bus->state_cb, &state_changed);

		/* Add the callback */
		int cb_ret = can_add_state_change_callback(bus->dev, &bus->state_cb);
		if (cb_ret < 0) {
			return cb_ret;
		}

		/* Add can rx filter */
		for (size_t j = 0; j < ARRAY_SIZE(rx_ids); j++) {
			const struct can_filter filter = {
        		.flags = 0U,               /* standard (11-bit) ID */
        		.id = rx_ids[j],
        		.mask = CAN_STD_ID_MASK,   /* compare all 11 bits: match exactly this ID */
			};

			int rx_ret = can_add_rx_filter(bus->dev, rx_frame, bus, &filter);
			if (rx_ret < 0) {
				return rx_ret;
			}
		}

		/* Start the bus */
		int start_ret = can_start(bus->dev);
		if (start_ret < 0) {
			printk("can_start bus %zu failed [%d]\n", i, start_ret);
			return start_ret;
		}
		atomic_set(&bus->state, CAN_STATE_ERROR_ACTIVE);
	}

#if NODE_ID == ZONE_N2
	gpio_pin_configure_dt(&relay_tp, GPIO_OUTPUT_INACTIVE);
#endif

	/* Stall recovery: nothing ACKed yet, so count from now. */
	for (size_t i = 0; i < BUS_COUNT; i++) {
		atomic_set(&buses[i].tx_ok_ms, (atomic_val_t)k_uptime_get_32());
	}
	k_thread_start(can_recover_tid);

	return 0;
}
