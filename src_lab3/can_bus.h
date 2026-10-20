#ifndef CAN_BUS_H_
#define CAN_BUS_H_

/*
 * Dual-bus CAN layer shared by N2 and N3 (Lab 3 Part 5).
 *
 * The declarations below are the contract the rest of the firmware (and the
 * Pi) is written against. The EXERCISE 1.1 macros are yours.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#ifndef NODE_ID
#error "Build with -DNODE_ID=2 (steering) or -DNODE_ID=3 (drivetrain)"
#endif

/* Message catalog (lab handout, Part 2). Standard 11-bit IDs, DLC 8. */
#define CAN_ID_CTRL  0x010
#define CAN_ID_FAULT 0x020
#define CAN_ID_HB_N1 0x100
#define CAN_ID_HB_N2 0x101
#define CAN_ID_HB_N3 0x102
#define CAN_ID_BLINK 0x200
#define CAN_ID_FORCE 0x300
#define CAN_BUS_DLC  8

/* Zone ids, as used in FAULT byte 1. NODE_ID is this node's zone. */
#define ZONE_N1 1
#define ZONE_N2 2
#define ZONE_N3 3

/*
 * EXERCISE 1.1: the heartbeat ID of a zone. Zone 1 -> 0x100, 2 -> 0x101,
 * 3 -> 0x102. Placeholder below is wrong on purpose.
 */
#define CAN_ID_HB(zone) (CAN_ID_HB_N1 + ((zone) - 1))

/*
 * EXERCISE 1.1: byte 0 of every payload is the 4-bit sequence counter in
 * the low nibble and 4 flag bits in the high nibble (bit 4 = relayed).
 * Write the mask and the two accessors. Placeholders are wrong on purpose.
 */
#define CAN_SEQ_MASK      0x0F
#define CAN_FLAG_RELAYED  0x10
#define CAN_SEQ(data)     ((data)[0] & CAN_SEQ_MASK)
#define CAN_RELAYED(data) (((data)[0] & CAN_FLAG_RELAYED) != 0)

/* FAULT reason codes (byte 2). */
#define FAULT_TEST_BUTTON 1
#define FAULT_BAD_CTRL    2
#define FAULT_HB_LOSS     3

/*
 * HB payload: byte 1 zone state, byte 2 bus health bits, byte 3 = 1 while
 * this zone has an active LOCAL fault (test button latched, bad-CTRL hold).
 * Byte 3 is what clears other zones' remote-fault latches: see zone_state.h.
 */
#define HB_STATE_OK    0
#define HB_STATE_ERROR 1
#define HB_HEALTH_A    BIT(0)
#define HB_HEALTH_B    BIT(1)

/* Timing thresholds, in one place (R4.5, R4.7). */
#define HB_PERIOD_MS  20
#define HB_MISS_LIMIT 3
#define HB_TIMEOUT_MS (HB_PERIOD_MS * HB_MISS_LIMIT) /* DEG on / zone failed */
#define DUP_WINDOW_MS 10 /* < 20 ms min interarrival of any one ID */
#define TX_STALL_MS   HB_TIMEOUT_MS /* nothing sent this long: restart the controller */
#define CAN_TX_WAIT_MS 2 /* max wait for the driver's single TX buffer */

enum can_bus_id {
	BUS_A,
	BUS_B,
	BUS_COUNT,
};

/*
 * Called for the first copy of each frame of a registered ID.
 *
 * Runs in the MCP2515 driver's interrupt thread (cooperative, prio -14),
 * with that controller's driver mutex held. Keep it short: copy what you
 * need into shared state or a k_msgq and wake your task. Do not call
 * can_bus_send() or can_bus_forward() from here (the other bus's driver
 * thread may hold the other mutex: deadlock), and do not print or sleep.
 * Taking a short-held k_mutex (like zone_state's) is fine: this is a thread.
 */
typedef void (*can_bus_handler_t)(const struct can_frame *frame, enum can_bus_id bus);

/* Register a handler for one catalog ID. Call before can_bus_init(). */
int can_bus_set_handler(uint16_t id, can_bus_handler_t handler);

/* Set up test points, add the catalog filters on both buses, start both. */
int can_bus_init(void);

/*
 * Send one catalog frame on A then B with one new sequence number.
 * data[0] high nibble carries flags; the low nibble is overwritten.
 * Returns 0 if at least one bus queued the frame, -EIO if neither did.
 * Thread context only.
 */
int can_bus_send(uint16_t id, const uint8_t data[CAN_BUS_DLC]);

/* RELAY (N2, stage 6): queue an already-received frame on one bus, seq unchanged. */
int can_bus_forward(enum can_bus_id to, const struct can_frame *frame);

/*
 * RELAY (N2): also call `tap` for every first copy of every received frame,
 * after the ID's handler. Same context rules as a handler.
 */
void can_bus_set_relay_tap(can_bus_handler_t tap);

/*
 * Milliseconds since a heartbeat from this zone last arrived on this bus,
 * counting every copy (before dedup). INT64_MAX if never seen.
 */
int64_t can_bus_hb_age_ms(uint8_t zone, enum can_bus_id bus);

/* Controller state as last reported by the driver. */
enum can_state can_bus_state(enum can_bus_id bus);

/* Frames each bus failed to queue or send since the last call. */
uint32_t can_bus_take_tx_fail(enum can_bus_id bus);

/* Times this bus's controller was restarted after a TX stall, since the last call. */
uint32_t can_bus_take_restarts(enum can_bus_id bus);

/* CRC-8, poly 0x07, init 0x00 (same as the Pi side). CTRL: over bytes 0..6. */
uint8_t can_bus_crc8(const uint8_t *buf, size_t len);

#endif /* CAN_BUS_H_ */
