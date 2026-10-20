#ifndef CONTROL_H_
#define CONTROL_H_

/*
 * CTRL (0x010) consumers: STEER on N2 (-> SRV), DRIVE + PID on N3
 * (-> DIR_A, PWM_SET, PWM_OUT). Shared CTRL decode and validation.
 *
 * CTRL payload (team-defined values table):
 *   b0 seq/flags | b1-2 steer int16 LE | b3 throttle % | b4 brake % |
 *   b5 buttons   | b6 0                | b7 CRC-8 over b0..b6, with the
 *                                         FLAG bits of b0 masked to 0
 *
 * The CRC skips b0's flag nibble because RELAY sets the relayed flag in
 * transit: a CRC over the raw b0 fails every relayed CTRL (found in checkoff
 * step 7). The flags are transport metadata, not part of the command, and
 * CAN's own frame CRC already protects every bit on the wire.
 */

#include <stdint.h>

#include <zephyr/drivers/can.h>

#define CTRL_STEER_MIN   (-32767)
#define CTRL_STEER_MAX   32766
#define CTRL_PCT_MAX     100
#define CTRL_BUTTON_MASK 0x07 /* bit0 self-test, bit1 right, bit2 left */
#define BRAKE_ON_PCT     50   /* brake pedal past half travel = brake */

struct ctrl {
	int16_t steer;
	uint8_t throttle;
	uint8_t brake;
	uint8_t buttons;
};

/* 0 and *out filled if the frame is a valid CTRL; -EBADMSG otherwise. */
int ctrl_decode(const struct can_frame *frame, struct ctrl *out);

/* Register the CTRL handler. Call before can_bus_init(). */
void control_init(void);

/* Configure outputs and start this node's CTRL tasks. After can_bus_init(). */
int control_start(void);

/* Valid / rejected CTRL frames since the last call, for the console. */
uint32_t control_take_ok(void);
uint32_t control_take_bad(void);

#endif /* CONTROL_H_ */
