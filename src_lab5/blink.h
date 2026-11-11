#ifndef APP_BLINK_H_
#define APP_BLINK_H_

/*
 * BLINK_F (N2) / BLINK_R (N3): follow the Pi's BLINK frames (0x200).
 * The Pi is the blinker master: it toggles the phase every 500 ms and every
 * zone just shows what it's told, so all four lamps stay in sync.
 *
 * Payload: b1 state (BLINK_OFF/LEFT/RIGHT/HAZARD) | b2 bit 0 = phase (1 = lamps on)
 */

#include <zephyr/drivers/can.h>

/* rx_cb (driver thread): queue this BLINK frame, keeping only the newest. Never blocks. */
void blink_post(const struct can_frame *f);

/* Configure the two blinker outputs (off) and start the BLINK thread. */
int blink_start(void);

#endif /* APP_BLINK_H_ */
