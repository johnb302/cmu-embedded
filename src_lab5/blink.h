#ifndef APP_BLINK_H_
#define APP_BLINK_H_

/*
 * BLINK_F (N2) / BLINK_R (N3). The Pi's BLINK frames (0x200) say which lamps blink
 * (b1: BLINK_OFF/LEFT/RIGHT/HAZARD). Lab 5: when they light comes from global_gettime(),
 * so all four lamps switch together (FL and RL within 1 ms), and in failure mode (cockpit
 * lost) the STM32s blink hazards at 4 Hz by themselves, still in step.
 */

#include <zephyr/drivers/can.h>

/* rx_cb (driver thread): queue this BLINK frame, keeping only the newest. Never blocks. */
void blink_post(const struct can_frame *f);

/* Configure the two blinker outputs (off) and start the BLINK thread. */
int blink_start(void);

#endif /* APP_BLINK_H_ */
