#ifndef BLINK_H_
#define BLINK_H_

/*
 * BLINK_F (N2) / BLINK_R (N3): drive this node's two blinkers from the
 * cockpit's BLINK frames (0x200).
 *
 * BLINK payload: b1 state (0 off, 1 left, 2 right, 3 hazard), b2 bit0 phase
 * (1 = lamps on). The Pi is the blinker master: it sends the toggle edge,
 * so N2 and N3 switch on the same frame (R2.4's 1 ms front/rear sync).
 */

#define BLINK_OFF    0
#define BLINK_LEFT   1
#define BLINK_RIGHT  2
#define BLINK_HAZARD 3

/* Register the BLINK handler. Call before can_bus_init(). */
void blink_init(void);

/* Configure the blinker pins and start the task. */
int blink_start(void);

#endif /* BLINK_H_ */
