#ifndef APP_RELAY_H_
#define APP_RELAY_H_

/*
 * RELAY (N2 only, Part 4 / checkoff step 7).
 *
 * Rule: a frame from sender S, received on bus X, is forwarded onto the other
 * bus Y only if
 *   - S hasn't been heard on Y for HB_TIMEOUT_MS  (S can't reach Y itself), and
 *   - some other zone IS heard on Y                (someone there to receive it), and
 *   - it isn't already a relayed copy               (no loops).
 * With both buses healthy nothing is forwarded: no extra bus load. The cost is
 * that the first relayed frame waits for the 60 ms detection (R4.7).
 */

#include <zephyr/drivers/can.h>

/* rx_cb (driver thread), every accepted frame: decide, and queue it if the other
 * bus needs it. Never blocks, never sends. */
void relay_offer(const struct can_frame *f, int from_bus);

/* Configure RELAY_TX and start the RELAY thread. After can_io_init(). */
int relay_start(void);

/* Frames forwarded since the last call, for the console. */
uint32_t relay_take_count(void);

#endif /* APP_RELAY_H_ */
