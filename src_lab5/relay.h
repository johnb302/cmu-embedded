#ifndef APP_RELAY_H_
#define APP_RELAY_H_

/*
 * RELAY (Lab 3 Part 4, Lab 5 step 6): forward frames to the other bus for a node that
 * can't reach it. Lab 5: any node on both buses (N2, N3, or the Pi in cockpit.c), for
 * Raft frames too, by the routing table built from getNetwork() (shared/netwatch.c), plus
 * the same rule on direct heartbeats until the log catches up (Part 5.2). A frame from S,
 * heard on bus X, goes onto bus Y when S isn't on Y and some third node is on Y but not
 * on X. Never a relayed copy (no loops). A relayed REF gets the first hop and its time in
 * the relay added to its master time.
 */

#include <zephyr/drivers/can.h>

/* rx_cb (driver thread), every accepted frame: decide, and queue it if the other
 * bus needs it. Never blocks, never sends. rx_us: its receive time (local_us()). */
void relay_offer(const struct can_frame *f, int from_bus, int64_t rx_us);

/* The RAFT thread, every 10 ms: the routing table from the committed graph */
struct route_table;
void relay_set_routes(const struct route_table *t);

/* Configure RELAY_TX and start the RELAY thread. After can_io_init(). */
int relay_start(void);

/* Frames forwarded since the last call, for the console. */
uint32_t relay_take_count(void);

#endif /* APP_RELAY_H_ */
