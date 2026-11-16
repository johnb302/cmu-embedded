#ifndef APP_TIME_NODE_H_
#define APP_TIME_NODE_H_

/*
 * Global time on the STM32s (Part 6, steps 7-8): shared/timesync.c behind a spinlock,
 * the PPS output and its console line, and the `time` shell commands.
 */

#include <stdbool.h>
#include <stdint.h>

/* The local clock: µs since boot from the 84 MHz cycle counter (12 ns steps, unlike the
 * 100 µs kernel tick) */
int64_t local_us(void);

/* Part 6.3: the current global time in µs. The leader's own clock; a follower's local clock
 * mapped through the last REF and the drift ratio. */
int64_t global_gettime(void);

/* The local time at which global time reads global_us */
int64_t time_local_for_global(int64_t global_us);

/* Wait until global time reaches global_us: sleep, then spin the last ~200 µs */
void time_wait_global(int64_t global_us);

/* The RAFT thread: a fresh REF from the leader, received at rx_us (stamped in the CAN RX
 * callback); and whether this node is the time master */
void time_on_ref(uint8_t master, const uint8_t data[8], int64_t rx_us);
void time_set_master(bool master);

/* A relay forwarding a REF: rewrite its master time for the next hop */
void time_relay_ref(uint8_t master, uint8_t data[8], int64_t rx_us);

/* REF bytes 2-7: the master time, 48-bit LE */
void time_put_ref(uint8_t data[8], int64_t global_us);

/* Configure PPS and start its thread. After raft_node_start(). */
int time_node_start(void);

/* main(): wait for the next PPS edge (up to timeout_ms), then print its line. False on timeout. */
bool time_print_pps(int timeout_ms);

#endif /* APP_TIME_NODE_H_ */
