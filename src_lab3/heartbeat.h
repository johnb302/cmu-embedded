#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

/*
 * HB_TX and HB_MON (Part 2 task table, R4.5, R4.7).
 *
 * HB_TX:  every 20 ms, send this zone's heartbeat on both buses.
 * HB_MON: every 20 ms, decide per bus whether it's degraded (DEG_A/DEG_B)
 *         and whether every other zone is still alive (zone_state).
 */

#include <stdbool.h>
#include <stdint.h>

#include "can_bus.h"

/* Register the peer-heartbeat handlers. Call before can_bus_init(). */
void heartbeat_init(void);

/* Configure DEG LEDs and start both threads. Call after can_bus_init(). */
int heartbeat_start(void);

/* HB_MON's current verdict for a bus, for HB_TX's health bits and the console. */
bool heartbeat_bus_degraded(enum can_bus_id bus);

/* Loop counters, so the watchdog can check both threads are still running. */
uint32_t heartbeat_progress(void);

#endif /* HEARTBEAT_H_ */
