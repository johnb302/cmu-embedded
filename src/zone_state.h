#ifndef ZONE_STATE_H_
#define ZONE_STATE_H_

/*
 * This zone's error state machine (Lab 1 R3, Lab 3 Part 5).
 *
 *   power-up --> ERROR
 *   ERROR --> OK   when ALL of:
 *                  - every other zone heard on >= 1 bus within HB_TIMEOUT_MS
 *                  - a valid CTRL arrived since we entered ERROR
 *                  - no test-button latch (cleared by a double press)
 *                  - a bad-CTRL error has been held for ERROR_HOLD_MS
 *                  - no remote fault latched (see below)
 *   OK --> ERROR  on: test button, out-of-range CTRL, a zone missed
 *                  HB_MISS_LIMIT heartbeats on both buses, or a FAULT
 *                  frame from another zone.
 *
 * Remote faults: a FAULT from zone Z latches until Z's heartbeat reports
 * no active LOCAL fault (HB byte 3 == 0), at least REMOTE_CLEAR_MS later.
 * That's how "single press on N2 puts every zone in error, double press
 * brings them all back" works without a "fault cleared" message.
 * Why byte 3 and not Z's overall state: when a bad CTRL makes N2 AND N3
 * fault, each latches the other. If each waited for the other to be fully
 * OK, neither could ever leave (deadlock, found in checkoff step 10). A zone
 * that is only in ERROR because of someone else's FAULT reports byte 3 = 0,
 * so nobody waits on a follower.
 *
 * ERR (PB3) is high while in ERROR. All functions take a k_mutex, so call
 * them from threads (the CAN handlers count: they run in the driver's
 * thread), never from an ISR.
 */

#include <stdbool.h>
#include <stdint.h>

#include "can_bus.h"

enum zone_state {
	ZONE_OK = HB_STATE_OK,
	ZONE_ERROR = HB_STATE_ERROR,
};

/* Extra reason code for the console: we're in error because another zone said so. */
#define FAULT_REMOTE 4

#define ERROR_HOLD_MS   500 /* bad CTRL: stay in ERROR at least this long (visible) */
#define REMOTE_CLEAR_MS (3 * HB_PERIOD_MS)
#define FAULT_MIN_INTERVAL_MS 20 /* catalog: FAULT min interval */

/* Configure ERR, start in ZONE_ERROR, start the FAULT_TX thread. Call first. */
int zone_state_init(void);

enum zone_state zone_state_get(void);

/* Enter ERROR locally for `reason` (FAULT_*). Does not send FAULT. */
void zone_state_fail(uint8_t reason);

/* Enter ERROR for `reason` AND broadcast FAULT (test button, bad CTRL). */
void zone_state_report_fault(uint8_t reason);

/* Double press: drop the test-button latch (the zone still has to meet the exit rule). */
void zone_state_clear_test(void);
bool zone_state_test_latched(void);

/* A CTRL passed validation. */
void zone_state_note_valid_ctrl(void);

/* FAULT frame from another zone (FAULT handler). */
void zone_state_remote_fault(uint8_t zone, uint8_t reason);

/* Another zone's heartbeat arrived: its byte 3 local-fault flag (HB handler). */
void zone_state_peer_report(uint8_t zone, bool local_fault);

/* Our own local-fault flag, for HB byte 3. */
bool zone_state_local_fault(void);

/* HB_MON, every period: are all other zones alive? Evaluates the transitions. */
void zone_state_update_peers(bool all_peers_alive);

/* The last reason we entered ERROR, for the console. */
uint8_t zone_state_last_reason(void);

#endif /* ZONE_STATE_H_ */
