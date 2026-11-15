#ifndef SHARED_RAFT_H_
#define SHARED_RAFT_H_

/*
 * Raft agent (Part 1, Figure 1): consensus. The state machine is netstate.c.
 *
 * Portable C, built into all three nodes. The platform owns the threads, CAN, and
 * clock (src_lab5/raft_node.c on the STM32s, cockpit/cockpit.c on the Pi) and calls in:
 *   raft_init()      once at boot
 *   raft_on_frame()  for every Raft frame received, from either bus
 *   raft_tick()      at least every RAFT_TICK_MS
 * and implements raft_platform_send(). Nothing in shared/ is thread-safe: the platform
 * holds its one Raft lock around every call in.
 *
 * Steps 3-4 (fixed leader): the leader sends REF every heartbeat_ms and an empty APPEND
 * every 100 ms. Events reach the leader's log through raft_submit() (EVENT frames from
 * followers); the leader sends each entry in an APPEND and commits it once a majority
 * holds it. No elections yet (step 5).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---------------------------- frame catalog ----------------------------
 * Lab 5 additions to the Lab 3 catalog. One CAN ID per frame type per sender
 * (ID = base + sender node 1..3), so two nodes never send the same ID with different
 * data, which was the Lab 3 FAULT collision.
 * TODO: confirm with the TA. The handout names REF, APPEND, REQ_VOTE, MEMBER, and EVENT
 * but gives no IDs or layouts.
 */
#define RAFT_ID_REF         0x000 /* leader -> all, every heartbeat_ms. Above CTRL (0x010): sync accuracy */
#define RAFT_ID_APPEND      0x040 /* leader -> all: AppendEntries with one entry, or empty */
#define RAFT_ID_APPEND_RESP 0x048 /* follower -> leader (step 4) */
#define RAFT_ID_REQ_VOTE    0x050 /* candidate -> all (step 5) */
#define RAFT_ID_VOTE        0x058 /* voter -> candidate (step 5) */
#define RAFT_ID_EVENT       0x060 /* client -> leader: an event to log (steps 4, 6) */

#define RAFT_ID(base, sender) ((uint16_t)((base) + (sender)))
#define RAFT_ID_TYPE(id)      ((uint16_t)((id) & 0x7F8))
#define RAFT_ID_SENDER(id)    ((uint8_t)((id) & 0x007))

/* Two mask filters cover every Raft frame: 0x000-0x003 (REF) and 0x040-0x07F (the rest) */
#define RAFT_FILTER_REF_ID   0x000
#define RAFT_FILTER_REF_MASK 0x7FC
#define RAFT_FILTER_ID       0x040
#define RAFT_FILTER_MASK     0x7C0

static inline bool raft_is_frame(uint16_t id) {
    bool is_ref = (id & RAFT_FILTER_REF_MASK) == RAFT_FILTER_REF_ID && id != 0;
    bool is_other = (id & RAFT_FILTER_MASK) == RAFT_FILTER_ID;
    return is_ref || is_other;
}

/*
 * Byte 0 of every Raft frame: bits 0-6 = the sender's term, bit 7 = relayed (step 6).
 * It is NOT the Lab 3 seq byte, so Raft frames skip the (ID, seq) dedup. Each frame
 * arrives once per bus, so every handler must be fine seeing it twice.
 * 7-bit terms wrap at 127: far more elections than a demo has.
 */
#define RAFT_TERM_MASK    0x7F
#define RAFT_FLAG_RELAYED 0x80

/*
 * REF     b0 term | b1 cycle counter: +1 per REF, so both bus copies carry the same
 *         count | b2-7 master time in us, 48-bit little-endian. The platform's send
 *         hook stamps b2-7 just before transmit (step 7: global_gettime()).
 *
 * APPEND  b0 term | b1-2 prevLogIndex LE | b3 prevLogTerm | b4 entry term |
 *         b5 entry event (net_event_encode code), 0 = none (an empty APPEND) |
 *         b6-7 leaderCommit LE. An empty APPEND's prev is the leader's last entry.
 *
 * APPEND_RESP  b0 term | b1 1 = success, 0 = failure | b2-3 index LE. Success: the
 *         follower's log matches the leader's through index (its matchIndex). Failure: it
 *         can't match past index, so the leader resends from index + 1.
 *
 * EVENT   b0 term (not checked: a client request isn't tied to a term) | b1 sequence
 *         number, +1 per event, so the leader appends each event once |
 *         b2 event (net_event_encode code)
 */

/* ------------------------------- the API ------------------------------- */

enum raft_role { RAFT_FOLLOWER, RAFT_CANDIDATE, RAFT_LEADER };

#define RAFT_TICK_MS 5 /* platforms call raft_tick() at least this often */

void raft_init(uint8_t my_id, int64_t now_ms);
void raft_on_frame(uint16_t id, const uint8_t data[8], int64_t now_ms);
void raft_tick(int64_t now_ms);

enum raft_role raft_get_role(void);
const char *raft_role_name(enum raft_role role);
uint8_t raft_get_term(void);
uint8_t raft_get_leader(void);     /* RAFT_NODE_NONE if not known */
int64_t raft_last_heard(void);     /* follower: when the leader was last heard */
bool raft_in_sync(int64_t now_ms); /* the SYNC LED */

/* The Part 2 config as one line, for the consoles' `raft config`. The same on every node,
 * since every node builds from the one raft_config.h. */
void raft_config_format(char *buf, size_t len);

/*
 * A local client's event (a net_event_encode code). On the leader it becomes the next log
 * entry and the index is returned. On a follower it goes to the leader in an EVENT frame
 * (the Part 1 redirect) and 0 is returned; the entry arrives when the leader replicates
 * it. Fire and forget: if the leader dies first, the event is lost.
 * -EINVAL for a bad code, -EAGAIN while no leader is known, -ENOSPC if the log is full.
 */
int raft_submit(uint8_t event);

/*
 * Each platform implements this: send one 8-byte frame on both buses with byte 0
 * untouched (no Lab 3 seq stamp). For a REF, stamp the master time into b2-7 first.
 */
void raft_platform_send(uint16_t id, uint8_t data[8]);

#endif /* SHARED_RAFT_H_ */
