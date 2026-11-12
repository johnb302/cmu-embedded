#include "raft.h"
#include "raft_config.h"
#include <stdio.h>

/* Names follow Figure 2 of the Raft paper (currentTerm, votedFor, ...) */

static uint8_t my_id;
static uint8_t current_term;
static uint8_t voted_for;
static enum raft_role role = RAFT_FOLLOWER;
static uint8_t leader_id = RAFT_NODE_NONE;
static int64_t last_heard_ms; /* last REF or APPEND from leader_id (boot time until then) */

/* SYNC: when this follower last got a REF, the time-sync frame */
static bool heard_ref;
static int64_t last_ref_ms;

/* Leader only */
static uint8_t ref_cycle; /* REF cycle counter: +1 per REF, so both bus copies carry the same count */
static int64_t ref_next_due;
static int64_t append_next_due;

/* Part 2: a follower declares the leader dead after missed_heartbeats x heartbeat_ms
 * (plus jitter, step 5). Also how long SYNC stays on without a REF. */
#define LEADER_TIMEOUT_MS (RAFT_MISSED_HEARTBEATS * RAFT_HEARTBEAT_MS)

/* RAFT_DEBUG: one line per change of role, term, or leader. Never per frame:
 * the leader sends 60 frames/s and the console would drown. */
static void log_state(const char *why) {
#if RAFT_DEBUG
    printf("raft N%u: %s -> %s, term %u, leader N%u\n", my_id, why, raft_role_name(role), current_term,
           leader_id);
#else
    (void)why;
#endif
}

void raft_init(uint8_t id, int64_t now_ms) {
    my_id = id;
    last_heard_ms = now_ms; /* followers give the leader a full timeout to show up */
    heard_ref = false;
    ref_cycle = 0;

    if (RAFT_DEFAULT_LEADER == RAFT_NODE_NONE) {
        /* LEADER=random: no preset leader, so nobody has voted yet. Step 5 elects one. */
        current_term = 0;
        voted_for = RAFT_NODE_NONE;
        leader_id = RAFT_NODE_NONE;
        role = RAFT_FOLLOWER;
    } else {
        /* Preset leader (Part 2): everyone boots in term 1 having, in effect, voted for it */
        current_term = 1;
        voted_for = RAFT_DEFAULT_LEADER;
        leader_id = RAFT_DEFAULT_LEADER;
        role = (my_id == leader_id) ? RAFT_LEADER : RAFT_FOLLOWER;
    }

    if (role == RAFT_LEADER) {
        /* First REF and APPEND on the first tick. From the clock, never from 0: the Pi's
         * clock is ms since it booted, and a due time of 0 would mean "overdue" for minutes. */
        ref_next_due = now_ms;
        append_next_due = now_ms;
    }
    log_state("boot");
}

/* ---- Leader frames (layouts in raft.h). Each builds one frame and sends it. ---- */

/* 16-bit fields go little-endian: low byte first */
static void put_u16(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)(v >> 8);
}

/* REF: b0 term | b1 cycle counter | b2-7 left 0: the platform stamps the master time */
static void send_ref(void) {
    uint8_t data[8] = {0};

    data[0] = current_term & RAFT_TERM_MASK;
    data[1] = ref_cycle++;
    raft_platform_send(RAFT_ID(RAFT_ID_REF, my_id), data);
}

/* Empty APPEND: the leader's log position and commit index, with no entry (b4, b5 = 0) */
static void send_empty_append(void) {
    /* TODO(step 2): from the log: last index, the term of that entry, the commit index.
     * All 0 while the log is empty. */
    uint16_t prev_index = 0;
    uint8_t prev_term = 0;
    uint16_t commit_index = 0;

    uint8_t data[8] = {0};
    data[0] = current_term & RAFT_TERM_MASK;
    put_u16(&data[1], prev_index);
    data[3] = prev_term;
    /* data[4] entry term and data[5] event stay 0: event 0 = no entry */
    put_u16(&data[6], commit_index);
    raft_platform_send(RAFT_ID(RAFT_ID_APPEND, my_id), data);
}

void raft_tick(int64_t now_ms) {
    if (role != RAFT_LEADER) {
        return; /* step 5: a follower's election timeout goes here */
    }

    /* Due times move on by exactly one period, so REFs stay on a 20 ms grid even when a
     * tick is late. REF first: at t = 0, 100, 200, ... both are due, and the sync frame
     * shouldn't queue behind the APPEND. */
    if (now_ms >= ref_next_due) {
        send_ref();
        ref_next_due += RAFT_HEARTBEAT_MS;
    }
    if (now_ms >= append_next_due) {
        send_empty_append();
        append_next_due += RAFT_APPEND_PERIOD_MS;
    }
}

void raft_on_frame(uint16_t id, const uint8_t data[8], int64_t now_ms) {
    uint16_t type = RAFT_ID_TYPE(id);
    uint8_t sender = RAFT_ID_SENDER(id);
    uint8_t term = data[0] & RAFT_TERM_MASK;

    if (sender < 1 || sender > RAFT_N || sender == my_id) {
        return; /* not a cluster node, or our own frame relayed back to us (step 6) */
    }
    if (term < current_term) {
        return; /* stale sender: an old leader or candidate. Step 5 replies so it steps down. */
    }

    uint8_t old_term = current_term;
    uint8_t old_leader = leader_id;
    enum raft_role old_role = role;

    /* Raft rule for every server: a higher term means we're behind. Adopt it and follow.
     * A leader that sees one steps down here. Who leads the new term isn't known until
     * its REF or APPEND arrives (maybe this very frame, just below). */
    if (term > current_term) {
        current_term = term;
        voted_for = RAFT_NODE_NONE;
        role = RAFT_FOLLOWER;
        leader_id = RAFT_NODE_NONE;
    }

    /* Only this term's leader sends REF and APPEND. A leader never hears another one in its
     * own term (Raft allows one leader per term), so it ignores them. A candidate that hears
     * one has lost this term's election (step 5) and follows. Duplicates (one copy per bus)
     * just set the same values again. */
    bool from_leader = type == RAFT_ID_REF || type == RAFT_ID_APPEND;
    if (from_leader && role != RAFT_LEADER) {
        role = RAFT_FOLLOWER;
        leader_id = sender;
        last_heard_ms = now_ms;
        if (type == RAFT_ID_REF) {
            heard_ref = true;
            last_ref_ms = now_ms;
        }
        /* TODO(step 4): APPEND: check prevLogIndex/prevLogTerm against the log, append the
         * entry, advance the commit index, and reply with APPEND_RESP. */
    }
    /* TODO(step 4): APPEND_RESP and EVENT at the leader. TODO(step 5): REQ_VOTE and VOTE. */

    if (current_term != old_term || leader_id != old_leader || role != old_role) {
        char why[24];
        snprintf(why, sizeof(why), "frame from N%u", sender);
        log_state(why);
    }
}

bool raft_in_sync(int64_t now_ms) {
    if (role == RAFT_LEADER) {
        return true; /* the leader is the time master */
    }
    /* A follower is in sync while REFs keep coming. Not from boot: until the first REF
     * arrives there is nothing to be in sync with. */
    return heard_ref && now_ms - last_ref_ms < LEADER_TIMEOUT_MS;
}

enum raft_role raft_get_role(void) {
    return role;
}

const char *raft_role_name(enum raft_role r) {
    static const char *const names[] = {
        [RAFT_FOLLOWER] = "follower",
        [RAFT_CANDIDATE] = "candidate",
        [RAFT_LEADER] = "leader",
    };
    return r <= RAFT_LEADER ? names[r] : "?";
}

uint8_t raft_get_term(void) {
    return current_term;
}

uint8_t raft_get_leader(void) {
    return leader_id;
}

int64_t raft_last_heard(void) {
    return last_heard_ms;
}
