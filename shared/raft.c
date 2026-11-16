#include "raft.h"
#include "netstate.h"
#include "raft_config.h"
#include "raft_log.h"
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Names follow Figure 2 of the Raft paper (currentTerm, votedFor, nextIndex, ...) */

static uint8_t my_id;
static uint8_t current_term;
static uint8_t voted_for;
static enum raft_role role = RAFT_FOLLOWER;
static uint8_t leader_id = RAFT_NODE_NONE;
static int64_t last_heard_ms; /* last REF or APPEND from leader_id (boot time until then) */

/* Elections (step 5) */
static bool elections_on;    /* off at boot with a preset leader, until a leader is heard */
static int64_t election_due; /* not leader: when to start the next pre-vote */
static uint8_t votes;        /* (pre-)candidate: bit n = node n said yes. Bus copies set the same bit. */
static uint8_t answered_term; /* the last RequestVote answer printed, so each prints once, */
static uint8_t answered_cand; /* not once per bus copy */

/* SYNC: when this follower last got a REF, the time-sync frame */
static bool heard_ref;
static int64_t last_ref_ms;

/* Leader only */
static uint8_t ref_cycle; /* REF cycle counter: +1 per REF, so both bus copies carry the same count */
static int64_t ref_next_due;
static int64_t append_next_due;
static uint16_t next_index[RAFT_N + 1];  /* by node ID: the next entry to send that follower */
static uint16_t match_index[RAFT_N + 1]; /* by node ID: the last entry known to be in its log */
static int64_t resend_due[RAFT_N + 1];   /* by node ID: when its next entry may go out */
static bool commit_news;                 /* commitIndex moved, and no APPEND has carried it yet */

/* Client side: +1 per EVENT sent, so the leader can tell a new event from a bus copy */
static uint8_t event_seq;

/* Part 2: a follower declares the leader dead after missed_heartbeats x heartbeat_ms
 * plus its jitter (raft_election_timeout_ms). Without the jitter: how long SYNC stays on
 * without a REF, and how long after hearing a leader a node turns down pre-votes. */
#define LEADER_TIMEOUT_MS (RAFT_MISSED_HEARTBEATS * RAFT_HEARTBEAT_MS)

/* An entry a follower hasn't acknowledged goes again after this. A follower that's down
 * costs one frame per RESEND_MS, like the empty APPEND. */
#define RESEND_MS RAFT_APPEND_PERIOD_MS

/* Every frame arrives once per bus, but an APPEND or EVENT must be acted on once. The
 * second copy has the same bytes within a few ms; the same bytes later are a new frame
 * (the next heartbeat, or a resend). A sender's back-to-back frames can arrive as
 * A1 A2 B1 B2, so each sender's last RX_SLOTS frames are kept, not just its last one. */
#define BUS_COPY_MS RAFT_HEARTBEAT_MS
#define RX_SLOTS    8

struct recent_rx {
    uint8_t data[RX_SLOTS][8];
    int64_t at_ms[RX_SLOTS];
    uint8_t used; /* slots filled so far */
    uint8_t next; /* the slot to overwrite next */
};
static struct recent_rx append_rx[RAFT_N]; /* by sender - 1 */
static struct recent_rx event_rx[RAFT_N];  /* by sender - 1 */

/* "N3", or "none" for RAFT_NODE_NONE */
static const char *node_str(uint8_t id, char buf[8]) {
    if (id == RAFT_NODE_NONE) {
        return "none";
    }
    snprintf(buf, 8, "N%u", id);
    return buf;
}

/* RAFT_DEBUG: one line per change of role, term, or leader. Never per frame:
 * the leader sends 60 frames/s and the console would drown. */
static void log_state(const char *why) {
#if RAFT_DEBUG
    char leader[8];
    printf("raft N%u: %s -> %s, term %u, leader %s\n", my_id, why, raft_role_name(role), current_term,
           node_str(leader_id, leader));
#else
    (void)why;
#endif
}

/* RAFT_DEBUG: one line per log change, the Part 2 "append msg" */
static void debug(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void debug(const char *fmt, ...) {
    if (RAFT_DEBUG) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
}

/* e.g. "raft N1: append 5 term 1 CANLINK_DOWN N2 A (from N2)" */
static void log_entry(const char *what, uint16_t index, uint8_t from) {
    if (!RAFT_DEBUG) {
        return;
    }
    struct net_event ev;
    char name[24] = "?";
    if (net_event_decode(raft_log_event(index), &ev) == 0) {
        net_event_format(&ev, name, sizeof(name));
    }
    char from_str[16] = "";
    if (from != RAFT_NODE_NONE) {
        snprintf(from_str, sizeof(from_str), " (from N%u)", from);
    }
    printf("raft N%u: %s %u term %u %s%s\n", my_id, what, index, raft_log_term(index), name, from_str);
}

/* 16-bit fields go little-endian: low byte first */
static void put_u16(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *src) {
    return (uint16_t)(src[0] | src[1] << 8);
}

static uint16_t min_u16(uint16_t a, uint16_t b) {
    return a < b ? a : b;
}

/* True if this frame is the other bus's copy of one of the sender's recent frames.
 * Otherwise remembers it and returns false. */
static bool is_bus_copy(struct recent_rx *rx, const uint8_t data[8], int64_t now_ms) {
    for (int i = 0; i < rx->used; i++) {
        if (now_ms - rx->at_ms[i] < BUS_COPY_MS && memcmp(rx->data[i], data, 8) == 0) {
            return true;
        }
    }
    memcpy(rx->data[rx->next], data, 8);
    rx->at_ms[rx->next] = now_ms;
    rx->next = (rx->next + 1) % RX_SLOTS;
    if (rx->used < RX_SLOTS) {
        rx->used++;
    }
    return false;
}

/* lowbias32 (Chris Wellons' hash prospector): every input bit flips about half the output bits */
static uint32_t mix32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

int raft_election_timeout_ms(uint32_t seed, uint8_t node, uint8_t term) {
    uint32_t h = mix32(mix32(mix32(seed) ^ node) ^ term);
    return LEADER_TIMEOUT_MS + (int)(h % (RAFT_ELECTION_JITTER_MS + 1));
}

/* Figure 2: followers restart the timeout on hearing the leader or granting a vote,
 * candidates on starting an election */
static void reset_election_timer(int64_t now_ms) {
    election_due = now_ms + raft_election_timeout_ms(RAFT_SEED, my_id, current_term);
}

/* Is a leader still heard here? Then this node turns down pre-votes. A leader hears itself. */
static bool leader_alive(int64_t now_ms) {
    return role == RAFT_LEADER || (leader_id != RAFT_NODE_NONE && now_ms - last_heard_ms < LEADER_TIMEOUT_MS);
}

static int count_votes(void) {
    int n = 0;
    for (uint8_t node = 1; node <= RAFT_N; node++) {
        n += (votes >> node) & 1;
    }
    return n;
}

/* The preset leader at boot, or an election won. REF and APPEND start on the next tick.
 * Figure 2: nextIndex = last log index + 1 and matchIndex = 0 for every follower; their
 * replies show where each one really is. */
static void become_leader(int64_t now_ms) {
    role = RAFT_LEADER;
    leader_id = my_id;
    elections_on = true; /* if it ever steps down, it times out like any follower */

    /* From the clock, never from 0: the Pi's clock is ms since it booted, and a due time
     * of 0 would mean "overdue" for minutes. */
    ref_next_due = now_ms;
    append_next_due = now_ms;
    for (uint8_t node = 1; node <= RAFT_N; node++) {
        next_index[node] = raft_log_last_index() + 1;
        match_index[node] = 0;
        resend_due[node] = now_ms;
    }
    commit_news = false;
}

void raft_init(uint8_t id, int64_t now_ms) {
    my_id = id;
    last_heard_ms = now_ms; /* followers give the leader a full timeout to show up */
    heard_ref = false;
    ref_cycle = 0;
    event_seq = 0;
    votes = 0;
    answered_term = 0;
    answered_cand = RAFT_NODE_NONE;
    memset(append_rx, 0, sizeof(append_rx));
    memset(event_rx, 0, sizeof(event_rx));
    role = RAFT_FOLLOWER;

    if (RAFT_DEFAULT_LEADER == RAFT_NODE_NONE) {
        /* LEADER=random: no preset leader and nobody has voted. Every timer runs from boot,
         * and the first node to time out (the seed decides) asks for votes. */
        current_term = 0;
        voted_for = RAFT_NODE_NONE;
        leader_id = RAFT_NODE_NONE;
        elections_on = true;
    } else {
        /* Preset leader (Part 2): everyone boots in term 1 having, in effect, voted for it,
         * and "no election is held at boot": followers wait for the leader's first frame
         * however long it takes to start (the Pi takes ~30 s to boot, the STM32s < 1 s).
         * From then on, its silence starts an election. */
        current_term = 1;
        voted_for = RAFT_DEFAULT_LEADER;
        leader_id = RAFT_DEFAULT_LEADER;
        elections_on = false;
    }
    reset_election_timer(now_ms);

    if (my_id == leader_id) {
        become_leader(now_ms);
    }
    log_state("boot");
}

/* ---- Leader frames (layouts in raft.h). Each builds one frame and sends it. ---- */

/* REF: b0 term | b1 cycle counter | b2-7 left 0: the platform stamps the master time */
static void send_ref(void) {
    uint8_t data[8] = {0};

    data[0] = current_term & RAFT_TERM_MASK;
    data[1] = ref_cycle++;
    raft_platform_send(RAFT_ID(RAFT_ID_REF, my_id), data);
}

/* APPEND with the entry after prev_index, or with none when prev_index is the last entry
 * (an empty APPEND). Every APPEND carries leaderCommit, so it also tells followers what
 * they can apply. */
static void send_append(uint16_t prev_index) {
    uint16_t index = prev_index + 1;
    uint8_t data[8] = {0};

    data[0] = current_term & RAFT_TERM_MASK;
    put_u16(&data[1], prev_index);
    data[3] = raft_log_term(prev_index);
    data[4] = raft_log_term(index); /* past the end: term 0 and NET_EVENT_NONE, no entry */
    data[5] = raft_log_event(index);
    put_u16(&data[6], raft_log_commit_index());
    raft_platform_send(RAFT_ID(RAFT_ID_APPEND, my_id), data);
    commit_news = false;
}

/* Send each follower the entry it needs next, one at a time: the next goes when it
 * acknowledges this one, or the same one again after RESEND_MS. Every node hears the
 * frame, so followers waiting for the same entry share it. */
static void replicate(int64_t now_ms) {
    uint16_t last = raft_log_last_index();

    for (uint8_t node = 1; node <= RAFT_N; node++) {
        if (node == my_id || next_index[node] > last || now_ms < resend_due[node]) {
            continue;
        }
        uint16_t index = next_index[node];
        send_append(index - 1);
        for (uint8_t other = 1; other <= RAFT_N; other++) {
            if (other != my_id && next_index[other] == index) {
                resend_due[other] = now_ms + RESEND_MS;
            }
        }
    }
}

/* Commit the newest entry from this term that a majority holds (Figure 2, last leader
 * rule); the entries before it commit with it. Only this term's entries are counted
 * (Raft 5.4.2): an older term's entry on a majority can still be overwritten. */
static void advance_commit(void) {
    for (uint16_t n = raft_log_last_index(); n > raft_log_commit_index(); n--) {
        if (raft_log_term(n) != current_term) {
            return; /* terms only go down from here */
        }
        int holders = 1; /* the leader */
        for (uint8_t node = 1; node <= RAFT_N; node++) {
            if (node != my_id && match_index[node] >= n) {
                holders++;
            }
        }
        if (holders >= RAFT_MAJORITY) {
            raft_log_commit(n);
            commit_news = true;
            debug("raft N%u: commit %u\n", my_id, n);
            return;
        }
    }
}

/* A client's event becomes the next entry; replicate() sends it on the next tick */
static int leader_append(uint8_t event, uint8_t from) {
    int index = raft_log_append(current_term, event);
    if (index < 0) {
        debug("raft N%u: can't append event 0x%02X from N%u (%d)\n", my_id, event, from, index);
        return index;
    }
    log_entry("append", (uint16_t)index, from);
    return index;
}

/* ---- Elections (step 5) ---- */

/* REQ_VOTE for term, describing our log by its last entry */
static void send_vote_request(uint8_t term, bool prevote) {
    uint8_t data[8] = {0};

    data[0] = term & RAFT_TERM_MASK;
    data[1] = prevote;
    put_u16(&data[2], raft_log_last_index());
    data[4] = raft_log_last_term();
    raft_platform_send(RAFT_ID(RAFT_ID_REQ_VOTE, my_id), data);
}

static void send_vote(uint8_t term, bool granted, uint8_t candidate, bool prevote) {
    uint8_t data[8] = {0};

    data[0] = term & RAFT_TERM_MASK;
    data[1] = granted;
    data[2] = candidate;
    data[3] = prevote;
    raft_platform_send(RAFT_ID(RAFT_ID_VOTE, my_id), data);
}

/* Raft 5.4.1: a vote only goes to a candidate whose log is at least as up-to-date as ours
 * (a later last term, or the same last term and at least as long), so whoever wins holds
 * every committed entry. data is a REQ_VOTE. */
static bool log_up_to_date(const uint8_t data[8]) {
    uint16_t last_index = get_u16(&data[2]);
    uint8_t last_term = data[4];
    uint8_t my_last_term = raft_log_last_term();

    return last_term > my_last_term || (last_term == my_last_term && last_index >= raft_log_last_index());
}

/*
 * Frames go out before the debug line about them, here and in every handler: a line takes
 * ~5 ms on the STM32 console (115200 baud, and printf blocks), and someone is waiting on
 * the frame. Printing first made the board's first election take 90 ms instead of ~70.
 */

/* Votes from a majority: this term is ours. REF and the empty APPEND go out at once
 * rather than on the next tick, so everyone learns who leads as soon as possible. */
static void win(int64_t now_ms) {
    become_leader(now_ms);
    raft_tick(now_ms);
    log_state("won the vote");
}

/* A majority would vote for us: take the next term, vote for ourselves, and ask for real
 * votes. The others hear the higher term and follow it, and vote if our log is up to date. */
static void become_candidate(int64_t now_ms) {
    current_term = (current_term + 1) & RAFT_TERM_MASK;
    role = RAFT_CANDIDATE;
    voted_for = my_id;
    leader_id = RAFT_NODE_NONE;
    votes = (uint8_t)(1U << my_id);
    reset_election_timer(now_ms); /* no majority by then (a split vote): start over */
    send_vote_request(current_term, false);
    log_state("pre-vote won");
    if (count_votes() >= RAFT_MAJORITY) {
        win(now_ms); /* only when N = 1 */
    }
}

/* The election timeout: no leader heard for a whole timeout. Ask the others whether
 * they'd vote for us in the next term, without taking it yet (pre-vote). A node that
 * still hears a leader says no, so a node that is merely cut off can't raise the term. */
static void start_prevote(int64_t now_ms) {
    bool first_try = role != RAFT_PRECANDIDATE;

    role = RAFT_PRECANDIDATE;
    votes = (uint8_t)(1U << my_id);
    reset_election_timer(now_ms); /* no majority by then: ask again */
    send_vote_request((uint8_t)(current_term + 1), true);
    if (first_try) {
        log_state("election timeout"); /* not every retry: a cut-off node retries ~12 times/s */
    }
    if (count_votes() >= RAFT_MAJORITY) {
        become_candidate(now_ms); /* only when N = 1 */
    }
}

/* Pre-vote asked of us: would we vote for this node in the term it names? Yes only if that
 * term is ahead of ours, we no longer hear a leader either, and its log is up to date.
 * It changes nothing here: no term, no votedFor, no timer. A yes carries the term asked
 * about; a no carries ours, which teaches a node that's behind the current term. */
static void on_prevote_request(uint8_t candidate, const uint8_t data[8], int64_t now_ms) {
    uint8_t term = data[0] & RAFT_TERM_MASK;
    bool granted = term > current_term && !leader_alive(now_ms) && log_up_to_date(data);

    send_vote(granted ? term : current_term, granted, candidate, true);
}

/* A pre-vote yes. Its term is the one we asked about, one past ours, so it isn't a newer
 * term to adopt. */
static void on_prevote_granted(uint8_t voter, const uint8_t data[8], int64_t now_ms) {
    uint8_t term = data[0] & RAFT_TERM_MASK;

    if (role != RAFT_PRECANDIDATE || data[2] != my_id || term != ((current_term + 1) & RAFT_TERM_MASK)) {
        return; /* for someone else, or an old round */
    }
    votes |= (uint8_t)(1U << voter);
    if (count_votes() >= RAFT_MAJORITY) {
        become_candidate(now_ms);
    }
}

/* Figure 2's RequestVote receiver. The frame's term is current by now. Each request comes
 * once per bus, and the second copy gets the same answer. */
static void on_request_vote(uint8_t candidate, const uint8_t data[8], int64_t now_ms) {
    bool granted = (voted_for == RAFT_NODE_NONE || voted_for == candidate) && log_up_to_date(data);

    if (granted) {
        voted_for = candidate;
        role = RAFT_FOLLOWER; /* a pre-candidate backs this candidate instead */
        reset_election_timer(now_ms);
    }
    send_vote(current_term, granted, candidate, false);

    if (answered_term != current_term || answered_cand != candidate) {
        answered_term = current_term;
        answered_cand = candidate;
        if (granted) {
            debug("raft N%u: vote for N%u in term %u\n", my_id, candidate, current_term);
        } else if (voted_for != RAFT_NODE_NONE && voted_for != candidate) {
            debug("raft N%u: no vote for N%u in term %u: voted for N%u\n", my_id, candidate, current_term,
                  voted_for);
        } else {
            debug("raft N%u: no vote for N%u in term %u: its log is behind ours\n", my_id, candidate,
                  current_term);
        }
    }
}

/* A candidate counts the yes votes for itself in its own term */
static void on_vote(uint8_t voter, const uint8_t data[8], int64_t now_ms) {
    if (data[3] || !data[1] || data[2] != my_id) {
        return; /* a pre-vote no, a no, or someone else's vote */
    }
    votes |= (uint8_t)(1U << voter);
    if (count_votes() >= RAFT_MAJORITY) {
        win(now_ms);
    }
}

void raft_tick(int64_t now_ms) {
    if (role != RAFT_LEADER) {
        if (elections_on && now_ms >= election_due) {
            start_prevote(now_ms);
        }
        return;
    }

    /* Due times move on by exactly one period, so REFs stay on a 20 ms grid even when a
     * tick is late. REF first: at t = 0, 100, 200, ... both are due, and the sync frame
     * shouldn't queue behind the APPEND. */
    if (now_ms >= ref_next_due) {
        send_ref();
        ref_next_due += RAFT_HEARTBEAT_MS;
    }
    replicate(now_ms);

    /* The empty APPEND: prev is the leader's last entry. A follower holding that index with
     * the same term matches the leader's whole log (Raft's Log Matching Property), so it
     * can apply up to leaderCommit. Every 100 ms, and also when the commit index has moved
     * and no APPEND has carried it yet, so followers apply a new entry a tick after the
     * leader does instead of up to 100 ms later. */
    if (now_ms >= append_next_due) {
        send_append(raft_log_last_index());
        append_next_due += RAFT_APPEND_PERIOD_MS;
    } else if (commit_news) {
        send_append(raft_log_last_index());
    }
}

/* ---- Frames in ---- */

static void send_append_resp(bool success, uint16_t index) {
    uint8_t data[8] = {0};

    data[0] = current_term & RAFT_TERM_MASK;
    data[1] = success;
    put_u16(&data[2], index);
    raft_platform_send(RAFT_ID(RAFT_ID_APPEND_RESP, my_id), data);
}

/* Follower: Figure 2's AppendEntries receiver. The frame's term is current by now.
 * One reply per APPEND, not per bus copy. */
static void on_append(uint8_t leader, const uint8_t data[8], int64_t now_ms) {
    if (is_bus_copy(&append_rx[leader - 1], data, now_ms)) {
        return;
    }
    uint16_t prev_index = get_u16(&data[1]);
    uint8_t prev_term = data[3];
    uint8_t entry_term = data[4];
    uint8_t entry_event = data[5];
    uint16_t leader_commit = get_u16(&data[6]);
    uint16_t last = raft_log_last_index();

    /* Our log must hold the leader's entry at prevLogIndex, or nothing after it can match.
     * The reply says how far we could match; the leader resends from there. */
    if (prev_index > last || raft_log_term(prev_index) != prev_term) {
        uint16_t can_match = prev_index == 0 ? 0 : min_u16(prev_index - 1, last);
        debug("raft N%u: no entry %u term %u here (last %u)\n", my_id, prev_index, prev_term, last);
        send_append_resp(false, can_match);
        return;
    }

    uint16_t match = prev_index; /* our log matches the leader's through here */
    if (entry_event != NET_EVENT_NONE) {
        uint16_t index = prev_index + 1;
        if (raft_log_term(index) != entry_term) { /* not ours yet (term 0 = no entry there) */
            if (index <= last) {
                /* A different entry here, from an old leader. Raft never commits one that
                 * conflicts, so it goes, with everything after it. */
                if (raft_log_truncate(index) < 0) {
                    debug("raft N%u: entry %u conflicts with a committed one\n", my_id, index);
                    send_append_resp(false, prev_index);
                    return;
                }
                debug("raft N%u: drop entries %u to %u: they conflict with the leader's\n", my_id, index, last);
            }
            if (raft_log_append(entry_term, entry_event) < 0) {
                send_append_resp(false, prev_index); /* a bad entry, or the log is full */
                return;
            }
            log_entry("append", index, RAFT_NODE_NONE);
        }
        match = index;
    }

    /* Apply what the leader has committed, but only as far as our log is known to match
     * its own: entries past `match` may be an old leader's (Figure 2: min(leaderCommit,
     * index of last new entry)). */
    uint16_t commit = min_u16(leader_commit, match);
    if (commit > raft_log_commit_index()) {
        raft_log_commit(commit);
        debug("raft N%u: commit %u\n", my_id, commit);
    }
    send_append_resp(true, match);
}

/* Leader: a follower's reply. Each comes twice (one per bus) and maybe late, so a reply
 * only moves the indexes forward (success) or back to what the follower can match
 * (failure), and a repeat changes nothing. */
static void on_append_resp(uint8_t follower, const uint8_t data[8], int64_t now_ms) {
    bool success = data[1] != 0;
    uint16_t index = get_u16(&data[2]);

    if (index > raft_log_last_index()) {
        return; /* not a reply to anything we sent */
    }
    if (success) {
        if (index + 1 > next_index[follower]) {
            next_index[follower] = index + 1;
            resend_due[follower] = now_ms; /* its next entry, if any, on the next tick */
        }
        if (index > match_index[follower]) {
            match_index[follower] = index;
            advance_commit();
        }
    } else if (index + 1 < next_index[follower]) {
        next_index[follower] = index + 1;
        resend_due[follower] = now_ms;
        if (match_index[follower] > index) {
            match_index[follower] = index; /* it lost entries: it rebooted (logs live in RAM) */
        }
    }
}

/* An event from another node's client (raft_submit there). Every node hears the frame
 * and only the leader acts on it: that's the whole redirect. */
static void on_event(uint8_t sender, const uint8_t data[8], int64_t now_ms) {
    if (role == RAFT_LEADER && !is_bus_copy(&event_rx[sender - 1], data, now_ms)) {
        leader_append(data[2], sender);
    }
}

void raft_on_frame(uint16_t id, const uint8_t data[8], int64_t now_ms) {
    uint16_t type = RAFT_ID_TYPE(id);
    uint8_t sender = RAFT_ID_SENDER(id);
    uint8_t term = data[0] & RAFT_TERM_MASK;

    if (sender < 1 || sender > RAFT_N || sender == my_id) {
        return; /* not a cluster node, or our own frame relayed back to us (step 6) */
    }
    if (type == RAFT_ID_EVENT) {
        on_event(sender, data, now_ms); /* a client request, not an RPC: no term rules */
        return;
    }

    /* Pre-votes skip the term rules: the term in a request, and in a yes, is one the
     * candidate doesn't hold yet, so nobody adopts it. A pre-vote no carries the voter's
     * real term and goes through the rules below like any answer. */
    if (type == RAFT_ID_REQ_VOTE && data[1]) {
        on_prevote_request(sender, data, now_ms);
        return;
    }
    if (type == RAFT_ID_VOTE && data[3] && data[1]) {
        on_prevote_granted(sender, data, now_ms);
        return;
    }

    if (term < current_term) {
        /* A stale leader or candidate: Figure 2 answers no with our term, and the newer term
         * makes it step down. An old leader's REF needs no answer: its APPENDs get one. The
         * no to an APPEND says we match through our last entry at most, which is true, so
         * this term's leader, which hears it too, loses nothing. */
        if (type == RAFT_ID_APPEND) {
            send_append_resp(false, raft_log_last_index());
        } else if (type == RAFT_ID_REQ_VOTE) {
            send_vote(current_term, false, sender, false);
        }
        return;
    }

    uint8_t old_term = current_term;
    uint8_t old_leader = leader_id;
    enum raft_role old_role = role;

    /* Raft rule for every server: a higher term means we're behind. Adopt it and follow.
     * A leader or candidate that sees one steps down here. Who leads the new term isn't
     * known until its REF or APPEND arrives (maybe this very frame, just below). Seeing a
     * term past 1 also means a cluster is running, so elections are on. */
    if (term > current_term) {
        current_term = term;
        voted_for = RAFT_NODE_NONE;
        role = RAFT_FOLLOWER;
        leader_id = RAFT_NODE_NONE;
        elections_on = true;
        reset_election_timer(now_ms);
    }

    /* Only this term's leader sends REF and APPEND. A leader never hears another one in its
     * own term (Raft elects one leader per term), so it ignores them. A candidate that hears
     * one has lost this term's election and follows. Duplicates (one copy per bus) just set
     * the same values again. */
    bool from_leader = type == RAFT_ID_REF || type == RAFT_ID_APPEND;
    if (from_leader && role != RAFT_LEADER) {
        if (leader_id != RAFT_NODE_NONE && sender != leader_id) {
            /* Two leaders in one term: only possible if the nodes were built with different
             * default_leader settings. Stay with the first. */
            debug("raft N%u: N%u and N%u both lead term %u: rebuild all nodes with one config\n", my_id,
                  leader_id, sender, current_term);
            return;
        }
        role = RAFT_FOLLOWER;
        leader_id = sender;
        last_heard_ms = now_ms;
        elections_on = true;
        reset_election_timer(now_ms);
        if (type == RAFT_ID_REF) {
            heard_ref = true;
            last_ref_ms = now_ms;
        }
    }

    bool changed = current_term != old_term || leader_id != old_leader || role != old_role;

    /* Answer first, then print what this frame changed (see win()) */
    if (type == RAFT_ID_APPEND && role == RAFT_FOLLOWER) {
        on_append(sender, data, now_ms);
    } else if (type == RAFT_ID_APPEND_RESP && role == RAFT_LEADER) {
        on_append_resp(sender, data, now_ms);
    } else if (type == RAFT_ID_REQ_VOTE) {
        on_request_vote(sender, data, now_ms);
    } else if (type == RAFT_ID_VOTE && role == RAFT_CANDIDATE) {
        on_vote(sender, data, now_ms);
    }

    if (changed) {
        char why[24];
        snprintf(why, sizeof(why), "frame from N%u", sender);
        log_state(why);
    }
}

int raft_submit(uint8_t event) {
    struct net_event ev;

    if (net_event_decode(event, &ev) < 0) {
        return -EINVAL;
    }
    if (role == RAFT_LEADER) {
        return leader_append(event, my_id);
    }
    if (leader_id == RAFT_NODE_NONE) {
        return -EAGAIN; /* mid-election (step 5): no one to send it to yet */
    }

    /* Part 1 redirect: a follower hands its client's event to the leader */
    uint8_t data[8] = {0};
    data[0] = current_term & RAFT_TERM_MASK;
    data[1] = event_seq++;
    data[2] = event;
    raft_platform_send(RAFT_ID(RAFT_ID_EVENT, my_id), data);

    char name[24];
    net_event_format(&ev, name, sizeof(name));
    debug("raft N%u: send %s to leader N%u\n", my_id, name, leader_id);
    return 0;
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
        [RAFT_PRECANDIDATE] = "pre-candidate",
        [RAFT_CANDIDATE] = "candidate",
        [RAFT_LEADER] = "leader",
    };
    return r <= RAFT_LEADER ? names[r] : "?";
}

static const char *node_name(uint8_t id) {
    switch (id) {
    case RAFT_NODE_COCKPIT:
        return "cockpit";
    case RAFT_NODE_STEERING:
        return "steering";
    case RAFT_NODE_DRIVETRAIN:
        return "drivetrain";
    default:
        return "none";
    }
}

/* In the handout's key names (Part 2), e.g. "N=3 default_leader=cockpit heartbeat_ms=20 ..." */
void raft_config_format(char *buf, size_t len) {
    snprintf(buf, len,
             "N=%d default_leader=%s heartbeat_ms=%d missed_heartbeats=%d election_timeout_jitter_ms=%d "
             "log_size=%d nodes=cockpit:%d,steering:%d,drivetrain:%d debug=%s seed=%u",
             RAFT_N, node_name(RAFT_DEFAULT_LEADER), RAFT_HEARTBEAT_MS, RAFT_MISSED_HEARTBEATS,
             RAFT_ELECTION_JITTER_MS, RAFT_LOG_SIZE, RAFT_NODE_COCKPIT, RAFT_NODE_STEERING,
             RAFT_NODE_DRIVETRAIN, RAFT_DEBUG ? "yes" : "no", (unsigned)RAFT_SEED);
}

void raft_status_format(char *buf, size_t len, int64_t now_ms) {
    char leader[8];
    int n = snprintf(buf, len, "%s | term %u | leader %s | SYNC %s", raft_role_name(role), current_term,
                     node_str(leader_id, leader), raft_in_sync(now_ms) ? "on" : "off");

    if (role == RAFT_LEADER || n < 0 || (size_t)n >= len) {
        return;
    }
    if (leader_id != RAFT_NODE_NONE && elections_on) { /* not the preset leader before it's heard */
        int m = snprintf(buf + n, len - (size_t)n, " | heard %lld ms ago", (long long)(now_ms - last_heard_ms));
        if (m < 0 || (size_t)(n + m) >= len) {
            return;
        }
        n += m;
    }
    if (elections_on) {
        snprintf(buf + n, len - (size_t)n, " | election timeout %d ms",
                 raft_election_timeout_ms(RAFT_SEED, my_id, current_term));
    } else {
        snprintf(buf + n, len - (size_t)n, " | no election until the leader is heard");
    }
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
