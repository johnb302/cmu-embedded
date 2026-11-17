/* Runs the shared/ code on the Mac (or the Pi), no board needed: make -C shared/test */
#include "netstate.h"
#include "raft.h"
#include "raft_config.h"
#include "raft_log.h"
#include "netwatch.h"
#include "timesync.h"
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

static int64_t now;      /* simulated ms */
static bool print_tx;    /* print each frame sent, or only count them */
static int refs, appends, resps, events, vote_reqs, votes;
static uint8_t last_append[8]; /* data of the last APPEND sent */
static uint8_t last_resp[8];   /* ... APPEND_RESP */
static uint8_t last_event[8];  /* ... EVENT */
static uint8_t last_req[8];    /* ... REQ_VOTE */
static uint8_t last_vote[8];   /* ... VOTE */
static int failures;

/* Stand-in for the CAN send hook: count each frame, and print it if asked */
void raft_platform_send(uint16_t id, uint8_t data[8]) {
    if (RAFT_ID_TYPE(id) == RAFT_ID_REF) {
        refs++;
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_APPEND) {
        appends++;
        memcpy(last_append, data, sizeof(last_append));
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_APPEND_RESP) {
        resps++;
        memcpy(last_resp, data, sizeof(last_resp));
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_EVENT) {
        events++;
        memcpy(last_event, data, sizeof(last_event));
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_REQ_VOTE) {
        vote_reqs++;
        memcpy(last_req, data, sizeof(last_req));
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_VOTE) {
        votes++;
        memcpy(last_vote, data, sizeof(last_vote));
    }
    if (print_tx) {
        printf("  t=%3lld ms  tx 0x%03X ", (long long)now, id);
        for (int i = 0; i < 8; i++) {
            printf(" %02X", data[i]);
        }
        printf("\n");
    }
}

static void check(const char *what, bool ok) {
    printf("  %-58s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) {
        failures++;
    }
}

/* ---- step 2: event codes and the log ---- */

static uint8_t code(enum net_event_type type, uint8_t node, uint8_t bus) {
    struct net_event ev = {type, node, bus};
    return net_event_encode(&ev);
}

static bool same_event(const struct net_event *a, const struct net_event *b) {
    return a->type == b->type && a->node == b->node && a->bus == b->bus;
}

/* The graph as `net show` prints it */
static const char *graph(void) {
    static char buf[96];
    struct network net = getNetwork();
    netstate_format(&net, buf, sizeof(buf));
    return buf;
}

#define ALL_DOWN "N1 down | N2 down | N3 down | cockpit ERROR"

/* 2.1: the 1-byte form of an event, stored in the log and sent in APPEND b5 */
static void event_codes(void) {
    printf("\nnetstate 2.1: 1-byte event codes\n");

    /* Every valid event: 2 cockpit, 3 nodes x up/down, 3 nodes x 2 buses x up/down */
    struct net_event all[20];
    int n = 0;
    all[n++] = (struct net_event){COCKPIT_OK, 0, 0};
    all[n++] = (struct net_event){COCKPIT_ERROR, 0, 0};
    for (uint8_t node = 1; node <= NET_NODES; node++) {
        all[n++] = (struct net_event){NODE_UP, node, 0};
        all[n++] = (struct net_event){NODE_DOWN, node, 0};
        for (uint8_t bus = 0; bus < NET_BUSES; bus++) {
            all[n++] = (struct net_event){CANLINK_UP, node, bus};
            all[n++] = (struct net_event){CANLINK_DOWN, node, bus};
        }
    }
    bool round_trip = true;
    bool distinct = true;
    for (int i = 0; i < n; i++) {
        uint8_t c = net_event_encode(&all[i]);
        struct net_event back;
        round_trip &= c != NET_EVENT_NONE && net_event_decode(c, &back) == 0 && same_event(&back, &all[i]);
        for (int j = 0; j < i; j++) {
            distinct &= c != net_event_encode(&all[j]);
        }
    }
    check("all 20 events: distinct codes that decode back to the event", round_trip && distinct);

    int decodable = 0;
    for (int c = 0; c < 256; c++) {
        struct net_event ev;
        decodable += net_event_decode((uint8_t)c, &ev) == 0;
    }
    check("of all 256 bytes, only those 20 decode (0, no event, doesn't)", decodable == 20);

    check("layout: CANLINK_UP N2 B = 0x56, NODE_DOWN N3 = 0x43, COCKPIT_OK = 0x10",
          code(CANLINK_UP, 2, NET_BUS_B) == 0x56 && code(NODE_DOWN, 3, 0) == 0x43 &&
              code(COCKPIT_OK, 0, 0) == 0x10);
    check("node 0 or 4, bus 2, type 6: encode to 0 (no event)",
          code(NODE_UP, 0, 0) == 0 && code(NODE_DOWN, 4, 0) == 0 && code(CANLINK_UP, 2, 2) == 0 &&
              code((enum net_event_type)6, 1, 0) == 0);
    check("unused fields ignored: COCKPIT_OK N3 B = COCKPIT_OK, NODE_UP N2 B = NODE_UP N2",
          code(COCKPIT_OK, 3, 1) == code(COCKPIT_OK, 0, 0) && code(NODE_UP, 2, 1) == code(NODE_UP, 2, 0));

    struct net_event untouched = {NODE_UP, 2, 0};
    check("decode 0x5F (CANLINK_UP on bus 3): -EINVAL, *ev untouched",
          net_event_decode(0x5F, &untouched) == -EINVAL && untouched.type == NODE_UP);
}

/* 2.2: entries change the graph only once committed, each applied once and in order */
static void log_commit(void) {
    printf("\nraft_log 2.2: append, commit, truncate\n");
    raft_log_init();
    check("empty: last index 0, last term 0, commit 0",
          raft_log_last_index() == 0 && raft_log_last_term() == 0 && raft_log_commit_index() == 0);

    bool indexes = raft_log_append(1, code(NODE_UP, 2, 0)) == 1 &&
                   raft_log_append(1, code(CANLINK_UP, 2, NET_BUS_A)) == 2 &&
                   raft_log_append(2, code(CANLINK_UP, 2, NET_BUS_B)) == 3;
    check("3 appends get indexes 1, 2, 3; last term 2", indexes && raft_log_last_term() == 2);
    check("entry 2 reads back: term 1, CANLINK_UP N2 A",
          raft_log_term(2) == 1 && raft_log_event(2) == code(CANLINK_UP, 2, NET_BUS_A));
    check("index 0 and 4 (past the end): term 0, no event",
          raft_log_term(0) == 0 && raft_log_event(0) == NET_EVENT_NONE && raft_log_term(4) == 0 &&
              raft_log_event(4) == NET_EVENT_NONE);
    check("appended, nothing committed: graph still all down", strcmp(graph(), ALL_DOWN) == 0);

    check("commit 2: applies entries 1-2, N2 up on A only",
          raft_log_commit(2) == 2 && strcmp(graph(), "N1 down | N2 up A+ B- | N3 down | cockpit ERROR") == 0);
    check("commit 2 again (bus B copy), then 1 (stale): applies nothing",
          raft_log_commit(2) == 0 && raft_log_commit(1) == 0 && raft_log_commit_index() == 2);
    check("commit 4, past the last entry: -EINVAL, commit stays 2",
          raft_log_commit(4) == -EINVAL && raft_log_commit_index() == 2);
    check("commit 3: applies only entry 3, N2 up on A and B",
          raft_log_commit(3) == 1 && strcmp(graph(), "N1 down | N2 up A+ B+ | N3 down | cockpit ERROR") == 0);

    check("append event 0, a bad code, or term 0: -EINVAL, log unchanged",
          raft_log_append(2, NET_EVENT_NONE) == -EINVAL && raft_log_append(2, 0xFF) == -EINVAL &&
              raft_log_append(0, code(COCKPIT_OK, 0, 0)) == -EINVAL && raft_log_last_index() == 3);

    raft_log_append(2, code(NODE_UP, 3, 0));                 /* 4 */
    raft_log_append(2, code(CANLINK_UP, 3, NET_BUS_A));      /* 5 */
    check("truncate 3 (committed) or 0 or 6: -EINVAL, log keeps 5",
          raft_log_truncate(3) == -EINVAL && raft_log_truncate(0) == -EINVAL &&
              raft_log_truncate(6) == -EINVAL && raft_log_last_index() == 5);
    check("truncate 4: entries 4-5 gone, last term back to 2",
          raft_log_truncate(4) == 0 && raft_log_last_index() == 3 && raft_log_last_term() == 2);
    check("next append reuses index 4; graph unchanged until commit",
          raft_log_append(3, code(COCKPIT_OK, 0, 0)) == 4 && raft_log_term(4) == 3 &&
              strcmp(graph(), "N1 down | N2 up A+ B+ | N3 down | cockpit ERROR") == 0);
    check("commit 4: cockpit OK",
          raft_log_commit(4) == 1 && strcmp(graph(), "N1 down | N2 up A+ B+ | N3 down | cockpit OK") == 0);

    raft_log_init();
    check("init: log empty and graph back to all down",
          raft_log_last_index() == 0 && raft_log_commit_index() == 0 && strcmp(graph(), ALL_DOWN) == 0);
}

/* 2.2: log_size entries fit, the next is refused, and committing them all applies each once */
static void log_full(void) {
    printf("\nraft_log 2.2: full log (log_size = %d)\n", RAFT_LOG_SIZE);
    raft_log_init();

    /* Alternate N1 up and down, so the graph after commit shows the last entry won */
    bool in_order = true;
    for (int i = 1; i <= RAFT_LOG_SIZE; i++) {
        in_order &= raft_log_append(1, code(i % 2 ? NODE_UP : NODE_DOWN, 1, 0)) == i;
    }
    check("log_size appends fill indexes 1 ... log_size", in_order && raft_log_last_index() == RAFT_LOG_SIZE);
    check("one more: -ENOSPC, log unchanged",
          raft_log_append(1, code(NODE_UP, 1, 0)) == -ENOSPC && raft_log_last_index() == RAFT_LOG_SIZE);
    check("commit all: each applied once, last entry (N1 down, even index) wins",
          raft_log_commit(RAFT_LOG_SIZE) == RAFT_LOG_SIZE && strcmp(graph(), ALL_DOWN) == 0);

    raft_log_init();
}

/* N1, the preset leader, booted early enough that its listen window ends at t: it leads
 * term 1 from its tick at t (REF and APPEND at t) */
static void boot_leader(int64_t t) {
    raft_init(RAFT_NODE_COCKPIT, t - RAFT_CLAIM_LISTEN_MS);
    now = t;
    raft_tick(t);
}

/* 2.3: each empty APPEND reads the leader's log as it is when sent */
static void append_from_log(void) {
    printf("\nraft 2.3: empty APPEND from the leader's log\n");
    raft_log_init();
    boot_leader(0);
    static const uint8_t empty_log[8] = {1}; /* term 1, the rest 0 */
    check("empty log, t=0: term 1, prevLogIndex 0, prevLogTerm 0, leaderCommit 0",
          memcmp(last_append, empty_log, 8) == 0);

    /* Both indexes > 255, so a byte-order mistake shows */
    for (int i = 0; i < 300; i++) {
        raft_log_append(1, code(NODE_UP, 2, 0));
    }
    raft_log_commit(258);
    for (now = RAFT_TICK_MS; now <= 100; now += RAFT_TICK_MS) {
        raft_tick(now);
    }
    static const uint8_t grown[8] = {1, 0x2C, 0x01, 1, 0, 0, 0x02, 0x01};
    check("300 entries, 258 committed, t=100: prev 300 (2C 01), term 1, commit 258 (02 01)",
          memcmp(last_append, grown, 8) == 0);

    raft_log_init();
}

/* Deliver one REF or APPEND, as the CAN RX path would */
static void rx(uint16_t base, uint8_t sender, uint8_t term, int64_t t) {
    uint8_t data[8] = {term};
    now = t;
    raft_on_frame(RAFT_ID(base, sender), data, t);
}

/* Tick a leader for 1 s with the given tick spacing, counting frames */
static void run_leader(int64_t start, const int *steps, int nsteps) {
    refs = appends = 0;
    raft_init(RAFT_NODE_COCKPIT, start - RAFT_CLAIM_LISTEN_MS);
    int i = 0;
    for (now = start; now <= start + 1000; now += steps[i++ % nsteps]) {
        raft_tick(now);
    }
}

/* 3.2: the frames themselves, to read by eye. REF at 0, 20, ... 200; APPEND at 0, 100, 200. */
static void leader_trace(void) {
    printf("\nraft 3.2: N1 as default leader, ticks 0..200 ms\n");
    print_tx = true;
    raft_init(RAFT_NODE_COCKPIT, -RAFT_CLAIM_LISTEN_MS);
    for (now = 0; now <= 200; now += RAFT_TICK_MS) {
        raft_tick(now);
    }
    print_tx = false;
}

/* 3.2: timing with clocks like the real ones */
static void leader_timing(void) {
    static const int even[] = {5};
    static const int jitter[] = {5, 6, 5, 4, 7, 5}; /* ticks a ms or two late or early */

    printf("\nraft 3.2: leader timing over 1 s\n");
    run_leader(0, even, 1);
    check("ticks every 5 ms from t=0: 51 REF, 11 APPEND", refs == 51 && appends == 11);
    run_leader(1000003, even, 1);
    check("Pi booted 1,000,003 ms ago: 51 REF, 11 APPEND", refs == 51 && appends == 11);
    run_leader(0, jitter, 6);
    /* The last tick is at t=997, so the frames due at t=1000 aren't sent yet */
    check("ticks with 1-2 ms jitter: 50 REF, 10 APPEND", refs == 50 && appends == 10);
}

/* 3.3 and 3.4: N2 following, with the cases the CAN bus will throw at it */
static void follower(void) {
    printf("\nraft 3.3/3.4: N2 as follower\n");
    raft_init(RAFT_NODE_STEERING, 0);
    check("boot: follower, term 1, leader N1",
          raft_get_role() == RAFT_FOLLOWER && raft_get_term() == 1 && raft_get_leader() == 1);
    check("boot: SYNC off until the first REF", !raft_in_sync(0));

    rx(RAFT_ID_REF, 1, 1, 10);
    check("REF from N1 at t=10: SYNC on, heard at 10", raft_in_sync(10) && raft_last_heard() == 10);
    rx(RAFT_ID_REF, 1, 1, 10); /* the other bus's copy */
    check("same REF again (bus B copy): nothing changes",
          raft_get_term() == 1 && raft_get_leader() == 1 && raft_last_heard() == 10);
    check("t=69, 59 ms after the REF: SYNC still on", raft_in_sync(69));
    check("t=70, 60 ms after the REF: SYNC off", !raft_in_sync(70));

    rx(RAFT_ID_APPEND, 1, 1, 80);
    check("APPEND at t=80: leader heard, but SYNC stays off (no REF)",
          raft_last_heard() == 80 && !raft_in_sync(80));

    rx(RAFT_ID_REF, 3, 2, 100);
    check("REF from N3 in term 2: adopt term 2, leader N3, SYNC on",
          raft_get_term() == 2 && raft_get_leader() == 3 && raft_in_sync(100));
    rx(RAFT_ID_REF, 1, 1, 105);
    check("stale REF from N1 in term 1: ignored",
          raft_get_term() == 2 && raft_get_leader() == 3 && raft_last_heard() == 100);
    rx(RAFT_ID_REF, 2, 5, 110);
    check("frame with our own ID (relayed back): ignored", raft_get_term() == 2);
}

/* ---- step 4: one node's side of replication. The three together run in sim.c. ---- */

/* Deliver an APPEND (layout in raft.h) */
static void rx_append(uint8_t leader, uint8_t term, uint16_t prev, uint8_t prev_term, uint8_t entry_term,
                      uint8_t event, uint16_t commit, int64_t t) {
    uint8_t data[8] = {term, prev & 0xFF, prev >> 8, prev_term, entry_term, event, commit & 0xFF, commit >> 8};
    now = t;
    raft_on_frame(RAFT_ID(RAFT_ID_APPEND, leader), data, t);
}

/* The last APPEND_RESP said this */
static bool replied(bool success, uint16_t index) {
    return last_resp[1] == success && (last_resp[2] | last_resp[3] << 8) == index;
}

/* 4.1: Figure 2's AppendEntries receiver, including what only a later leader would send */
static void follower_append(void) {
    printf("\nraft 4.1: N2 follower takes APPENDs\n");
    uint8_t n2_up = code(NODE_UP, 2, 0);
    uint8_t n2_a = code(CANLINK_UP, 2, NET_BUS_A);
    uint8_t n2_b = code(CANLINK_UP, 2, NET_BUS_B);
    uint8_t n3_up = code(NODE_UP, 3, 0);
    uint8_t n3_a = code(CANLINK_UP, 3, NET_BUS_A);
    raft_log_init();
    raft_init(RAFT_NODE_STEERING, 0);
    resps = 0;

    rx_append(1, 1, 0, 0, 1, n2_up, 0, 10);
    check("entry 1: appended, not applied; reply success 1",
          raft_log_last_index() == 1 && raft_log_commit_index() == 0 && strcmp(graph(), ALL_DOWN) == 0 &&
              resps == 1 && replied(true, 1));
    rx_append(1, 1, 0, 0, 1, n2_up, 0, 11);
    check("its bus B copy 1 ms later: ignored, no second reply", raft_log_last_index() == 1 && resps == 1);
    rx_append(1, 1, 1, 1, 1, n2_a, 1, 20);
    check("entry 2 with leaderCommit 1: entry 1 applied; reply success 2",
          raft_log_last_index() == 2 && raft_log_commit_index() == 1 &&
              strcmp(graph(), "N1 down | N2 up A- B- | N3 down | cockpit ERROR") == 0 && replied(true, 2));
    rx_append(1, 1, 2, 1, 0, 0, 2, 100);
    check("empty APPEND, prev 2 term 1, commit 2: entry 2 applied",
          raft_log_commit_index() == 2 && strcmp(graph(), "N1 down | N2 up A+ B- | N3 down | cockpit ERROR") == 0 &&
              replied(true, 2));
    rx_append(1, 1, 5, 1, 0, 0, 2, 200);
    check("prev 5, past our last: reply failure 2 (we match through 2 at most)",
          replied(false, 2) && raft_log_last_index() == 2);
    rx_append(1, 1, 2, 7, 0, 0, 2, 300);
    check("prev 2 but term 7, not ours: reply failure 1", replied(false, 1));

    /* N1 leaves entries 3 and 4 uncommitted, then N3 leads term 2. Its log has the same
     * entry 3 but a different entry 4. */
    rx_append(1, 1, 2, 1, 1, n2_b, 2, 400);
    rx_append(1, 1, 3, 1, 1, n3_up, 2, 410);
    rx_append(3, 2, 2, 1, 1, n2_b, 5, 500);
    check("N3 (term 2) resends entry 3 with commit 5: apply through 3, not our stale 4",
          raft_get_term() == 2 && raft_get_leader() == 3 && raft_log_commit_index() == 3 && replied(true, 3));
    rx_append(3, 2, 3, 1, 2, n3_a, 5, 510);
    check("its entry 4 (term 2) replaces our stale one, then applies",
          raft_log_last_index() == 4 && raft_log_term(4) == 2 && raft_log_event(4) == n3_a &&
              raft_log_commit_index() == 4 && replied(true, 4));
    rx_append(3, 2, 0, 0, 2, n3_up, 5, 520);
    check("an entry 1 that conflicts with our committed one: refused, ours kept",
          replied(false, 0) && raft_log_term(1) == 1 && raft_log_last_index() == 4);
    raft_log_init();
}

/* 4.2: raft_submit on a leader and on a follower, and EVENT at the leader */
static void submit(void) {
    printf("\nraft 4.2: raft_submit and EVENT\n");
    raft_log_init();
    boot_leader(0);
    check("leader: appends to its own log, returns the index",
          raft_submit(code(NODE_UP, 1, 0)) == 1 && raft_log_last_index() == 1);
    check("a bad code or 0: -EINVAL", raft_submit(0xFF) == -EINVAL && raft_submit(NET_EVENT_NONE) == -EINVAL);

    uint8_t ev[8] = {1, 7, code(NODE_DOWN, 3, 0)};
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), ev, 10);
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), ev, 11);
    check("EVENT from N2, then its bus B copy: appended once as entry 2",
          raft_log_last_index() == 2 && raft_log_event(2) == code(NODE_DOWN, 3, 0));
    ev[1] = 8;
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), ev, 12);
    check("the same event with the next seq: a new entry 3", raft_log_last_index() == 3);
    uint8_t first[8] = {1, 9, code(NODE_UP, 3, 0)};
    uint8_t second[8] = {1, 10, code(CANLINK_UP, 3, NET_BUS_A)};
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), first, 20);
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), second, 20);
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), first, 21);
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 2), second, 21);
    check("two EVENTs arriving A1 A2 B1 B2: two entries (4, 5), not four", raft_log_last_index() == 5);

    raft_log_init();
    raft_init(RAFT_NODE_STEERING, 0);
    events = 0;
    check("follower: sends EVENT seq 0 to the leader, appends nothing itself",
          raft_submit(code(CANLINK_DOWN, 3, NET_BUS_B)) == 0 && events == 1 && last_event[1] == 0 &&
              last_event[2] == code(CANLINK_DOWN, 3, NET_BUS_B) && raft_log_last_index() == 0);
    raft_submit(code(NODE_UP, 3, 0));
    check("its next event: seq 1", events == 2 && last_event[1] == 1);
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 3), ev, 20);
    check("a follower ignores another node's EVENT", raft_log_last_index() == 0);

    uint8_t resp[8] = {2}; /* a term 2 frame that isn't from a leader */
    raft_on_frame(RAFT_ID(RAFT_ID_APPEND_RESP, 3), resp, 30);
    check("term 2 began and nobody leads it yet: -EAGAIN",
          raft_get_leader() == RAFT_NODE_NONE && raft_submit(code(NODE_UP, 3, 0)) == -EAGAIN);
    raft_log_init();
}

/* Split a console line on spaces, as cockpit.c and the STM32 shell do, then parse it */
static int parse(const char *line, struct net_event *ev) {
    char buf[64];
    char *argv[8];
    int argc = 0;
    snprintf(buf, sizeof(buf), "%s", line);
    for (char *tok = strtok(buf, " "); tok != NULL && argc < 8; tok = strtok(NULL, " ")) {
        argv[argc++] = tok;
    }
    return net_event_parse(argc, argv, ev);
}

/* 4.9: what the consoles (net_shell.c and raft_node.c on the STM32s, cockpit.c) build on */
static void console(void) {
    printf("\nraft 4.9: console helpers\n");
    struct net_event ev;
    check("\"node up 2\", \"link down 3 A\", \"link up 1 b\", \"cockpit error\"",
          parse("node up 2", &ev) == 0 && net_event_encode(&ev) == code(NODE_UP, 2, 0) &&
              parse("link down 3 A", &ev) == 0 && net_event_encode(&ev) == code(CANLINK_DOWN, 3, NET_BUS_A) &&
              parse("link up 1 b", &ev) == 0 && net_event_encode(&ev) == code(CANLINK_UP, 1, NET_BUS_B) &&
              parse("cockpit error", &ev) == 0 && net_event_encode(&ev) == code(COCKPIT_ERROR, 0, 0));
    static const char *const bad[] = {"node up 4", "node up 0", "node up 3x", "node sideways 3", "node up 3 A",
                                      "link up 2 C", "link up 2", "cockpit fine", "bogus up", ""};
    struct net_event before = ev;
    int rejected = 0;
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        rejected += parse(bad[i], &ev) == -EINVAL;
    }
    check("10 bad lines (node 4, bus C, a word missing or extra...): -EINVAL",
          rejected == 10 && same_event(&ev, &before));

    raft_log_init();
    uint8_t n3_a_down = code(CANLINK_DOWN, 3, NET_BUS_A);
    raft_log_append(1, code(NODE_UP, 3, 0));
    raft_log_append(1, n3_a_down);
    raft_log_append(1, code(NODE_UP, 2, 0));
    raft_log_append(2, n3_a_down);
    raft_log_commit(3);
    check("find: the first committed match after `after` (entry 2)",
          raft_log_find_committed(n3_a_down, 0) == 2 && raft_log_find_committed(n3_a_down, 1) == 2);
    check("find: a match past commitIndex (entry 4) isn't reported yet",
          raft_log_find_committed(n3_a_down, 2) == 0);
    raft_log_commit(4);
    check("... until it commits", raft_log_find_committed(n3_a_down, 2) == 4);
    check("find: no entry holds the event: 0", raft_log_find_committed(code(COCKPIT_OK, 0, 0), 0) == 0);

    raft_log_append(2, code(COCKPIT_OK, 0, 0));
    char line[64];
    raft_log_format(4, line, sizeof(line));
    check("format: entry 4", strcmp(line, "    4  term 2   CANLINK_DOWN N3 A") == 0);
    raft_log_format(5, line, sizeof(line));
    check("format: entry 5, not committed", strcmp(line, "    5  term 2   COCKPIT_OK  (not committed)") == 0);
    check("format: index 0 or past the end: -EINVAL, buf untouched",
          raft_log_format(0, line, sizeof(line)) == -EINVAL && raft_log_format(6, line, sizeof(line)) == -EINVAL &&
              strcmp(line, "    5  term 2   COCKPIT_OK  (not committed)") == 0);
    raft_log_init();

    char config[192];
    raft_config_format(config, sizeof(config));
    printf("  %s\n", config);
    check("config: every key through debug, within 192 bytes",
          strncmp(config, "N=", 2) == 0 && strstr(config, " debug=") != NULL && strlen(config) < sizeof(config) - 1);
}

/* 3.3: a leader that hears a higher term steps down and stops sending */
static void leader_steps_down(void) {
    printf("\nraft 3.3: N1 leader hears term 2\n");
    boot_leader(0);
    rx(RAFT_ID_REF, 3, 2, 5);
    check("REF from N3 in term 2: N1 becomes follower of N3",
          raft_get_role() == RAFT_FOLLOWER && raft_get_term() == 2 && raft_get_leader() == 3);
    refs = appends = 0;
    for (now = 5; now <= 200; now += RAFT_TICK_MS) {
        raft_tick(now);
    }
    check("then sends no REF or APPEND", refs == 0 && appends == 0);
}

/* ---- step 5: elections, one node's side. The three together run in sim.c. ---- */

/* Deliver a REQ_VOTE or a VOTE (layouts in raft.h) */
static void rx_req_vote(uint8_t cand, uint8_t term, bool prevote, uint16_t last_index, uint8_t last_term,
                        int64_t t) {
    uint8_t data[8] = {term, prevote, last_index & 0xFF, last_index >> 8, last_term};
    now = t;
    raft_on_frame(RAFT_ID(RAFT_ID_REQ_VOTE, cand), data, t);
}

static void rx_vote(uint8_t voter, uint8_t term, bool granted, uint8_t cand, bool prevote, int64_t t) {
    uint8_t data[8] = {term, granted, cand, prevote};
    now = t;
    raft_on_frame(RAFT_ID(RAFT_ID_VOTE, voter), data, t);
}

/* The last VOTE sent said this */
static bool answered(uint8_t term, bool granted, uint8_t cand, bool prevote) {
    return last_vote[0] == term && last_vote[1] == granted && last_vote[2] == cand && last_vote[3] == prevote;
}

/* Tick every RAFT_TICK_MS after now, up to t */
static void tick_until(int64_t t) {
    while (now + RAFT_TICK_MS <= t) {
        now += RAFT_TICK_MS;
        raft_tick(now);
    }
}

/* 5.1: the timeouts themselves: Part 2's formula, one draw per (seed, node, term) */
static void election_timeouts(void) {
    printf("\nraft 5.1: election timeouts\n");
    int lo = 1000, hi = 0;
    bool same_twice = true;
    for (uint32_t seed = 0; seed < 100; seed++) {
        for (uint8_t node = 1; node <= RAFT_N; node++) {
            for (uint8_t term = 0; term < 20; term++) {
                int t = raft_election_timeout_ms(seed, node, term);
                lo = t < lo ? t : lo;
                hi = t > hi ? t : hi;
                same_twice = same_twice && t == raft_election_timeout_ms(seed, node, term);
            }
        }
    }
    int base = RAFT_MISSED_HEARTBEATS * RAFT_HEARTBEAT_MS;
    check("6000 draws: all of 60..99 ms used, nothing outside",
          lo == base && hi == base + RAFT_ELECTION_JITTER_MS);
    check("the same (seed, node, term) always draws the same", same_twice);
    printf("  seed %u, term 1: N1 %d ms, N2 %d ms, N3 %d ms\n", (unsigned)RAFT_SEED,
           raft_election_timeout_ms(RAFT_SEED, 1, 1), raft_election_timeout_ms(RAFT_SEED, 2, 1),
           raft_election_timeout_ms(RAFT_SEED, 3, 1));
    bool pi_first = true;
    for (uint8_t term = 0; term <= 2; term++) {
        int pi = raft_election_timeout_ms(9, 1, term);
        pi_first = pi_first && pi + 10 <= raft_election_timeout_ms(9, 2, term) &&
                   pi + 10 <= raft_election_timeout_ms(9, 3, term);
    }
    check("seed 9: the Pi times out 10+ ms before both STM32s in terms 0-2", pi_first);
}

/* 5.2: N2 times out on its leader, asks for pre-votes, then votes, and wins */
static void candidate(void) {
    printf("\nraft 5.2: N2 times out, campaigns, wins\n");
    raft_log_init();
    raft_init(RAFT_NODE_STEERING, 0);
    vote_reqs = 0;
    now = 0;
    tick_until(1000);
    check("preset leader N1 never heard: no election for 1 s", vote_reqs == 0 && raft_get_role() == RAFT_FOLLOWER);

    int timeout = raft_election_timeout_ms(RAFT_SEED, RAFT_NODE_STEERING, 1);
    rx(RAFT_ID_REF, 1, 1, 1000);
    tick_until(1000 + timeout - 1);
    check("REF at t=1000: nothing sent before 1000 + timeout", vote_reqs == 0);
    tick_until(1000 + timeout + RAFT_TICK_MS - 1);
    check("the next tick: pre-vote for term 2 (b1 = 1); our term is still 1",
          vote_reqs == 1 && last_req[0] == 2 && last_req[1] == 1 && raft_get_term() == 1 &&
              raft_get_role() == RAFT_PRECANDIDATE);

    rx_vote(3, 1, false, 2, true, now);
    check("a pre-vote no from N3 carrying our term: nothing changes",
          raft_get_role() == RAFT_PRECANDIDATE && raft_get_term() == 1);
    rx_vote(1, 2, true, 3, true, now);
    check("N1's pre-vote yes for N3, not us: ignored, term not adopted",
          raft_get_role() == RAFT_PRECANDIDATE && raft_get_term() == 1);
    rx_vote(3, 2, true, 2, true, now);
    check("N3's pre-vote yes: candidate in term 2, REQ_VOTE term 2 (b1 = 0)",
          raft_get_role() == RAFT_CANDIDATE && raft_get_term() == 2 && vote_reqs == 2 && last_req[0] == 2 &&
              last_req[1] == 0 && raft_get_leader() == RAFT_NODE_NONE);
    rx_vote(3, 2, true, 2, true, now + 1);
    check("the pre-vote yes's bus B copy: no second REQ_VOTE", vote_reqs == 2);
    refs = appends = 0;
    rx_vote(3, 2, true, 2, false, now + 1);
    check("N3's vote: leader of term 2", raft_get_role() == RAFT_LEADER && raft_get_leader() == RAFT_NODE_STEERING);
    check("REF and an empty APPEND in term 2 at once, not on the next tick",
          refs == 1 && appends == 1 && last_append[0] == 2);
    tick_until(now + RAFT_HEARTBEAT_MS - 1);
    check("the next REF a heartbeat later", refs == 1);
    tick_until(now + RAFT_TICK_MS);
    check("... then on the 20 ms grid", refs == 2);

    printf("\nraft 5.2: N2 cut off: pre-votes go unanswered\n");
    raft_init(RAFT_NODE_STEERING, 0);
    rx(RAFT_ID_REF, 1, 1, 0);
    vote_reqs = 0;
    tick_until(1000);
    check("1 s with nobody answering: a pre-vote every timeout, term still 1",
          vote_reqs >= 1000 / (timeout + RAFT_TICK_MS) && raft_get_term() == 1 &&
              raft_get_role() == RAFT_PRECANDIDATE);
    rx(RAFT_ID_REF, 1, 1, 1000);
    check("N1's REF reaches it again: back to follower of N1",
          raft_get_role() == RAFT_FOLLOWER && raft_get_leader() == 1 && raft_get_term() == 1);

    printf("\nraft 5.2: two candidates split the vote\n");
    raft_init(RAFT_NODE_STEERING, 0);
    rx(RAFT_ID_REF, 1, 1, 0);
    tick_until(timeout + RAFT_TICK_MS - 1);
    rx_vote(3, 2, true, 2, true, now);
    rx_req_vote(3, 2, false, 0, 0, now);
    check("N3 is a candidate in term 2 too: no vote for it (voted for ourselves)",
          answered(2, false, 3, false) && raft_get_role() == RAFT_CANDIDATE);
    int timeout2 = raft_election_timeout_ms(RAFT_SEED, RAFT_NODE_STEERING, 2);
    vote_reqs = 0;
    tick_until(now + timeout2 + RAFT_TICK_MS);
    check("no majority within term 2's timeout: pre-vote for term 3",
          vote_reqs == 1 && last_req[0] == 3 && last_req[1] == 1 && raft_get_role() == RAFT_PRECANDIDATE);
}

/* 5.3: N3 answers pre-votes and votes, and replies to stale terms */
static void voter(void) {
    printf("\nraft 5.3: N3 answers (log: 2 entries of term 1)\n");
    raft_log_init();
    raft_log_append(1, code(NODE_UP, 1, 0));
    raft_log_append(1, code(NODE_UP, 2, 0));
    raft_init(RAFT_NODE_DRIVETRAIN, 0);
    rx(RAFT_ID_REF, 1, 1, 0);

    rx_req_vote(2, 2, true, 2, 1, 30);
    check("pre-vote 30 ms after N1's REF: no, carrying our term 1", answered(1, false, 2, true));
    rx_req_vote(2, 2, true, 2, 1, 70);
    check("70 ms after: yes, carrying term 2; our term stays 1",
          answered(2, true, 2, true) && raft_get_term() == 1);
    rx_req_vote(2, 2, true, 1, 1, 71);
    check("pre-vote from a shorter log: no", answered(1, false, 2, true));
    rx_req_vote(2, 1, true, 2, 1, 72);
    check("pre-vote for term 1, not past ours: no", answered(1, false, 2, true));

    rx_req_vote(2, 2, false, 2, 1, 75);
    check("RequestVote from N2 in term 2: adopt term 2, vote yes",
          raft_get_term() == 2 && answered(2, true, 2, false) && raft_get_leader() == RAFT_NODE_NONE);
    rx_req_vote(2, 2, false, 2, 1, 76);
    check("its bus B copy: yes again", answered(2, true, 2, false));
    rx_req_vote(1, 2, false, 5, 1, 77);
    check("N1 asks in term 2 too: no, we voted for N2", answered(2, false, 1, false));
    rx_req_vote(1, 3, false, 1, 1, 78);
    check("N1 in term 3 with a shorter log: adopt term 3, no", raft_get_term() == 3 && answered(3, false, 1, false));
    rx_req_vote(1, 3, false, 3, 1, 79);
    check("N1 again in term 3, longer log: yes (no vote cast in term 3)", answered(3, true, 1, false));
    rx_req_vote(2, 2, false, 2, 1, 80);
    check("stale RequestVote, term 2: no, carrying our term 3", answered(3, false, 2, false) && raft_get_term() == 3);
    rx_req_vote(2, 4, false, 1, 2, 81);
    check("term 4, 1 entry of term 2 beats our 2 of term 1: yes", answered(4, true, 2, false));

    resps = 0;
    rx(RAFT_ID_APPEND, 1, 1, 82);
    check("stale APPEND (term 1): APPEND_RESP no, our term 4, we match 2 at most",
          resps == 1 && last_resp[0] == 4 && replied(false, 2));
    resps = 0;
    int votes_before = votes;
    rx(RAFT_ID_REF, 1, 1, 83);
    check("stale REF: no answer", resps == 0 && votes == votes_before && raft_get_term() == 4);
    raft_log_init();

    printf("\nraft 5.3: a stale leader steps down on the answer\n");
    raft_init(RAFT_NODE_COCKPIT, 0);
    uint8_t answer[8] = {4, 0, 2, 0};
    raft_on_frame(RAFT_ID(RAFT_ID_APPEND_RESP, 3), answer, 5);
    check("N1, leader of term 1, hears term 4 in an answer: follower, no leader known",
          raft_get_role() == RAFT_FOLLOWER && raft_get_term() == 4 && raft_get_leader() == RAFT_NODE_NONE);

    char line[128];
    raft_status_format(line, sizeof(line), 5);
    printf("  %s\n", line);
    static const char status[] = "follower | term 4 | leader none | SYNC off | election timeout ";
    check("status: role, term, leader none, SYNC, timeout", strncmp(line, status, strlen(status)) == 0);
}


/* ---- steps 6-9 ---- */

static int64_t llabs64(int64_t v) {
    return v < 0 ? -v : v;
}

/* 7: the global clock on a follower (N2) of master N1 */
static void time_sync(void) {
    printf("\ntime 7.1: N2 follows N1's REFs\n");
    timesync_init(RAFT_NODE_STEERING);
    int64_t d = timesync_delay_us(RAFT_NODE_COCKPIT);
    check("before any REF: global = local", timesync_global_us(12345) == 12345);
    timesync_on_ref(1, 5000000, 1000);
    check("first REF sets the clock: global(rx) = L + delay", timesync_global_us(1000) == 5000000 + d);

    /* N1's clock runs 50 ppm fast against N2's: 20 ms of N2's time is 20.001 ms of N1's */
    int64_t worst = 0;
    for (int k = 1; k <= 200; k++) {
        int64_t rx = 1000 + (int64_t)k * 20000;
        int64_t master = 5000000 + (int64_t)k * 20001;
        timesync_on_ref(1, master, rx);
        if (k > 100) {
            int64_t truth = master + d + 10000; /* 10 ms after the REF */
            worst = llabs64(timesync_global_us(rx + 10000) - truth) > worst ? llabs64(timesync_global_us(rx + 10000) - truth) : worst;
        }
    }
    struct timesync_stats st = timesync_get_stats();
    printf("  drift %d ppb (true 50000), worst error 10 ms after a REF: %lld us\n", (int)st.ppb, (long long)worst);
    check("4 s of REFs: drift within 1 ppm of 50 ppm", llabs64(st.ppb - 50000) < 1000);
    check("between REFs the drift correction keeps it within 2 us", worst <= 2);
    check("local(global(t)) = t within 1 us", llabs64(timesync_local_us(timesync_global_us(7777777)) - 7777777) <= 1);

    int64_t rx = 1000 + 201 * 20000;
    int64_t before = timesync_global_us(rx);
    timesync_on_ref(1, 5000000 + 201 * 20001, rx + 250); /* waited 250 us for the bus */
    check("a REF that queued 250 us behind another frame is ignored",
          timesync_global_us(rx) == before && timesync_get_stats().outliers == 1);

    printf("\ntime 7.2: N2 becomes the master (handover)\n");
    int64_t t = rx + 20000;
    int64_t extrapolated = timesync_global_us(t + 100000);
    timesync_set_master(true);
    check("its global clock runs on unchanged: no step", timesync_global_us(t + 100000) == extrapolated);
    timesync_on_ref(3, 1, t);
    check("a stale REF can't move the master", timesync_global_us(t + 100000) == extrapolated);

    printf("\ntime 7.2: the new master's first REF queued behind the election\n");
    timesync_init(RAFT_NODE_DRIVETRAIN);
    for (int k = 0; k < 10; k++) {
        timesync_on_ref(1, 1000000 + k * 20000, k * 20000);
    }
    timesync_on_ref(2, 1000000 + 10 * 20000, 10 * 20000 + 354); /* N2's first REF, 354 us late */
    struct timesync_stats hs = timesync_get_stats();
    check("a late first REF from the new master: discarded, no handover reported",
          hs.handovers == 0 && hs.source == 1);
    /* N3's clock reads N1's time + its delay; N2's REF carries N2's clock, which we expect
     * at our clock - N2's delay. 57 us early: */
    timesync_on_ref(2, 1000000 + 11 * 20000 + timesync_delay_us(1) - timesync_delay_us(2) - 57, 11 * 20000);
    hs = timesync_get_stats();
    check("its next REF: the handover, with the step actually applied (-57 us)",
          hs.handovers == 1 && hs.prev_source == 1 && hs.source == 2 && hs.handover_step_us == -57);

    printf("\ntime 7.3: N3 relays a REF\n");
    timesync_init(RAFT_NODE_DRIVETRAIN);
    int64_t fwd = timesync_relay_master_us(1, 1000000, 500, 820);
    check("relayed master time = L + hop delay + 320 us in the relay + its TX - N1's TX",
          fwd == 1000000 + timesync_delay_us(1) + 320 + TIMESYNC_TX_US_STM32 - TIMESYNC_TX_US_PI);
}

static void obs_all(struct netwatch_obs *o) {
    *o = (struct netwatch_obs){0};
    for (int n = 1; n <= RAFT_N; n++) {
        o->alive[n] = true;
        o->direct[n][0] = o->direct[n][1] = true;
        o->fresh[n][0] = o->fresh[n][1] = true;
    }
}

/* 6.1: what each node can vouch for */
static void link_watch(void) {
    printf("\nnetwatch 6.1: facts from what a node hears\n");
    struct netwatch_obs o;
    struct netwatch_view v;

    obs_all(&o);
    netwatch_view(&o, 2, &v);
    check("all up: every node, the others' links and the cockpit up",
          v.node[1] == FACT_UP && v.node[3] == FACT_UP && v.link[1][0] == FACT_UP && v.link[3][1] == FACT_UP &&
              v.cockpit == FACT_UP);
    check("... but never our own links up: others vouch for those", v.link[2][0] == FACT_UNKNOWN);

    o.direct[3][NET_BUS_A] = o.fresh[3][NET_BUS_A] = false; /* N3's A jumper out, seen from N2 */
    netwatch_view(&o, 2, &v);
    check("N3 off A, seen from N2: N3 A down, N3 B up", v.link[3][0] == FACT_DOWN && v.link[3][1] == FACT_UP);

    obs_all(&o); /* the same, seen from N3: nobody heard on A */
    o.direct[1][NET_BUS_A] = o.direct[2][NET_BUS_A] = false;
    o.fresh[1][NET_BUS_A] = o.fresh[2][NET_BUS_A] = false;
    netwatch_view(&o, 3, &v);
    check("... seen from N3: its own A down; nothing said about others on A",
          v.link[3][0] == FACT_DOWN && v.link[3][1] == FACT_UNKNOWN && v.link[1][0] == FACT_UNKNOWN);

    /* Board, 2026-11-16: 0-20 ms after N3's A jumper came out, N2's heartbeat on A had
     * aged out but N1's (a different phase) hadn't yet. N3 blamed N2's link. */
    obs_all(&o);
    o.direct[2][NET_BUS_A] = false;                         /* 50 ms old */
    o.fresh[1][NET_BUS_A] = o.fresh[2][NET_BUS_A] = false; /* N1 35 ms old: heard, not fresh */
    netwatch_view(&o, 3, &v);
    check("N3's own A just died, N1 still within 50 ms: N2's A is not blamed",
          v.link[2][0] == FACT_UNKNOWN && v.link[3][0] == FACT_UNKNOWN);

    obs_all(&o); /* N1 off B, N3 off A, seen from N1: N3 only through N2's relay */
    o.direct[2][NET_BUS_B] = o.direct[3][NET_BUS_B] = false;
    o.fresh[2][NET_BUS_B] = o.fresh[3][NET_BUS_B] = false;
    o.direct[3][NET_BUS_A] = o.fresh[3][NET_BUS_A] = false;
    netwatch_view(&o, 1, &v);
    check("N1 off B, N3 off A, seen from N1: N3 up (relayed), N3 A down, N3 B unknown",
          v.node[3] == FACT_UP && v.link[3][0] == FACT_DOWN && v.link[3][1] == FACT_UNKNOWN && v.link[1][1] == FACT_DOWN);
    o.alive[3] = false; /* before the relay starts */
    netwatch_view(&o, 1, &v);
    check("... before the relay starts: N3 unknown, never reported down", v.node[3] == FACT_UNKNOWN);

    /* Board, 2026-11-16: N2's B transmitter dead, its receiver fine. N2 hears N1 and N3 on
     * B; nobody hears N2 there. */
    obs_all(&o);
    netwatch_view(&o, 2, &v);
    struct netwatch_view v3;
    struct netwatch_obs o3;
    obs_all(&o3);
    o3.direct[2][NET_BUS_B] = o3.fresh[2][NET_BUS_B] = false;
    netwatch_view(&o3, 3, &v3);
    check("N2 can't send on B: N3 says N2 B down, and N2 doesn't contradict it",
          v3.link[2][1] == FACT_DOWN && v.link[2][1] == FACT_UNKNOWN);

    o = (struct netwatch_obs){0}; /* N2 isolated: hears nobody */
    netwatch_view(&o, 2, &v);
    check("isolated: only itself up; no claims on links or others",
          v.node[2] == FACT_UP && v.link[2][0] == FACT_UNKNOWN && v.node[1] == FACT_UNKNOWN);

    obs_all(&o); /* the Pi powered off, seen from N2 */
    o.alive[1] = false;
    o.direct[1][0] = o.direct[1][1] = false;
    o.fresh[1][0] = o.fresh[1][1] = false;
    netwatch_view(&o, 2, &v);
    check("Pi off, seen from N2: N1 down, cockpit ERROR", v.node[1] == FACT_DOWN && v.cockpit == FACT_DOWN);
}

static struct network graph_all(void) {
    struct network g = {.cockpit_ok = true};
    for (int n = 0; n < NET_NODES; n++) {
        g.node_up[n] = true;
        g.link_up[n][0] = g.link_up[n][1] = true;
    }
    return g;
}

/* 6.2: the routing table from the graph */
static void routing(void) {
    printf("\nroute 6.2: routing tables from getNetwork()\n");
    struct network g = graph_all();
    struct route_table t;
    char line[64];
    bool any = false;

    for (uint8_t me = 1; me <= 3; me++) {
        route_build(&g, me, &t);
        for (int s = 1; s <= 3; s++) {
            any = any || t.relay[s][0] || t.relay[s][1];
        }
    }
    check("all links up: nobody relays", !any);

    g.link_up[2][NET_BUS_A] = false; /* N3 off A */
    any = false;
    for (uint8_t me = 1; me <= 3; me++) {
        route_build(&g, me, &t);
        for (int s = 1; s <= 3; s++) {
            any = any || t.relay[s][0] || t.relay[s][1];
        }
    }
    check("N3 off A only: still nobody (everyone hears N3 on B)", !any);

    g.link_up[0][NET_BUS_B] = false; /* and N1 off B */
    route_build(&g, 2, &t);
    route_format(&t, line, sizeof(line));
    printf("  N2: %s\n", line);
    check("N1 off B and N3 off A: N2 relays N1 A->B and N3 B->A",
          t.relay[1][NET_BUS_A] && t.relay[3][NET_BUS_B] && !t.relay[1][NET_BUS_B] && !t.relay[3][NET_BUS_A]);
    struct route_table t1, t3;
    route_build(&g, 1, &t1);
    route_build(&g, 3, &t3);
    check("... and N1 and N3 relay nothing (they're on one bus)", !t1.relay[3][1] && !t3.relay[1][0]);

    g = graph_all(); /* checkoff step 9: N2 leads, N2 off B, N3 off A: the Pi bridges */
    g.link_up[1][NET_BUS_B] = false;
    g.link_up[2][NET_BUS_A] = false;
    route_build(&g, 1, &t);
    route_format(&t, line, sizeof(line));
    printf("  N1: %s\n", line);
    check("N2 off B, N3 off A: the Pi relays N2 A->B and N3 B->A", t.relay[2][NET_BUS_A] && t.relay[3][NET_BUS_B]);

    struct netwatch_obs o;
    obs_all(&o);
    o.direct[1][NET_BUS_B] = o.fresh[1][NET_BUS_B] = false;
    o.direct[3][NET_BUS_A] = o.fresh[3][NET_BUS_A] = false;
    check("the same rule on what N2 hears, before the log has it",
          route_relay_observed(&o, 2, 1, NET_BUS_A) && route_relay_observed(&o, 2, 3, NET_BUS_B) &&
              !route_relay_observed(&o, 2, 1, NET_BUS_B));
}

/* 6.3: every node reports the same change; the log keeps one */
static void dedupe(void) {
    printf("\nraft 6.3: one entry per change\n");
    raft_log_init();
    boot_leader(0);
    uint8_t up2 = code(NODE_UP, 2, 0);
    check("NODE_UP N2 as a change: entry 1", raft_submit_change(up2) == 1);
    check("the same again, still uncommitted: dropped (0)", raft_submit_change(up2) == 0 && raft_log_last_index() == 1);
    uint8_t ev[8] = {1, 3, up2, 1};
    raft_on_frame(RAFT_ID(RAFT_ID_EVENT, 3), ev, 1);
    check("N3 reports it too (EVENT b3 = 1): dropped", raft_log_last_index() == 1);
    check("NODE_DOWN N2: a change, entry 2", raft_submit_change(code(NODE_DOWN, 2, 0)) == 2);
    check("a console submit is never dropped: entry 3", raft_submit(code(NODE_DOWN, 2, 0)) == 3);

    printf("\nnetwatch 6.4: our own link comes back (board, 2026-11-16)\n");
    raft_log_init();
    boot_leader(0);
    static const uint8_t setup[][3] = {{NODE_UP, 1, 0}, {NODE_UP, 2, 0}, {NODE_UP, 3, 0}, {CANLINK_UP, 1, 0},
                                       {CANLINK_UP, 1, 1}, {CANLINK_UP, 2, 0}, {CANLINK_UP, 2, 1},
                                       {CANLINK_UP, 3, 0}, {CANLINK_UP, 3, 1}, {COCKPIT_OK, 0, 0}};
    for (size_t i = 0; i < sizeof(setup) / sizeof(setup[0]); i++) {
        raft_log_append(1, code((enum net_event_type)setup[i][0], setup[i][1], setup[i][2]));
    }
    raft_log_commit(raft_log_last_index());
    uint16_t base = raft_log_last_index();
    netwatch_init(1, 0);
    struct netwatch_obs o;
    obs_all(&o); /* N1's A jumper out: nobody heard on A for a while (already logged) */
    o.direct[2][0] = o.direct[3][0] = o.fresh[2][0] = o.fresh[3][0] = false;
    netwatch_tick(&o, 1000);
    raft_log_init(); /* only what netwatch adds from here matters */
    for (size_t i = 0; i < sizeof(setup) / sizeof(setup[0]); i++) {
        raft_log_append(1, code((enum net_event_type)(setup[i][0]), setup[i][1], setup[i][2]));
    }
    raft_log_append(1, code(CANLINK_DOWN, 1, 0));
    raft_log_commit(raft_log_last_index());
    base = raft_log_last_index();
    obs_all(&o); /* back in: N3 heard on A at once, N2's next heartbeat still 15 ms away */
    o.direct[2][0] = o.fresh[2][0] = false;
    netwatch_tick(&o, 1010);
    bool blamed = false;
    for (uint16_t i = base + 1; i <= raft_log_last_index(); i++) {
        blamed = blamed || raft_log_event(i) == code(CANLINK_DOWN, 2, 0);
    }
    check("N3 heard first: N2's A not blamed (our A has been up 0 ms)", !blamed);
    check("our own A not reported up by us (N2 and N3 vouch for it)", raft_log_last_index() == base);
    o.direct[2][0] = true; /* N2 heard again within 20 ms; then N2 really gone from A */
    netwatch_tick(&o, 1030);
    o.direct[2][0] = o.fresh[2][0] = false;
    uint16_t before = raft_log_last_index();
    for (int64_t t = 1100; t < 2100; t += 10) {
        netwatch_tick(&o, t);
    }
    check("a real failure: nothing for the first second (the hold)", raft_log_last_index() == before);
    netwatch_tick(&o, 2110);
    check("... then reported once our A has settled and it held 1 s",
          raft_log_event(raft_log_last_index()) == code(CANLINK_DOWN, 2, 0));

    printf("\nnetwatch 6.5: the whole bus glitches for 600 ms (board, N3's B jumper)\n");
    obs_all(&o);
    before = raft_log_last_index();
    netwatch_tick(&o, 3000);
    raft_log_init(); /* a fresh leader log with everything up */
    for (size_t i = 0; i < sizeof(setup) / sizeof(setup[0]); i++) {
        raft_log_append(1, code((enum net_event_type)(setup[i][0]), setup[i][1], setup[i][2]));
    }
    raft_log_commit(raft_log_last_index());
    before = raft_log_last_index();
    obs_all(&o);
    for (int n = 1; n <= RAFT_N; n++) {
        o.direct[n][1] = o.fresh[n][1] = false; /* nobody heard on B */
    }
    for (int64_t t = 3010; t <= 3600; t += 10) {
        netwatch_tick(&o, t);
    }
    obs_all(&o);
    for (int64_t t = 3610; t <= 5000; t += 10) {
        netwatch_tick(&o, t);
    }
    check("bus B silent for every node for 600 ms, then back: nothing logged", raft_log_last_index() == before);
    raft_log_init();

    printf("\nraft 9.1: entries a follower catches up on are replayed\n");
    raft_log_init();
    raft_init(RAFT_NODE_STEERING, 0);
    rx_append(1, 1, 0, 0, 1, up2, 1, 10); /* entry 1, already committed by the leader */
    rx_append(1, 1, 1, 1, 1, code(NODE_UP, 3, 0), 1, 20); /* entry 2, live */
    check("entry 1 came committed (replayed), entry 2 live", raft_replayed_through() == 1);
    raft_log_init();
}

int main(void) {
    failures += netstate_selftest();
    event_codes();
    log_commit();
    log_full();
    append_from_log();
    leader_trace();
    leader_timing();
    follower();
    leader_steps_down();
    follower_append();
    submit();
    console();
    election_timeouts();
    candidate();
    voter();
    time_sync();
    link_watch();
    routing();
    dedupe();
    printf("\n%s: %d failure(s)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
