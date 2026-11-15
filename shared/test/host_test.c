/* Runs the shared/ code on the Mac (or the Pi), no board needed: make -C shared/test */
#include "netstate.h"
#include "raft.h"
#include "raft_config.h"
#include "raft_log.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>

static int64_t now;      /* simulated ms */
static bool print_tx;    /* print each frame sent, or only count them */
static int refs, appends;
static uint8_t last_append[8]; /* data of the last APPEND sent */
static int failures;

/* Stand-in for the CAN send hook: count each frame, and print it if asked */
void raft_platform_send(uint16_t id, uint8_t data[8]) {
    if (RAFT_ID_TYPE(id) == RAFT_ID_REF) {
        refs++;
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_APPEND) {
        appends++;
        memcpy(last_append, data, sizeof(last_append));
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

/* 2.3: each empty APPEND reads the leader's log as it is when sent */
static void append_from_log(void) {
    printf("\nraft 2.3: empty APPEND from the leader's log\n");
    raft_log_init();
    raft_init(RAFT_NODE_COCKPIT, 0);
    now = 0;
    raft_tick(now);
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
    raft_init(RAFT_NODE_COCKPIT, start);
    int i = 0;
    for (now = start; now <= start + 1000; now += steps[i++ % nsteps]) {
        raft_tick(now);
    }
}

/* 3.2: the frames themselves, to read by eye. REF at 0, 20, ... 200; APPEND at 0, 100, 200. */
static void leader_trace(void) {
    printf("\nraft 3.2: N1 as default leader, ticks 0..200 ms\n");
    print_tx = true;
    raft_init(RAFT_NODE_COCKPIT, 0);
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

/* 3.3: a leader that hears a higher term steps down and stops sending */
static void leader_steps_down(void) {
    printf("\nraft 3.3: N1 leader hears term 2\n");
    raft_init(RAFT_NODE_COCKPIT, 0);
    rx(RAFT_ID_REF, 3, 2, 5);
    check("REF from N3 in term 2: N1 becomes follower of N3",
          raft_get_role() == RAFT_FOLLOWER && raft_get_term() == 2 && raft_get_leader() == 3);
    refs = appends = 0;
    for (now = 5; now <= 200; now += RAFT_TICK_MS) {
        raft_tick(now);
    }
    check("then sends nothing", refs == 0 && appends == 0);
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
    printf("\n%s: %d failure(s)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
