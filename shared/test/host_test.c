/* Runs the shared/ code on the Mac (or the Pi), no board needed: make -C shared/test */
#include "netstate.h"
#include "raft.h"
#include "raft_config.h"
#include <stdio.h>

static int64_t now;      /* simulated ms */
static bool print_tx;    /* print each frame sent, or only count them */
static int refs, appends;
static int failures;

/* Stand-in for the CAN send hook: count each frame, and print it if asked */
void raft_platform_send(uint16_t id, uint8_t data[8]) {
    if (RAFT_ID_TYPE(id) == RAFT_ID_REF) {
        refs++;
    } else if (RAFT_ID_TYPE(id) == RAFT_ID_APPEND) {
        appends++;
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
    leader_trace();
    leader_timing();
    follower();
    leader_steps_down();
    printf("\n%s: %d failure(s)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
