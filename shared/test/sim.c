/*
 * Three nodes on two simulated CAN buses, in one process (make -C shared/test).
 * Each node is its own copy of the shared/ code (n1.so, n2.so, n3.so, opened with
 * dlopen), so each has its own statics, as on three boards. This file plays the
 * platform: it delivers every frame once per bus to every other powered node, ticks
 * each powered node every RAFT_TICK_MS, and powers nodes off and on.
 */
#include "netstate.h"
#include "raft.h"
#include "raft_config.h"
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* One node: its copy of the shared/ functions the sim needs */
struct node {
    bool up; /* powered: hears frames and ticks */
    void (*raft_init)(uint8_t, int64_t);
    void (*raft_tick)(int64_t);
    void (*raft_on_frame)(uint16_t, const uint8_t[8], int64_t);
    int (*raft_submit)(uint8_t);
    void (*raft_log_init)(void);
    uint16_t (*raft_log_last_index)(void);
    uint16_t (*raft_log_commit_index)(void);
    uint8_t (*raft_log_term)(uint16_t);
    uint8_t (*raft_log_event)(uint16_t);
    uint16_t (*raft_log_find_committed)(uint8_t, uint16_t);
    struct network (*getNetwork)(void);
    uint8_t (*net_event_encode)(const struct net_event *);
    void (*netstate_format)(const struct network *, char *, size_t);
};
static struct node nodes[RAFT_N + 1]; /* by node ID */

static void *symbol(void *lib, const char *name) {
    void *sym = dlsym(lib, name);
    if (sym == NULL) {
        fprintf(stderr, "sim: %s\n", dlerror());
        exit(2);
    }
    return sym;
}

#define LOAD(node, lib, fn) ((node)->fn = (__typeof__((node)->fn))symbol(lib, #fn))

static void load(uint8_t id) {
    char path[16];
    snprintf(path, sizeof(path), "./n%u.so", id);
    void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (lib == NULL) {
        fprintf(stderr, "sim: %s\n", dlerror());
        exit(2);
    }
    struct node *n = &nodes[id];
    LOAD(n, lib, raft_init);
    LOAD(n, lib, raft_tick);
    LOAD(n, lib, raft_on_frame);
    LOAD(n, lib, raft_submit);
    LOAD(n, lib, raft_log_init);
    LOAD(n, lib, raft_log_last_index);
    LOAD(n, lib, raft_log_commit_index);
    LOAD(n, lib, raft_log_term);
    LOAD(n, lib, raft_log_event);
    LOAD(n, lib, raft_log_find_committed);
    LOAD(n, lib, getNetwork);
    LOAD(n, lib, net_event_encode);
    LOAD(n, lib, netstate_format);
}

/* ---- The buses ---- */

struct frame {
    uint16_t id;
    uint8_t data[8];
    int64_t at; /* delivery time */
};
#define MAX_FRAMES 4096
static struct frame queue[MAX_FRAMES];
static int queued;
static int64_t now; /* simulated ms */

/* The platform hook every node calls: the frame goes out on bus A, and on bus B 1 ms later */
void raft_platform_send(uint16_t id, uint8_t data[8]) {
    for (int bus = 0; bus < 2; bus++) {
        if (queued == MAX_FRAMES) {
            fprintf(stderr, "sim: bus queue full\n");
            exit(2);
        }
        struct frame *f = &queue[queued++];
        f->id = id;
        memcpy(f->data, data, 8);
        f->at = now + 1 + bus;
    }
}

/* Every frame due by now reaches every other powered node. Frames sent from inside
 * raft_on_frame (replies) are queued for later. */
static void deliver(void) {
    int pending = queued;
    int kept = 0;
    for (int i = 0; i < pending; i++) {
        struct frame f = queue[i];
        if (f.at > now) {
            queue[kept++] = f;
            continue;
        }
        for (uint8_t id = 1; id <= RAFT_N; id++) {
            if (id != RAFT_ID_SENDER(f.id) && nodes[id].up) {
                nodes[id].raft_on_frame(f.id, f.data, now);
            }
        }
    }
    memmove(&queue[kept], &queue[pending], (size_t)(queued - pending) * sizeof(queue[0]));
    queued = kept + (queued - pending);
}

/* Advance the clock ms by ms: deliver frames, and tick every powered node every RAFT_TICK_MS */
static void run(int ms) {
    for (int64_t end = now + ms; now < end; now++) {
        deliver();
        if (now % RAFT_TICK_MS == 0) {
            for (uint8_t id = 1; id <= RAFT_N; id++) {
                if (nodes[id].up) {
                    nodes[id].raft_tick(now);
                }
            }
        }
    }
}

/* Power on with RAM cleared: an empty log and graph, then boot */
static void boot(uint8_t id) {
    nodes[id].raft_log_init();
    nodes[id].raft_init(id, now);
    nodes[id].up = true;
}

/* ---- Checks ---- */

static int failures;

static void check(const char *what, bool ok) {
    printf("  %-66s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) {
        failures++;
    }
}

static uint8_t code(enum net_event_type type, uint8_t node, uint8_t bus) {
    struct net_event ev = {type, node, bus};
    return nodes[1].net_event_encode(&ev);
}

/* Node id holds n entries, all committed, the same as N1's, and the same graph */
static bool agrees(uint8_t id, uint16_t n) {
    struct node *a = &nodes[1];
    struct node *b = &nodes[id];
    if (b->raft_log_last_index() != n || b->raft_log_commit_index() != n || a->raft_log_last_index() < n) {
        return false;
    }
    for (uint16_t i = 1; i <= n; i++) {
        if (a->raft_log_term(i) != b->raft_log_term(i) || a->raft_log_event(i) != b->raft_log_event(i)) {
            return false;
        }
    }
    struct network na = a->getNetwork();
    struct network nb = b->getNetwork();
    return memcmp(&na, &nb, sizeof(na)) == 0;
}

static bool all_agree(uint16_t n) {
    return agrees(1, n) && agrees(2, n) && agrees(3, n);
}

/* `log` and `net show` for every node, as the demo compares them */
static void show(void) {
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        struct node *n = &nodes[id];
        struct network net = n->getNetwork();
        char graph[96];
        n->netstate_format(&net, graph, sizeof(graph));
        printf("  N%u %-4s log %u, commit %u | %s\n", id, n->up ? "up" : "DOWN", n->raft_log_last_index(),
               n->raft_log_commit_index(), graph);
    }
}

int main(void) {
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        load(id);
        boot(id);
    }
    run(10);

    printf("\nsim 4.3: an event submitted on the leader\n");
    check("N1 appends NODE_UP N1 as entry 1", nodes[1].raft_submit(code(NODE_UP, 1, 0)) == 1);
    run(20);
    check("20 ms later: entry 1 committed on all three, graphs equal", all_agree(1));

    printf("\nsim 4.4: events submitted on followers go to the leader\n");
    uint8_t n2_up = code(NODE_UP, 2, 0);
    uint16_t n2_committed = nodes[2].raft_log_commit_index(); /* what N2's console notes first */
    check("N2 sends NODE_UP N2 to the leader (returns 0)", nodes[2].raft_submit(n2_up) == 0);
    check("N3 sends NODE_UP N3 in the same ms", nodes[3].raft_submit(code(NODE_UP, 3, 0)) == 0);
    run(30);
    check("30 ms later: entries 2-3 everywhere, one entry per event, not per bus copy", all_agree(3));
    uint16_t found = nodes[2].raft_log_find_committed(n2_up, n2_committed);
    check("N2's console finds its event committed, at that index on every node",
          found != 0 && nodes[1].raft_log_find_committed(n2_up, n2_committed) == found &&
              nodes[3].raft_log_find_committed(n2_up, n2_committed) == found);

    printf("\nsim 4.5: N3 powered off: N1 and N2 are still a majority\n");
    nodes[3].up = false;
    nodes[2].raft_submit(code(CANLINK_UP, 2, NET_BUS_A));
    nodes[2].raft_submit(code(CANLINK_UP, 2, NET_BUS_B));
    run(30);
    check("entries 4-5 committed on N1 and N2", agrees(1, 5) && agrees(2, 5));
    check("N3 still has 3", nodes[3].raft_log_last_index() == 3);

    printf("\nsim 4.6: N3 back with its log (a pulled connector, not a reboot)\n");
    nodes[3].up = true;
    run(150);
    check("within 150 ms it has entries 4-5 too", all_agree(5));

    printf("\nsim 4.7: N3 reboots: its log is gone (RAM)\n");
    boot(3);
    check("right after boot N3's log is empty", nodes[3].raft_log_last_index() == 0);
    run(250);
    check("within 250 ms it has all 5 again, and the same graph", all_agree(5));

    printf("\nsim 4.8: N2 and N3 off: no majority, so nothing commits\n");
    nodes[2].up = false;
    nodes[3].up = false;
    nodes[1].raft_submit(code(COCKPIT_OK, 0, 0));
    run(300);
    check("N1 has entry 6 but can't commit it",
          nodes[1].raft_log_last_index() == 6 && nodes[1].raft_log_commit_index() == 5);
    nodes[2].up = true;
    run(150);
    check("N2 back: entry 6 commits on N1 and N2", agrees(1, 6) && agrees(2, 6));
    nodes[3].up = true;
    run(150);
    check("N3 back: all three agree on 6 entries", all_agree(6));

    printf("\nsim: final state\n");
    show();
    printf("\nsim %s: %d failure(s)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
