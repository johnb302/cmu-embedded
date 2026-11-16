/*
 * Three nodes on two simulated CAN buses, in one process (make -C shared/test).
 * Each node is its own copy of the shared/ code (n1.so, n2.so, n3.so, opened with
 * dlopen), so each has its own statics, as on three boards. This file plays the
 * platform: it delivers every frame once per bus to every other powered node whose
 * jumper on that bus is in, ticks each powered node every RAFT_TICK_MS, and powers
 * nodes off and on.
 *
 *   ./sim                default_leader=cockpit: steps 4 and 5
 *   ./sim random         LEADER=random (r1.so ...: built with RAFT_DEFAULT_LEADER = none)
 *   ./sim random-seed9   LEADER=random with seed 9 (p1.so ...): the seed that makes the Pi win
 *   ./sim watch          steps 6-9: every node also runs link watching and relays frames
 *                        by its routing table, so the log follows the jumpers
 */
#include "netstate.h"
#include "netwatch.h"
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
    enum raft_role (*raft_get_role)(void);
    uint8_t (*raft_get_term)(void);
    uint8_t (*raft_get_leader)(void);
    void (*raft_log_init)(void);
    uint16_t (*raft_log_last_index)(void);
    uint16_t (*raft_log_commit_index)(void);
    uint8_t (*raft_log_term)(uint16_t);
    uint8_t (*raft_log_event)(uint16_t);
    uint16_t (*raft_log_find_committed)(uint8_t, uint16_t);
    struct network (*getNetwork)(void);
    uint8_t (*net_event_encode)(const struct net_event *);
    void (*netstate_format)(const struct network *, char *, size_t);
    int (*net_event_decode)(uint8_t, struct net_event *);
    void (*net_event_format)(const struct net_event *, char *, size_t);
    bool (*raft_in_sync)(int64_t);
    uint16_t (*raft_replayed_through)(void);
    int (*raft_held_for)(uint8_t);
    void (*netwatch_init)(uint8_t, int64_t);
    int (*netwatch_tick)(const struct netwatch_obs *, int64_t);
    void (*route_build)(const struct network *, uint8_t, struct route_table *);
    bool (*route_relay_observed)(const struct netwatch_obs *, uint8_t, uint8_t, uint8_t);
    struct route_table routes; /* rebuilt from its getNetwork() every 10 ms */
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

/* prefix "n" loads ./n1.so for node 1, and so on */
static void load(const char *prefix, uint8_t id) {
    char path[16];
    snprintf(path, sizeof(path), "./%s%u.so", prefix, id);
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
    LOAD(n, lib, raft_get_role);
    LOAD(n, lib, raft_get_term);
    LOAD(n, lib, raft_get_leader);
    LOAD(n, lib, raft_log_init);
    LOAD(n, lib, raft_log_last_index);
    LOAD(n, lib, raft_log_commit_index);
    LOAD(n, lib, raft_log_term);
    LOAD(n, lib, raft_log_event);
    LOAD(n, lib, raft_log_find_committed);
    LOAD(n, lib, getNetwork);
    LOAD(n, lib, net_event_encode);
    LOAD(n, lib, netstate_format);
    LOAD(n, lib, net_event_decode);
    LOAD(n, lib, net_event_format);
    LOAD(n, lib, raft_in_sync);
    LOAD(n, lib, raft_replayed_through);
    LOAD(n, lib, raft_held_for);
    LOAD(n, lib, netwatch_init);
    LOAD(n, lib, netwatch_tick);
    LOAD(n, lib, route_build);
    LOAD(n, lib, route_relay_observed);
}

/* ---- The buses ---- */

struct frame {
    uint16_t id;
    uint8_t data[8];
    uint8_t bus;     /* NET_BUS_A or NET_BUS_B */
    uint8_t relayer; /* the node that forwarded it, or 0 for the sender's own copy */
    int64_t at;      /* delivery time */
};
#define MAX_FRAMES 4096
static struct frame queue[MAX_FRAMES];
static int queued;
static int64_t now; /* simulated ms */

/* linked[node][bus]: that node's jumper on that bus is in. A pulled one cuts the node
 * off that bus both ways. */
static bool linked[RAFT_N + 1][2];

/* ./sim watch: nodes run link watching (netwatch_tick) and relay by their routing tables */
static bool watching;

/* What node `me` hears: other nodes' own frames on a bus they share, and relayed ones */
static bool hears_direct(uint8_t me, uint8_t n, int b) {
    return n != me && nodes[n].up && nodes[me].up && linked[n][b] && linked[me][b];
}

static void observe_direct(uint8_t me, struct netwatch_obs *o) {
    *o = (struct netwatch_obs){0};
    for (uint8_t n = 1; n <= RAFT_N; n++) {
        for (int b = 0; b < 2; b++) {
            o->direct[n][b] = hears_direct(me, n, b);
            o->fresh[n][b] = o->direct[n][b]; /* links change instantly here */
        }
    }
}

/* Does relay r forward sender s's frames heard on bus `from`? Its routing table from the
 * graph, or the same rule on what it hears (netwatch.h) */
static bool relays(uint8_t r, uint8_t s, int from) {
    struct netwatch_obs o;
    observe_direct(r, &o);
    return nodes[r].routes.relay[s][from] || nodes[r].route_relay_observed(&o, r, s, (uint8_t)from);
}

static void observe(uint8_t me, struct netwatch_obs *o) {
    observe_direct(me, o);
    for (uint8_t n = 1; n <= RAFT_N; n++) {
        o->alive[n] = o->direct[n][0] || o->direct[n][1];
        for (uint8_t r = 1; r <= RAFT_N && !o->alive[n] && watching; r++) {
            for (int from = 0; from < 2; from++) {
                if (r != me && r != n && hears_direct(r, n, from) && hears_direct(me, r, 1 - from) &&
                    relays(r, n, from)) {
                    o->alive[n] = true; /* n's heartbeats reach us through r */
                }
            }
        }
    }
}

/* The platform hook every node calls: the frame goes out on bus A, and on bus B 1 ms later */
void raft_platform_send(uint16_t id, uint8_t data[8]) {
    for (uint8_t bus = NET_BUS_A; bus <= NET_BUS_B; bus++) {
        if (queued == MAX_FRAMES) {
            fprintf(stderr, "sim: bus queue full\n");
            exit(2);
        }
        struct frame *f = &queue[queued++];
        f->id = id;
        memcpy(f->data, data, 8);
        f->bus = bus;
        f->relayer = 0;
        f->at = now + 1 + bus;
    }
}

static void queue_relayed(const struct frame *in, uint8_t relayer) {
    if (queued == MAX_FRAMES) {
        fprintf(stderr, "sim: bus queue full\n");
        exit(2);
    }
    struct frame *f = &queue[queued++];
    *f = *in;
    f->data[0] |= RAFT_FLAG_RELAYED;
    f->bus = 1 - in->bus;
    f->relayer = relayer;
    f->at = now + 1;
}

/* Every frame due by now reaches every other powered node on its bus. Frames sent from
 * inside raft_on_frame (replies) are queued for later. */
static void deliver(void) {
    int pending = queued;
    int kept = 0;
    for (int i = 0; i < pending; i++) {
        struct frame f = queue[i];
        if (f.at > now) {
            queue[kept++] = f;
            continue;
        }
        uint8_t from = RAFT_ID_SENDER(f.id);
        uint8_t on_wire = f.relayer ? f.relayer : from; /* who put this copy on the bus */
        for (uint8_t id = 1; id <= RAFT_N; id++) {
            if (id == from || id == f.relayer || !nodes[id].up || !linked[on_wire][f.bus] || !linked[id][f.bus]) {
                continue;
            }
            nodes[id].raft_on_frame(f.id, f.data, now);
            if (watching && f.relayer == 0 && relays(id, from, f.bus)) {
                queue_relayed(&f, id);
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
        if (watching && now % 10 == 0) { /* HB_MON's period on the boards */
            for (uint8_t id = 1; id <= RAFT_N; id++) {
                if (nodes[id].up) {
                    struct network g = nodes[id].getNetwork();
                    struct netwatch_obs o;
                    nodes[id].route_build(&g, id, &nodes[id].routes);
                    observe(id, &o);
                    nodes[id].netwatch_tick(&o, now);
                }
            }
        }
    }
}

/* Power on with RAM cleared: an empty log and graph, then boot */
static void boot(uint8_t id) {
    nodes[id].raft_log_init();
    nodes[id].raft_init(id, now);
    nodes[id].netwatch_init(id, now);
    nodes[id].routes = (struct route_table){0};
    nodes[id].up = true;
}

static void set_links(uint8_t id, bool a, bool b) {
    linked[id][NET_BUS_A] = a;
    linked[id][NET_BUS_B] = b;
}

/* ---- Checks ---- */

static int failures;

static void check(const char *what, bool ok) {
    printf("  %-70s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) {
        failures++;
    }
}

static uint8_t code(enum net_event_type type, uint8_t node, uint8_t bus) {
    struct net_event ev = {type, node, bus};
    return nodes[1].net_event_encode(&ev);
}

#define BIT(id) (1U << (id))
#define ALL     (BIT(1) | BIT(2) | BIT(3))

/* Node id holds n entries, all committed, the same as node ref's, and the same graph */
static bool same_as(uint8_t ref, uint8_t id, uint16_t n) {
    struct node *a = &nodes[ref];
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

/* Every node in mask holds the same n committed entries and the same graph */
static bool logs_agree(unsigned mask, uint16_t n) {
    uint8_t ref = 0;
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        if (mask & BIT(id)) {
            if (ref == 0) {
                ref = id;
            }
            if (!same_as(ref, id, n)) {
                return false;
            }
        }
    }
    return true;
}

static bool agrees(uint8_t id, uint16_t n) {
    return same_as(1, id, n);
}

static bool all_agree(uint16_t n) {
    return logs_agree(ALL, n);
}

/* The leader the nodes in mask agree on: exactly one of them leads, and all of them are
 * in its term and name it. 0 if they don't agree (yet). */
static uint8_t elected(unsigned mask) {
    uint8_t leader = 0;
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        if ((mask & BIT(id)) && nodes[id].raft_get_role() == RAFT_LEADER) {
            if (leader != 0) {
                return 0;
            }
            leader = id;
        }
    }
    if (leader == 0) {
        return 0;
    }
    uint8_t term = nodes[leader].raft_get_term();
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        if ((mask & BIT(id)) && (nodes[id].raft_get_term() != term || nodes[id].raft_get_leader() != leader)) {
            return 0;
        }
    }
    return leader;
}

/* Run ms by ms until the nodes in mask agree on a leader. The ms it took, or -1 after max_ms. */
static int run_until_elected(unsigned mask, int max_ms) {
    for (int ms = 0; ms <= max_ms; ms++) {
        if (elected(mask) != 0) {
            return ms;
        }
        run(1);
    }
    return -1;
}

/* Every node in mask is in term */
static bool in_term(unsigned mask, uint8_t term) {
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        if ((mask & BIT(id)) && nodes[id].raft_get_term() != term) {
            return false;
        }
    }
    return true;
}

/* The other STM32 */
static uint8_t other_stm32(uint8_t id) {
    return id == RAFT_NODE_STEERING ? RAFT_NODE_DRIVETRAIN : RAFT_NODE_STEERING;
}

/* `log` and `net show` for every node, as the demo compares them */
static void show(void) {
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        struct node *n = &nodes[id];
        struct network net = n->getNetwork();
        char graph[96];
        n->netstate_format(&net, graph, sizeof(graph));
        printf("  N%u %-4s %-9s term %u, log %u, commit %u | %s\n", id, n->up ? "up" : "DOWN",
               n->raft_get_role() == RAFT_LEADER ? "leader" : "", n->raft_get_term(), n->raft_log_last_index(),
               n->raft_log_commit_index(), graph);
    }
}

/* ---- default_leader = cockpit ---- */

static void step4(void) {
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
    check("N3 back: all three agree on 6 entries, still term 1", all_agree(6) && in_term(ALL, 1));
}

/* Link faults and failures, from step 4's state: N1 leads term 1, 6 entries */
static void step5(void) {
    uint16_t entries = 6;

    printf("\nsim 5.1: N3's A jumper out for 1 s (checkoff step 4: no election)\n");
    set_links(3, false, true);
    run(1000);
    check("term 1 on all three, all following N1", in_term(ALL, 1) && elected(ALL) == 1);
    set_links(3, true, true);

    printf("\nsim 5.2: N1 off B and N3 off A for 1 s: N3 can't hear the leader\n");
    set_links(1, true, false);
    set_links(3, false, true);
    nodes[2].raft_submit(code(CANLINK_DOWN, 1, NET_BUS_B));
    entries++;
    run(1000);
    check("N2 turned down N3's pre-votes: still term 1, N1 leads N2",
          in_term(ALL, 1) && elected(BIT(1) | BIT(2)) == 1);
    check("N2's event committed on N1 and N2 (N3 needs the relay, step 6)",
          logs_agree(BIT(1) | BIT(2), entries) && nodes[3].raft_log_last_index() == entries - 1);
    set_links(1, true, true);
    set_links(3, true, true);
    run(300);
    check("jumpers back: N3 follows N1 again, all agree, term 1", elected(ALL) == 1 && all_agree(entries));

    printf("\nsim 5.3: N2 isolated for 1 s, then back (checkoff step 10, not the leader)\n");
    set_links(2, false, false);
    run(1000);
    check("term 1 everywhere; N1 still leads N3", in_term(ALL, 1) && elected(BIT(1) | BIT(3)) == 1);
    set_links(2, true, true);
    run(300);
    check("N2 back: follows N1, term 1", elected(ALL) == 1 && in_term(ALL, 1) && all_agree(entries));

    printf("\nsim 5.4: the Pi powered off (checkoff step 6)\n");
    nodes[1].up = false;
    int ms = run_until_elected(BIT(2) | BIT(3), 1000);
    uint8_t leader = elected(BIT(2) | BIT(3));
    printf("  N%u leads term %u after %d ms\n", leader, nodes[leader].raft_get_term(), ms);
    check("an STM32 leads within 1 s, in term 2 (the term moved once)",
          ms >= 0 && ms < 1000 && in_term(BIT(2) | BIT(3), 2));
    nodes[other_stm32(leader)].raft_submit(code(NODE_DOWN, 1, 0));
    entries++;
    run(50);
    check("an event from the follower commits on both STM32s", logs_agree(BIT(2) | BIT(3), entries));

    printf("\nsim 5.5: the Pi powers back on (checkoff step 7): empty log, listens before leading\n");
    boot(1);
    run(400);
    check("it hears the cluster and follows the STM32; the STM32 still leads",
          elected(ALL) == leader && in_term(ALL, 2));
    check("the leader replayed the whole log to it: all agree", all_agree(entries));

    printf("\nsim 5.6: the STM32 leader isolated (checkoff step 10, the leader)\n");
    uint8_t old_leader = leader;
    uint8_t rest = BIT(1) | BIT(other_stm32(old_leader));
    set_links(old_leader, false, false);
    ms = run_until_elected(rest, 1000);
    leader = elected(rest);
    printf("  N%u leads term %u after %d ms\n", leader, leader ? nodes[leader].raft_get_term() : 0, ms);
    check("the other two elect within 1 s, term 3", ms >= 0 && ms < 1000 && in_term(rest, 3));
    check("the isolated node still thinks it leads term 2",
          nodes[old_leader].raft_get_role() == RAFT_LEADER && nodes[old_leader].raft_get_term() == 2);
    nodes[leader].raft_submit(code(NODE_UP, 1, 0));
    entries++;
    run(50);
    check("an event commits on the two", logs_agree(rest, entries));
    set_links(old_leader, true, true);
    run(300);
    check("reconnected: it steps down and follows; term 3 everywhere (moved once)",
          elected(ALL) == leader && in_term(ALL, 3));
    check("and it caught up: all agree", all_agree(entries));

    printf("\nsim 5.7: only one node left: no leader, and the term stays put\n");
    uint8_t lone = leader == 1 ? RAFT_NODE_STEERING : 1;
    uint8_t back = (uint8_t)(6 - leader - lone); /* the third node */
    nodes[leader].up = false;
    nodes[back].up = false;
    run(1000);
    check("1 s alone: not leader, still term 3 (pre-votes unanswered)",
          nodes[lone].raft_get_role() != RAFT_LEADER && nodes[lone].raft_get_term() == 3);
    nodes[back].up = true;
    ms = run_until_elected(BIT(lone) | BIT(back), 1000);
    uint8_t term = nodes[lone].raft_get_term();
    printf("  N%u back: N%u leads term %u after %d ms\n", back, elected(BIT(lone) | BIT(back)), term, ms);
    check("a second node back: a leader within 1 s, in a new term",
          ms >= 0 && ms < 1000 && term > 3 && in_term(BIT(lone) | BIT(back), term));
    nodes[leader].up = true; /* the old leader, frozen since it went off */
    run(300);
    check("the third back: it follows, one term everywhere, all agree",
          elected(ALL) != 0 && in_term(ALL, term) && all_agree(entries));
}


/* ---- steps 6-9: link watching, routing, the Pi's rejoin (./sim watch) ---- */

/* The committed entries every node shares, and the term everywhere unchanged */
static uint16_t entries_everywhere(unsigned mask) {
    uint16_t n = 0;
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        if (mask & BIT(id)) {
            n = nodes[id].raft_log_commit_index();
            break;
        }
    }
    return logs_agree(mask, n) ? n : 0;
}

static bool graph_is(uint8_t id, const char *want) {
    struct network g = nodes[id].getNetwork();
    char got[96];
    nodes[id].netstate_format(&g, got, sizeof(got));
    return strcmp(got, want) == 0;
}

static void print_new(uint8_t id, uint16_t from, uint16_t to) {
    for (uint16_t i = from + 1; i <= to; i++) {
        struct net_event ev;
        char name[24] = "?";
        if (nodes[id].net_event_decode(nodes[id].raft_log_event(i), &ev) == 0) {
            nodes[id].net_event_format(&ev, name, sizeof(name));
        }
        printf("  entry %u, term %u: %s\n", i, nodes[id].raft_log_term(i), name);
    }
}

static void watch_scenarios(void) {
    const char *all_up = "N1 up A+ B+ | N2 up A+ B+ | N3 up A+ B+ | cockpit OK";

    printf("\nsim 6.1: cold start: the nodes report themselves and each other\n");
    run(1000);
    uint16_t n = entries_everywhere(ALL);
    print_new(1, 0, n);
    check("1 s: one entry per node, link and the cockpit (10), the same on all three",
          n == 10 && graph_is(1, all_up) && graph_is(2, all_up) && graph_is(3, all_up));
    check("term 1, N1 leads", in_term(ALL, 1) && elected(ALL) == 1);

    printf("\nsim 6.2: N3's A jumper out, then back (checkoff step 4)\n");
    set_links(3, false, true);
    run(1300); /* down is reported after it has held 1 s (NETWATCH_DOWN_HOLD_MS) */
    uint16_t m = entries_everywhere(ALL);
    print_new(1, n, m);
    check("one new entry, CANLINK_DOWN N3 A, at the same index everywhere; term 1",
          m == n + 1 && nodes[2].raft_log_event(m) == code(CANLINK_DOWN, 3, NET_BUS_A) && in_term(ALL, 1));
    set_links(3, true, true);
    run(300);
    n = entries_everywhere(ALL);
    print_new(1, m, n);
    check("back: one entry, CANLINK_UP N3 A", n == m + 1 && graph_is(3, all_up));

    printf("\nsim 6.3: N1 off B, then N3 off A: the hop through N2 (checkoff step 5)\n");
    set_links(1, true, false);
    run(1300);
    set_links(3, false, true);
    run(1500);
    m = entries_everywhere(ALL);
    print_new(1, n, m);
    const char *hop = "N1 up A+ B- | N2 up A+ B+ | N3 up A- B+ | cockpit OK";
    check("two entries, the same everywhere (N3's by relay); term 1",
          m == n + 2 && graph_is(3, hop) && graph_is(1, hop) && in_term(ALL, 1));
    check("N2's table relays N1 A->B and N3 B->A", nodes[2].routes.relay[1][NET_BUS_A] && nodes[2].routes.relay[3][NET_BUS_B]);
    check("N3 still hears REF (relayed): SYNC on", nodes[3].raft_in_sync(now));
    nodes[2].raft_submit(code(COCKPIT_OK, 0, 0));
    run(100);
    check("an EVENT from N2 reaches N3's log through the leader N1 and the relay N2",
          entries_everywhere(ALL) == m + 1);
    set_links(1, true, true);
    set_links(3, true, true);
    run(500);
    n = entries_everywhere(ALL);
    check("jumpers back: links up everywhere, nobody relays",
          n == m + 3 && graph_is(2, all_up) && !nodes[2].routes.relay[1][NET_BUS_A]);

    printf("\nsim 6.4: the Pi powered off (checkoff step 6)\n");
    nodes[1].up = false;
    int ms = run_until_elected(BIT(2) | BIT(3), 1000);
    uint8_t leader = elected(BIT(2) | BIT(3));
    run(1500);
    m = entries_everywhere(BIT(2) | BIT(3));
    print_new(2, n, m);
    printf("  N%u leads term %u after %d ms; holding %d entries for N1\n", leader, nodes[leader].raft_get_term(), ms,
           nodes[leader].raft_held_for(1));
    check("an STM32 leads within 1 s, and logs NODE_DOWN N1 and COCKPIT_ERROR once each",
          ms >= 0 && ms < 1000 && m == n + 2 && graph_is(2, "N1 down | N2 up A+ B+ | N3 up A+ B+ | cockpit ERROR"));
    check("the leader holds those two entries for N1", nodes[leader].raft_held_for(1) == 2);

    printf("\nsim 6.5: the Pi powers back on (checkoff step 7)\n");
    boot(1);
    run(600);
    n = entries_everywhere(ALL);
    print_new(1, m, n);
    uint16_t replayed = nodes[1].raft_replayed_through();
    printf("  N1 replayed entries 1-%u\n", replayed);
    check("it rejoins as a follower; the STM32 still leads; term unchanged",
          elected(ALL) == leader && in_term(ALL, nodes[leader].raft_get_term()));
    nodes[3].raft_submit(code(COCKPIT_OK, 0, 0)); /* a live event once it has caught up */
    run(100);
    n = entries_everywhere(ALL);
    check("everything logged while it was away came replayed, then the next one live",
          replayed >= m && n == nodes[1].raft_log_commit_index() && nodes[1].raft_replayed_through() == replayed &&
              n > replayed);
    check("NODE_UP N1, links and COCKPIT_OK logged: all up again", graph_is(1, all_up) && graph_is(leader, all_up));
    check("the leader holds nothing for N1 any more", nodes[leader].raft_held_for(1) == 0);

    printf("\nsim 6.6: the STM32 leader cut off from the other STM32: the Pi bridges (checkoff step 9)\n");
    uint8_t other = other_stm32(leader);
    uint8_t term = nodes[leader].raft_get_term();
    set_links(leader, true, false); /* leader on A only */
    set_links(other, false, true);  /* the other on B only */
    run(1600);
    m = entries_everywhere(ALL);
    print_new(1, n, m);
    check("two entries everywhere, term unchanged, SYNC on at the cut-off STM32",
          m == n + 2 && in_term(ALL, term) && elected(ALL) == leader && nodes[other].raft_in_sync(now));
    check("the Pi's table relays both STM32s", nodes[1].routes.relay[leader][NET_BUS_A] && nodes[1].routes.relay[other][NET_BUS_B]);
    set_links(leader, true, true);
    set_links(other, true, true);
    run(500);
    n = entries_everywhere(ALL);
    check("restored: all up, term unchanged", graph_is(1, all_up) && in_term(ALL, term));

    printf("\nsim 6.7: a follower isolated, then back (checkoff step 10)\n");
    set_links(other, false, false);
    run(1500);
    m = entries_everywhere(BIT(1) | BIT(leader));
    print_new(1, n, m);
    check("NODE_DOWN once; term unchanged", m == n + 1 && in_term(BIT(1) | BIT(leader), term));
    set_links(other, true, true);
    run(500);
    n = entries_everywhere(ALL);
    print_new(1, m, n);
    check("back: NODE_UP and its two links; all agree; term unchanged", n == m + 3 && graph_is(other, all_up) && in_term(ALL, term));
}

/* ---- LEADER=random (checkoff step 8) ---- */

static void random_leader(bool pi_seed) {
    printf("\nsim R.1: all three boot at once with no preset leader\n");
    int ms = run_until_elected(ALL, 1000);
    uint8_t leader = elected(ALL);
    printf("  N%u leads term %u after %d ms\n", leader, leader ? nodes[leader].raft_get_term() : 0, ms);
    check("one leader within 1 s, everyone in its term", ms >= 0 && ms < 1000);
    nodes[leader].raft_submit(code(NODE_UP, 1, 0));
    run(50);
    check("an event commits everywhere", all_agree(1));

    printf("\nsim R.2: reboot everything; the Pi's cockpit starts 30 s after the STM32s\n");
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        nodes[id].up = false;
    }
    run(100);
    boot(RAFT_NODE_STEERING);
    boot(RAFT_NODE_DRIVETRAIN);
    ms = run_until_elected(BIT(2) | BIT(3), 1000);
    leader = elected(BIT(2) | BIT(3));
    printf("  N%u leads term %u after %d ms\n", leader, leader ? nodes[leader].raft_get_term() : 0, ms);
    check("an STM32 leads within 1 s", ms >= 0 && ms < 1000);
    run(30000 - ms);
    boot(1);
    run(200);
    check("the Pi starts: it follows the STM32", elected(ALL) == leader);
    nodes[1].raft_submit(code(NODE_UP, 2, 0));
    nodes[1].raft_submit(code(NODE_UP, 3, 0));
    run(100);
    check("events from the Pi commit everywhere", all_agree(2));

    printf("\nsim R.3: power off that STM32: the Pi and the other STM32 elect\n");
    uint8_t first = leader;
    uint8_t term = nodes[first].raft_get_term();
    unsigned rest = BIT(1) | BIT(other_stm32(first));
    nodes[first].up = false;
    ms = run_until_elected(rest, 1000);
    leader = elected(rest);
    printf("  N%u leads term %u after %d ms\n", leader, leader ? nodes[leader].raft_get_term() : 0, ms);
    check("a new leader within 1 s, term + 1", ms >= 0 && ms < 1000 && in_term(rest, (uint8_t)(term + 1)));
    if (pi_seed) {
        check("seed 9: the Pi wins", leader == 1);
    }
    check("logs agree", logs_agree(rest, 2));

    printf("\nsim R.4: the STM32 powers back on, then the Pi goes off (checkoff step 6 again)\n");
    boot(first);
    run(300);
    check("the STM32 rejoins as a follower and catches up", elected(ALL) == leader && all_agree(2));
    if (leader == 1) {
        nodes[1].up = false;
        ms = run_until_elected(BIT(2) | BIT(3), 1000);
        leader = elected(BIT(2) | BIT(3));
        printf("  N%u leads term %u after %d ms\n", leader, leader ? nodes[leader].raft_get_term() : 0, ms);
        check("Pi off: an STM32 leads within 1 s; logs agree",
              ms >= 0 && ms < 1000 && logs_agree(BIT(2) | BIT(3), 2));
    }
}

int main(int argc, char **argv) {
    const char *mode = argc > 1 ? argv[1] : "default";
    bool random_mode = strcmp(mode, "random") == 0;
    bool pi_seed = strcmp(mode, "random-seed9") == 0;
    watching = strcmp(mode, "watch") == 0;
    const char *prefix = random_mode ? "r" : (pi_seed ? "p" : "n");

    printf("\nsim (%s): %s1.so %s2.so %s3.so\n", mode, prefix, prefix, prefix);
    for (uint8_t id = 1; id <= RAFT_N; id++) {
        load(prefix, id);
        set_links(id, true, true);
        boot(id);
    }
    if (random_mode || pi_seed) {
        random_leader(pi_seed);
    } else if (watching) {
        watch_scenarios();
    } else {
        run(RAFT_CLAIM_LISTEN_MS + 10); /* N1 listens, hears no cluster, leads term 1 */
        step4();
        step5();
    }

    printf("\nsim (%s): final state\n", mode);
    show();
    printf("\nsim (%s) %s: %d failure(s)\n", mode, failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
