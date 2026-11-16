#include "netstate.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>

/* Boots with nothing known: every node and link down, cockpit in error */
static struct network graph;

static const char *const event_names[] = {
    [COCKPIT_OK] = "COCKPIT_OK",
    [COCKPIT_ERROR] = "COCKPIT_ERROR",
    [NODE_UP] = "NODE_UP",
    [NODE_DOWN] = "NODE_DOWN",
    [CANLINK_UP] = "CANLINK_UP",
    [CANLINK_DOWN] = "CANLINK_DOWN",
};

/* Which fields each event type uses */
static bool uses_bus(enum net_event_type type) {
    return type == CANLINK_UP || type == CANLINK_DOWN;
}

static bool uses_node(enum net_event_type type) {
    return type == NODE_UP || type == NODE_DOWN || uses_bus(type);
}

/* A known type, with a node and bus in range wherever the type uses them.
 * netstate_apply() and net_event_encode() share this, so every code that decodes applies. */
static bool event_valid(const struct net_event *ev) {
    if ((unsigned)ev->type > CANLINK_DOWN) {
        return false;
    }
    if (uses_node(ev->type) && (ev->node < 1 || ev->node > NET_NODES)) {
        return false;
    }
    if (uses_bus(ev->type) && ev->bus >= NET_BUSES) {
        return false;
    }
    return true;
}

#define CODE_TYPE_SHIFT 4
#define CODE_BUS_SHIFT  2
#define CODE_FIELD_MASK 0x3 /* bus and node are 2 bits each */

uint8_t net_event_encode(const struct net_event *ev) {
    if (!event_valid(ev)) {
        return NET_EVENT_NONE;
    }
    unsigned node = uses_node(ev->type) ? ev->node : 0;
    unsigned bus = uses_bus(ev->type) ? ev->bus : 0;
    return (uint8_t)((ev->type + 1u) << CODE_TYPE_SHIFT | bus << CODE_BUS_SHIFT | node);
}

int net_event_decode(uint8_t code, struct net_event *ev) {
    unsigned type_field = code >> CODE_TYPE_SHIFT;
    if (type_field == 0 || type_field - 1 > CANLINK_DOWN) {
        return -EINVAL; /* NET_EVENT_NONE, or no such type */
    }
    struct net_event got = {
        .type = (enum net_event_type)(type_field - 1),
        .node = code & CODE_FIELD_MASK,
        .bus = (code >> CODE_BUS_SHIFT) & CODE_FIELD_MASK,
    };
    /* Accept exactly the codes encode produces. This rejects node 0 or bus 2-3 where the
     * type uses them, and stray bits where it doesn't. */
    if (net_event_encode(&got) != code) {
        return -EINVAL;
    }
    *ev = got;
    return 0;
}

void net_event_format(const struct net_event *ev, char *buf, size_t len) {
    const char *name = (unsigned)ev->type <= CANLINK_DOWN ? event_names[ev->type] : "?";
    if (uses_bus(ev->type)) {
        snprintf(buf, len, "%s N%u %c", name, ev->node, ev->bus == NET_BUS_A ? 'A' : 'B');
    } else if (uses_node(ev->type)) {
        snprintf(buf, len, "%s N%u", name, ev->node);
    } else {
        snprintf(buf, len, "%s", name);
    }
}

/* 1 for yes, 0 for no, -1 for any other word */
static int parse_word(const char *arg, const char *yes, const char *no) {
    if (strcmp(arg, yes) == 0) {
        return 1;
    }
    return strcmp(arg, no) == 0 ? 0 : -1;
}

/* "1" to "3"; anything else ("4", "3x", "") gives 0, which event_valid() rejects */
static uint8_t parse_node(const char *arg) {
    bool digit = arg[0] >= '1' && arg[0] <= '0' + NET_NODES && arg[1] == '\0';
    return digit ? (uint8_t)(arg[0] - '0') : 0;
}

/* A or B, either case; anything else gives NET_BUSES, which event_valid() rejects */
static uint8_t parse_bus(const char *arg) {
    if (strcmp(arg, "A") == 0 || strcmp(arg, "a") == 0) {
        return NET_BUS_A;
    }
    if (strcmp(arg, "B") == 0 || strcmp(arg, "b") == 0) {
        return NET_BUS_B;
    }
    return NET_BUSES;
}

int net_event_parse(int argc, char *const argv[], struct net_event *ev) {
    struct net_event got = {0};
    int up = -1;

    if (argc == 3 && strcmp(argv[0], "node") == 0) {
        up = parse_word(argv[1], "up", "down");
        got.type = up == 1 ? NODE_UP : NODE_DOWN;
        got.node = parse_node(argv[2]);
    } else if (argc == 4 && strcmp(argv[0], "link") == 0) {
        up = parse_word(argv[1], "up", "down");
        got.type = up == 1 ? CANLINK_UP : CANLINK_DOWN;
        got.node = parse_node(argv[2]);
        got.bus = parse_bus(argv[3]);
    } else if (argc == 2 && strcmp(argv[0], "cockpit") == 0) {
        up = parse_word(argv[1], "ok", "error");
        got.type = up == 1 ? COCKPIT_OK : COCKPIT_ERROR;
    }
    if (up < 0 || !event_valid(&got)) {
        return -EINVAL;
    }
    *ev = got;
    return 0;
}

/* The four Part 4 mutators. They take the graph to change, so the leader can also apply
 * events to a copy (its projection of the uncommitted log tail, raft.c) without touching
 * the committed graph. */

/* addNode(X): node X is alive again, with no links until addLink */
static void addNode(struct network *g, uint8_t node) {
    /* No link reset needed: links only come up with addLink, and dropNode cleared them */
    g->node_up[node - 1] = true;
}

/* dropNode(X): node X goes down, along with every link attached to it */
static void dropNode(struct network *g, uint8_t node) {
    g->node_up[node - 1] = false;
    for (int bus = 0; bus < NET_BUSES; bus++) {
        g->link_up[node - 1][bus] = false;
    }
}

/* addLink(X, Y): the link between node X and bus Y is up */
static void addLink(struct network *g, uint8_t node, uint8_t bus) {
    g->link_up[node - 1][bus] = true;
}

/* dropLink(X, Y): the link between node X and bus Y is down; node X and the bus stay */
static void dropLink(struct network *g, uint8_t node, uint8_t bus) {
    g->link_up[node - 1][bus] = false;
}

int netstate_apply_to(struct network *g, const struct net_event *ev) {
    if (!event_valid(ev)) {
        return -EINVAL;
    }

    switch (ev->type) {
    case COCKPIT_OK:
        g->cockpit_ok = true;
        break;
    case COCKPIT_ERROR:
        g->cockpit_ok = false;
        break;
    case NODE_UP:
        addNode(g, ev->node);
        break;
    case NODE_DOWN:
        dropNode(g, ev->node);
        break;
    case CANLINK_UP:
        addLink(g, ev->node, ev->bus);
        break;
    case CANLINK_DOWN:
        dropLink(g, ev->node, ev->bus);
        break;
    }
    return 0;
}

int netstate_apply(const struct net_event *ev) {
    return netstate_apply_to(&graph, ev);
}

bool netstate_changes(const struct network *g, const struct net_event *ev) {
    struct network after = *g;
    return netstate_apply_to(&after, ev) == 0 && memcmp(&after, g, sizeof(after)) != 0;
}

struct network getNetwork(void) {
    return graph;
}

void netstate_reset(void) {
    memset(&graph, 0, sizeof(graph));
}

void netstate_format(const struct network *net, char *buf, size_t len) {
    size_t pos = 0;
    for (int n = 0; n < NET_NODES; n++) {
        if (net->node_up[n]) {
            pos += snprintf(buf + pos, len - pos, "N%d up A%c B%c | ", n + 1,
                            net->link_up[n][0] ? '+' : '-', net->link_up[n][1] ? '+' : '-');
        } else {
            pos += snprintf(buf + pos, len - pos, "N%d down | ", n + 1);
        }
    }
    snprintf(buf + pos, len - pos, "cockpit %s", net->cockpit_ok ? "OK" : "ERROR");
}

/* ---- Self-test: each step applies one event, then compares the whole graph ---- */

struct selftest_step {
    struct net_event ev;
    struct network expect;
};

#define BUS_A NET_BUS_A
#define BUS_B NET_BUS_B

static const struct selftest_step steps[] = {
    /* Bring N2 up with both links */
    {{NODE_UP, 2, 0}, {.node_up = {0, 1, 0}}},
    {{CANLINK_UP, 2, BUS_A}, {.node_up = {0, 1, 0}, .link_up = {{0, 0}, {1, 0}, {0, 0}}}},
    {{CANLINK_UP, 2, BUS_B}, {.node_up = {0, 1, 0}, .link_up = {{0, 0}, {1, 1}, {0, 0}}}},
    /* Bring N3 up with both links */
    {{NODE_UP, 3, 0}, {.node_up = {0, 1, 1}, .link_up = {{0, 0}, {1, 1}, {0, 0}}}},
    {{CANLINK_UP, 3, BUS_A}, {.node_up = {0, 1, 1}, .link_up = {{0, 0}, {1, 1}, {1, 0}}}},
    {{CANLINK_UP, 3, BUS_B}, {.node_up = {0, 1, 1}, .link_up = {{0, 0}, {1, 1}, {1, 1}}}},
    /* Cockpit heartbeats resume */
    {{COCKPIT_OK, 0, 0}, {.cockpit_ok = 1, .node_up = {0, 1, 1}, .link_up = {{0, 0}, {1, 1}, {1, 1}}}},
    /* dropLink: only that one link goes down, the node stays */
    {{CANLINK_DOWN, 2, BUS_A}, {.cockpit_ok = 1, .node_up = {0, 1, 1}, .link_up = {{0, 0}, {0, 1}, {1, 1}}}},
    /* dropNode: takes all of N3's links with it */
    {{NODE_DOWN, 3, 0}, {.cockpit_ok = 1, .node_up = {0, 1, 0}, .link_up = {{0, 0}, {0, 1}, {0, 0}}}},
    /* addNode: N3 is back, but with no links */
    {{NODE_UP, 3, 0}, {.cockpit_ok = 1, .node_up = {0, 1, 1}, .link_up = {{0, 0}, {0, 1}, {0, 0}}}},
    {{COCKPIT_ERROR, 0, 0}, {.node_up = {0, 1, 1}, .link_up = {{0, 0}, {0, 1}, {0, 0}}}},
};
#define NUM_STEPS (sizeof(steps) / sizeof(steps[0]))

static bool network_equal(const struct network *a, const struct network *b) {
    bool same_cockpit = a->cockpit_ok == b->cockpit_ok;
    bool same_nodes = memcmp(a->node_up, b->node_up, sizeof(a->node_up)) == 0;
    bool same_links = memcmp(a->link_up, b->link_up, sizeof(a->link_up)) == 0;
    return same_cockpit && same_nodes && same_links;
}

int netstate_selftest(void) {
    char ev_str[24];
    char got_str[96];
    char expect_str[96];
    int failures = 0;

    printf("netstate selftest\n");
    for (size_t i = 0; i < NUM_STEPS; i++) {
        const struct net_event *ev = &steps[i].ev;

        netstate_apply(ev);
        struct network got = getNetwork();
        netstate_format(&got, got_str, sizeof(got_str));
        net_event_format(ev, ev_str, sizeof(ev_str));

        bool pass = network_equal(&got, &steps[i].expect);
        printf("[%2zu] %-17s -> %s  %s\n", i + 1, ev_str, got_str, pass ? "PASS" : "FAIL");
        if (!pass) {
            netstate_format(&steps[i].expect, expect_str, sizeof(expect_str));
            printf("     expected %s\n", expect_str);
            failures++;
        }
    }
    printf("netstate selftest: %d/%d PASS\n", (int)NUM_STEPS - failures, (int)NUM_STEPS);

    /* Leave the graph as at boot (all down) so the shell starts clean */
    netstate_reset();
    return failures;
}
