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

/* addNode(X): node X is alive again, with no links until addLink */
static void addNode(uint8_t node) {
    /* No link reset needed: links only come up with addLink, and dropNode cleared them */
    graph.node_up[node - 1] = true;
}

/* dropNode(X): node X goes down, along with every link attached to it */
static void dropNode(uint8_t node) {
    graph.node_up[node - 1] = false;
    for (int bus = 0; bus < NET_BUSES; bus++) {
        graph.link_up[node - 1][bus] = false;
    }
}

/* addLink(X, Y): the link between node X and bus Y is up */
static void addLink(uint8_t node, uint8_t bus) {
    graph.link_up[node - 1][bus] = true;
}

/* dropLink(X, Y): the link between node X and bus Y is down; node X and the bus stay */
static void dropLink(uint8_t node, uint8_t bus) {
    graph.link_up[node - 1][bus] = false;
}

int netstate_apply(const struct net_event *ev) {
    bool is_link_event = ev->type == CANLINK_UP || ev->type == CANLINK_DOWN;
    bool is_node_event = is_link_event || ev->type == NODE_UP || ev->type == NODE_DOWN;
    bool node_valid = ev->node >= 1 && ev->node <= NET_NODES;
    bool bus_valid = ev->bus < NET_BUSES;

    if (is_node_event && !node_valid) {
        return -EINVAL;
    }
    if (is_link_event && !bus_valid) {
        return -EINVAL;
    }

    switch (ev->type) {
    case COCKPIT_OK:
        graph.cockpit_ok = true;
        break;
    case COCKPIT_ERROR:
        graph.cockpit_ok = false;
        break;
    case NODE_UP:
        addNode(ev->node);
        break;
    case NODE_DOWN:
        dropNode(ev->node);
        break;
    case CANLINK_UP:
        addLink(ev->node, ev->bus);
        break;
    case CANLINK_DOWN:
        dropLink(ev->node, ev->bus);
        break;
    }
    return 0;
}

struct network getNetwork(void) {
    return graph;
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
    char got_str[96];
    char expect_str[96];
    int failures = 0;

    printf("netstate selftest\n");
    for (size_t i = 0; i < NUM_STEPS; i++) {
        const struct net_event *ev = &steps[i].ev;

        netstate_apply(ev);
        struct network got = getNetwork();
        netstate_format(&got, got_str, sizeof(got_str));

        /* Show only the arguments this event type uses */
        char args[8] = "";
        bool is_link_event = ev->type == CANLINK_UP || ev->type == CANLINK_DOWN;
        bool is_node_event = ev->type == NODE_UP || ev->type == NODE_DOWN;
        if (is_link_event) {
            snprintf(args, sizeof(args), "N%u %c", ev->node, ev->bus == BUS_A ? 'A' : 'B');
        } else if (is_node_event) {
            snprintf(args, sizeof(args), "N%u", ev->node);
        }

        bool pass = network_equal(&got, &steps[i].expect);
        printf("[%2zu] %-13s %-4s -> %s  %s\n", i + 1, event_names[ev->type], args, got_str,
               pass ? "PASS" : "FAIL");
        if (!pass) {
            netstate_format(&steps[i].expect, expect_str, sizeof(expect_str));
            printf("     expected %s\n", expect_str);
            failures++;
        }
    }
    printf("netstate selftest: %d/%d PASS\n", (int)NUM_STEPS - failures, (int)NUM_STEPS);

    /* Leave the graph as at boot (all down) so the shell starts clean */
    memset(&graph, 0, sizeof(graph));
    return failures;
}
