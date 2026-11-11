#include "netstate.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

/* Boots with nothing known: every node and link down, cockpit in error */
static struct network graph;
static K_MUTEX_DEFINE(graph_lock);

static const char *const event_names[] = {
    [COCKPIT_OK] = "COCKPIT_OK",
    [COCKPIT_ERROR] = "COCKPIT_ERROR",
    [NODE_UP] = "NODE_UP",
    [NODE_DOWN] = "NODE_DOWN",
    [CANLINK_UP] = "CANLINK_UP",
    [CANLINK_DOWN] = "CANLINK_DOWN",
};

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

    int n = ev->node - 1;
    k_mutex_lock(&graph_lock, K_FOREVER);
    switch (ev->type) {
    case COCKPIT_OK:
        graph.cockpit_ok = true;
        break;
    case COCKPIT_ERROR:
        graph.cockpit_ok = false;
        break;
    case NODE_UP:
        /* Comes back with no links: each needs its own CANLINK_UP */
        graph.node_up[n] = true;
        break;
    case NODE_DOWN:
        graph.node_up[n] = false;
        graph.link_up[n][0] = false;
        graph.link_up[n][1] = false;
        break;
    case CANLINK_UP:
        graph.link_up[n][ev->bus] = true;
        break;
    case CANLINK_DOWN:
        graph.link_up[n][ev->bus] = false;
        break;
    }
    k_mutex_unlock(&graph_lock);
    return 0;
}

void netstate_get(struct network *out) {
    k_mutex_lock(&graph_lock, K_FOREVER);
    *out = graph;
    k_mutex_unlock(&graph_lock);
}

void netstate_format(const struct network *net, char *buf, size_t len) {
    size_t pos = 0;
    for (int n = 0; n < NET_NODES; n++) {
        if (net->node_up[n]) {
            pos += snprintk(buf + pos, len - pos, "N%d up A%c B%c | ", n + 1,
                            net->link_up[n][0] ? '+' : '-', net->link_up[n][1] ? '+' : '-');
        } else {
            pos += snprintk(buf + pos, len - pos, "N%d down | ", n + 1);
        }
    }
    snprintk(buf + pos, len - pos, "cockpit %s", net->cockpit_ok ? "OK" : "ERROR");
}

/* ---- Self-test: each step applies one event, then compares the whole graph ---- */

struct selftest_step {
    struct net_event ev;
    struct network expect;
};

#define BUS_A 0
#define BUS_B 1

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

    printk("netstate selftest\n");
    for (size_t i = 0; i < ARRAY_SIZE(steps); i++) {
        const struct net_event *ev = &steps[i].ev;
        struct network got;

        netstate_apply(ev);
        netstate_get(&got);
        netstate_format(&got, got_str, sizeof(got_str));

        /* Show only the arguments this event type uses */
        char args[8] = "";
        bool is_link_event = ev->type == CANLINK_UP || ev->type == CANLINK_DOWN;
        bool is_node_event = ev->type == NODE_UP || ev->type == NODE_DOWN;
        if (is_link_event) {
            snprintk(args, sizeof(args), "N%u %c", ev->node, ev->bus == BUS_A ? 'A' : 'B');
        } else if (is_node_event) {
            snprintk(args, sizeof(args), "N%u", ev->node);
        }

        bool pass = network_equal(&got, &steps[i].expect);
        printk("[%2zu] %-13s %-4s -> %s  %s\n", i + 1, event_names[ev->type], args, got_str,
               pass ? "PASS" : "FAIL");
        if (!pass) {
            netstate_format(&steps[i].expect, expect_str, sizeof(expect_str));
            printk("     expected %s\n", expect_str);
            failures++;
        }
    }
    printk("netstate selftest: %d/%d PASS\n", (int)ARRAY_SIZE(steps) - failures, (int)ARRAY_SIZE(steps));

    /* Leave the graph as at boot so the shell starts clean */
    k_mutex_lock(&graph_lock, K_FOREVER);
    memset(&graph, 0, sizeof(graph));
    k_mutex_unlock(&graph_lock);
    return failures;
}

/* ---- Shell: drive the state machine by hand ---- */

static int cmd_show(const struct shell *sh, size_t argc, char **argv) {
    struct network net;
    char str[96];

    netstate_get(&net);
    netstate_format(&net, str, sizeof(str));
    shell_print(sh, "%s", str);
    return 0;
}

/* "up" / "down" -> true / false; -EINVAL otherwise */
static int parse_up(const struct shell *sh, const char *arg, bool *up) {
    if (strcmp(arg, "up") == 0) {
        *up = true;
        return 0;
    }
    if (strcmp(arg, "down") == 0) {
        *up = false;
        return 0;
    }
    shell_error(sh, "expected up or down, got %s", arg);
    return -EINVAL;
}

/* Applies the event, reports a bad argument, then shows the new graph */
static int apply_and_show(const struct shell *sh, const struct net_event *ev) {
    if (netstate_apply(ev) < 0) {
        shell_error(sh, "node must be 1-3, bus A or B");
        return -EINVAL;
    }
    return cmd_show(sh, 0, NULL);
}

static int cmd_node(const struct shell *sh, size_t argc, char **argv) {
    bool up;
    if (parse_up(sh, argv[1], &up) < 0) {
        return -EINVAL;
    }
    struct net_event ev = {up ? NODE_UP : NODE_DOWN, (uint8_t)atoi(argv[2]), 0};
    return apply_and_show(sh, &ev);
}

static int cmd_link(const struct shell *sh, size_t argc, char **argv) {
    bool up;
    if (parse_up(sh, argv[1], &up) < 0) {
        return -EINVAL;
    }
    bool is_a = strcmp(argv[3], "A") == 0 || strcmp(argv[3], "a") == 0;
    bool is_b = strcmp(argv[3], "B") == 0 || strcmp(argv[3], "b") == 0;
    uint8_t bus = is_a ? 0 : (is_b ? 1 : NET_BUSES); /* NET_BUSES = invalid, rejected by apply */
    struct net_event ev = {up ? CANLINK_UP : CANLINK_DOWN, (uint8_t)atoi(argv[2]), bus};
    return apply_and_show(sh, &ev);
}

static int cmd_cockpit(const struct shell *sh, size_t argc, char **argv) {
    bool ok = strcmp(argv[1], "ok") == 0;
    bool error = strcmp(argv[1], "error") == 0;
    if (!ok && !error) {
        shell_error(sh, "expected ok or error, got %s", argv[1]);
        return -EINVAL;
    }
    struct net_event ev = {ok ? COCKPIT_OK : COCKPIT_ERROR, 0, 0};
    return apply_and_show(sh, &ev);
}

SHELL_STATIC_SUBCMD_SET_CREATE(net_cmds,
    SHELL_CMD_ARG(show, NULL, "Print the network graph (getNetwork)", cmd_show, 1, 0),
    SHELL_CMD_ARG(node, NULL, "node up|down <1-3>", cmd_node, 3, 0),
    SHELL_CMD_ARG(link, NULL, "link up|down <1-3> <A|B>", cmd_link, 4, 0),
    SHELL_CMD_ARG(cockpit, NULL, "cockpit ok|error", cmd_cockpit, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(net, &net_cmds, "Raft network state machine", NULL);
