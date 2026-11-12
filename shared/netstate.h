#ifndef APP_NETSTATE_H_
#define APP_NETSTATE_H_

/*
 * Raft state machine (Part 4): replays network events into a graph of
 * nodes (N1 cockpit, N2 steering, N3 drivetrain) and their links to buses A and B.
 *
 * Part 4 operations (netstate.c), applied by netstate_apply():
 *   addNode(X)     on NODE_X_UP         node X alive again, with no links yet
 *   dropNode(X)    on NODE_X_DOWN       node X down, along with every link on it
 *   addLink(X, Y)  on CANLINK_X_Y_UP    link between node X and bus Y up
 *   dropLink(X, Y) on CANLINK_X_Y_DOWN  that one link down; node X stays
 *   getNetwork()                        the only read
 * The four mutators are static so the graph changes only through netstate_apply(),
 * which Raft calls once per committed log entry. That keeps every node's graph identical.
 * Boots all down (nodes, links, cockpit ERROR); the log brings it up.
 *
 * Portable C (built for the STM32s and the Pi): no Zephyr or Linux headers in shared/.
 * Not thread-safe: each platform calls into shared/ while holding its one Raft lock.
 * STM32 shell commands live in src_lab5/net_shell.c.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NET_NODES 3
#define NET_BUSES 2 /* bus index, same as can_bus.c */
#define NET_BUS_A 0
#define NET_BUS_B 1

enum net_event_type {
    COCKPIT_OK,
    COCKPIT_ERROR,
    NODE_UP,      /* addNode(node) */
    NODE_DOWN,    /* dropNode(node) */
    CANLINK_UP,   /* addLink(node, bus) */
    CANLINK_DOWN, /* dropLink(node, bus) */
};

/* node is 1..3; bus is 0/1 and only used by CANLINK_* */
struct net_event {
    enum net_event_type type;
    uint8_t node;
    uint8_t bus;
};

/* Indexed by node - 1 */
struct network {
    bool cockpit_ok;
    bool node_up[NET_NODES];
    bool link_up[NET_NODES][NET_BUSES];
};

/* Applies one committed log entry by calling the matching Part 4 mutator.
 * -EINVAL if node or bus is out of range; the graph is unchanged then. */
int netstate_apply(const struct net_event *ev);

/* Part 4 getNetwork(): a consistent copy of the current graph, for routing (Part 5) */
struct network getNetwork(void);

/* One line, e.g. "N1 up A+ B+ | N2 up A- B+ | N3 down | cockpit OK" */
void netstate_format(const struct network *net, char *buf, size_t len);

/* Replays a scripted event list at boot and prints PASS/FAIL per event. Returns failures. */
int netstate_selftest(void);

#endif /* APP_NETSTATE_H_ */
