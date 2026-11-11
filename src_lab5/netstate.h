#ifndef APP_NETSTATE_H_
#define APP_NETSTATE_H_

/*
 * Raft state machine (Part 4): replays network events into a graph of
 * nodes (N1 cockpit, N2 steering, N3 drivetrain) and their links to buses A and B.
 * Shell: "net show | net node up|down <1-3> | net link up|down <1-3> <A|B> | net cockpit ok|error"
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NET_NODES 3
#define NET_BUSES 2 /* 0 = A, 1 = B, same as can_bus.c */

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

/* -EINVAL if node or bus is out of range; the graph is unchanged then. */
int netstate_apply(const struct net_event *ev);

/* getNetwork(): a consistent copy of the current graph */
void netstate_get(struct network *out);

/* One line, e.g. "N1 up A+ B+ | N2 up A- B+ | N3 down | cockpit OK" */
void netstate_format(const struct network *net, char *buf, size_t len);

/* Replays a scripted event list at boot and prints PASS/FAIL per event. Returns failures. */
int netstate_selftest(void);

#endif /* APP_NETSTATE_H_ */
