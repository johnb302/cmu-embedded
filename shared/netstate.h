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
 * which the log (raft_log.c) calls once per committed entry. That keeps every node's graph
 * identical. Boots all down (nodes, links, cockpit ERROR); the log brings it up.
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

/*
 * One byte per event: the log's event[] array and APPEND b5.
 *   bits 7-4  type + 1: 1 COCKPIT_OK, 2 COCKPIT_ERROR, 3 NODE_UP, 4 NODE_DOWN,
 *             5 CANLINK_UP, 6 CANLINK_DOWN. 0 = no event (an empty APPEND).
 *   bits 3-2  bus, 0 = A, 1 = B (CANLINK_* only, else 0)
 *   bits 1-0  node 1-3 (NODE_* and CANLINK_* only, else 0)
 * In candump the high digit is the type and the low digit the node, +4 on bus B:
 * 0x56 = CANLINK_UP N2 B, 0x43 = NODE_DOWN N3, 0x10 = COCKPIT_OK.
 * 20 codes are valid; the other 236 byte values don't decode.
 */
#define NET_EVENT_NONE 0

/* The code for ev, or NET_EVENT_NONE if ev is invalid (bad type, node or bus). Fields the
 * type doesn't use are ignored, so each event has exactly one code. */
uint8_t net_event_encode(const struct net_event *ev);

/* Fills *ev from a code. -EINVAL (and *ev untouched) for NET_EVENT_NONE or any byte that
 * net_event_encode() never produces. */
int net_event_decode(uint8_t code, struct net_event *ev);

/* One event, e.g. "CANLINK_UP N2 B", "NODE_DOWN N3", "COCKPIT_OK" */
void net_event_format(const struct net_event *ev, char *buf, size_t len);

/* An event as the consoles' `net` commands take it, argv[0] first (the STM32 shell and the
 * Pi's stdin share this, so the syntax is the same on every node):
 *   node up|down <1-3>   link up|down <1-3> <A|B>   cockpit ok|error
 * Fills *ev and returns 0, or -EINVAL (*ev untouched) for anything else. */
#define NET_EVENT_USAGE "node up|down <1-3> | link up|down <1-3> <A|B> | cockpit ok|error"
int net_event_parse(int argc, char *const argv[], struct net_event *ev);

/* Applies one committed log entry by calling the matching Part 4 mutator.
 * -EINVAL if the type, node or bus is out of range; the graph is unchanged then. */
int netstate_apply(const struct net_event *ev);

/* The same, on any graph: the leader projects its uncommitted entries onto a copy */
int netstate_apply_to(struct network *g, const struct net_event *ev);

/* Would ev change g? Lets the leader log each change once, however many nodes report it. */
bool netstate_changes(const struct network *g, const struct net_event *ev);

/* Back to the boot graph (all down). For raft_log_init(): an empty log means an empty graph. */
void netstate_reset(void);

/* Part 4 getNetwork(): a consistent copy of the current graph, for routing (Part 5) */
struct network getNetwork(void);

/* One line, e.g. "N1 up A+ B+ | N2 up A- B+ | N3 down | cockpit OK" */
void netstate_format(const struct network *net, char *buf, size_t len);

/* Replays a scripted event list at boot and prints PASS/FAIL per event. Returns failures. */
int netstate_selftest(void);

#endif /* APP_NETSTATE_H_ */
