#ifndef SHARED_NETWATCH_H_
#define SHARED_NETWATCH_H_

/*
 * The Raft client (Part 1, step 6): watches nodes and links, reports changes to the
 * leader as log events, and builds the routing table from getNetwork() (Part 5.1).
 *
 * Link watching. Every node turns what it hears into facts about the network:
 *   - itself: up. Its own link on bus Y: down if it hears others only on the other bus;
 *     never up (that only proves its receiver; the others vouch for its transmitter).
 *     Hearing nobody at all (alone, or isolated) says nothing.
 *   - another node X: up if heard at all (directly or relayed). Down only if X is heard
 *     on neither bus while some other node is fresh on both: a node that can't see a bus
 *     can't say who is missing from it.
 *   - X's link on Y, if X is up: up if X's own frames are heard on Y; down if not, while
 *     a third node is fresh on Y (proof our own link on Y works, NETWATCH_FRESH_MS).
 *   - the cockpit: OK while N1 is up, ERROR once it's down (COCKPIT_OK/ERROR).
 * Wherever a fact disagrees with the committed graph, the node submits the event with
 * raft_submit_change(), at most once per NETWATCH_RETRY_MS each. Every node that sees a
 * change reports it; the leader logs only events that change its graph (committed plus
 * the uncommitted tail), so each change is one entry at one index everywhere.
 *
 * Routing. A node forwards a frame from sender S, heard on bus X, onto bus Y when the
 * graph says: this node has both links, S is on X but not on Y, and some third node D is
 * on Y but not on X (it can't hear S any other way). With three nodes at most one node
 * qualifies, so nothing is forwarded twice. Part 5.2's edge case: when a second link
 * fails, the graph lags behind by the time it takes to report and commit the change,
 * and the node cut off from the leader can't even report its own. So the same rule also
 * runs on what this node hears directly (route_relay_observed): it bridges the gap until
 * the log catches up, and the graph rule then agrees with it.
 *
 * Portable C. netwatch_tick() calls into raft.c: the caller holds the Raft lock.
 */

#include "netstate.h"
#include "raft_config.h"
#include <stdbool.h>
#include <stdint.h>

#define NETWATCH_GRACE_MS 100 /* after boot: hear everyone once before claiming anything */
#define NETWATCH_RETRY_MS 300 /* the same report again, if the log still disagrees */

/* "Fresh": heard within one heartbeat period plus jitter. Ages run out 50 ms after each
 * node's last frame, and heartbeats are 20 ms apart at different phases, so for up to
 * 20 ms after our own link dies one node already looks gone while another still looks
 * heard. Blaming someone else's link needs a third node heard on that bus within 25 ms:
 * if our own link had died, every other node there would be >= 30 ms old by then. */
#define NETWATCH_FRESH_MS 25

/* The same race when our link comes back: the first node heard again is fresh while the
 * next one's heartbeat is still up to 20 ms away. Blame nobody on a bus until our own link
 * there has been up this long: every node on it heard at least twice. */
#define NETWATCH_SETTLE_MS 50

/* A link or node is reported down only once it has looked down this long, continuously.
 * Pulling a jumper by hand can upset the whole bus for a fraction of a second (board,
 * 2026-11-16: N3's B jumper took bus B down for every node for < 1 s); without the hold
 * every node would log its own link down and up again. Up is reported at once. Routing
 * doesn't wait for the log (route_relay_observed), and failure mode uses the 60 ms
 * heartbeat loss, so the hold only delays the log entry. */
#define NETWATCH_DOWN_HOLD_MS 1000

/* What this node hears, built by the platform from heartbeat ages */
struct netwatch_obs {
    bool direct[RAFT_N + 1][NET_BUSES]; /* node n's own frames on bus b, within the 50 ms timeout */
    bool fresh[RAFT_N + 1][NET_BUSES];  /* ... and within NETWATCH_FRESH_MS */
    bool alive[RAFT_N + 1];             /* node n heard at all (relayed copies too) */
};

/* A fact this node can vouch for, or not */
enum fact { FACT_UNKNOWN, FACT_DOWN, FACT_UP };

struct netwatch_view {
    enum fact node[RAFT_N + 1];
    enum fact link[RAFT_N + 1][NET_BUSES];
    enum fact cockpit;
};

void netwatch_init(uint8_t my_id, int64_t now_ms);

/* The facts this node can vouch for (pure: the sim and the tests check it directly) */
void netwatch_view(const struct netwatch_obs *obs, uint8_t me, struct netwatch_view *v);

/* Compare the facts with the committed graph and report differences. Returns how many
 * events were submitted. Caller holds the Raft lock. */
int netwatch_tick(const struct netwatch_obs *obs, int64_t now_ms);

/* The routing table (Part 5.1): relay[S][X] = forward S's frames heard on bus X to the
 * other bus */
struct route_table {
    bool relay[RAFT_N + 1][NET_BUSES];
};

void route_build(const struct network *g, uint8_t me, struct route_table *t);

/* The same rule on direct observation, built with a shorter age (RELAY_AFTER_MS) so the
 * relay starts before the receivers' heartbeat timeout */
bool route_relay_observed(const struct netwatch_obs *obs, uint8_t me, uint8_t sender, uint8_t from_bus);

/* "relay N1 A->B, N3 B->A" or "relay none", for the consoles */
void route_format(const struct route_table *t, char *buf, unsigned len);

#endif /* SHARED_NETWATCH_H_ */
