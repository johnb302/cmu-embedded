#include "netwatch.h"
#include "raft.h"
#include <errno.h>
#include <stdio.h>

static uint8_t me;
static int64_t start_ms;
static int64_t bus_up_since[NET_BUSES]; /* when we started hearing others on each bus; -1 not now */

/* Last time each event was reported, by code slot: type x 12 + bus x 4 + node */
#define SLOTS (6 * 12)
static int64_t last_sent[SLOTS];
/* When each DOWN fact started to hold, continuously (same slots); -1 not now */
static int64_t down_since[SLOTS];

void netwatch_init(uint8_t my_id, int64_t now_ms) {
    me = my_id;
    start_ms = now_ms;
    bus_up_since[NET_BUS_A] = bus_up_since[NET_BUS_B] = -1;
    for (int i = 0; i < SLOTS; i++) {
        last_sent[i] = INT64_MIN / 2;
        down_since[i] = -1;
    }
}

void netwatch_view(const struct netwatch_obs *obs, uint8_t self, struct netwatch_view *v) {
    *v = (struct netwatch_view){0};
    v->node[self] = FACT_UP;

    /* Our own links: up on every bus where some other node's own frames are heard */
    bool heard_any = false;
    bool heard_on[NET_BUSES] = {false, false};
    for (uint8_t n = 1; n <= RAFT_N; n++) {
        for (int b = 0; b < NET_BUSES; b++) {
            if (n != self && obs->direct[n][b]) {
                heard_on[b] = true;
                heard_any = true;
            }
        }
    }
    /* Our own link: only ever down, never up. Hearing others on a bus proves our receiver
     * works, not our transmitter; the nodes that hear us vouch for that (below, from their
     * side). Board, 2026-11-16: N2's B transmitter stayed dead for seconds after its jumper
     * went back in while its receiver worked; N2 claiming "up" against N3's "down" made the
     * log flip every 1.3 s. */
    if (heard_any) {
        for (int b = 0; b < NET_BUSES; b++) {
            if (!heard_on[b]) {
                v->link[self][b] = FACT_DOWN;
            }
        }
    }

    for (uint8_t n = 1; n <= RAFT_N; n++) {
        if (n == self) {
            continue;
        }
        /* Proof that our own link on each bus works right now: a third node fresh there */
        bool proof[NET_BUSES] = {false, false};
        for (uint8_t y = 1; y <= RAFT_N; y++) {
            for (int b = 0; b < NET_BUSES; b++) {
                if (y != self && y != n && obs->fresh[y][b]) {
                    proof[b] = true;
                }
            }
        }
        if (obs->alive[n]) {
            v->node[n] = FACT_UP;
        } else if (proof[NET_BUS_A] && proof[NET_BUS_B]) {
            v->node[n] = FACT_DOWN;
        }
        if (v->node[n] != FACT_UP) {
            continue; /* a down node has no links: dropNode took them */
        }
        for (int b = 0; b < NET_BUSES; b++) {
            if (obs->direct[n][b]) {
                v->link[n][b] = FACT_UP;
            } else if (proof[b]) {
                v->link[n][b] = FACT_DOWN;
            }
        }
    }
    if (v->node[RAFT_NODE_COCKPIT] != FACT_UNKNOWN) {
        v->cockpit = v->node[RAFT_NODE_COCKPIT];
    }
}

static int slot_of(const struct net_event *ev) {
    return (int)ev->type * 12 + ev->bus * 4 + ev->node;
}

static struct net_event normalized(struct net_event ev) {
    if (ev.type != CANLINK_UP && ev.type != CANLINK_DOWN) {
        ev.bus = 0;
    }
    if (ev.type == COCKPIT_OK || ev.type == COCKPIT_ERROR) {
        ev.node = 0;
    }
    return ev;
}

/* Track how long each DOWN fact has held. Called for every fact on every tick, so a fact
 * that stops holding even once starts over. */
static void note_down(struct net_event ev, bool holds, int64_t now_ms) {
    int slot = slot_of(&ev);
    if (!holds) {
        down_since[slot] = -1;
    } else if (down_since[slot] < 0) {
        down_since[slot] = now_ms;
    }
}

/* Submit ev unless it was sent less than NETWATCH_RETRY_MS ago, or it's a DOWN that hasn't
 * held for NETWATCH_DOWN_HOLD_MS yet. 1 if sent. */
static int report(struct net_event ev, int64_t now_ms) {
    ev = normalized(ev);
    int slot = slot_of(&ev);
    if (now_ms - last_sent[slot] < NETWATCH_RETRY_MS) {
        return 0;
    }
    bool down = ev.type == NODE_DOWN || ev.type == CANLINK_DOWN || ev.type == COCKPIT_ERROR;
    if (down && (down_since[slot] < 0 || now_ms - down_since[slot] < NETWATCH_DOWN_HOLD_MS)) {
        return 0;
    }
    if (raft_submit_change(net_event_encode(&ev)) == -EAGAIN) {
        return 0; /* no leader known (mid-election): try again next tick */
    }
    last_sent[slot] = now_ms;
    return 1;
}

int netwatch_tick(const struct netwatch_obs *obs, int64_t now_ms) {
    /* Our own link on each bus: how long it has been up. Until NETWATCH_SETTLE_MS, nothing
     * on that bus counts as fresh, so nobody there can be blamed yet. */
    struct netwatch_obs o = *obs;
    for (int b = 0; b < NET_BUSES; b++) {
        bool heard = false;
        for (uint8_t n = 1; n <= RAFT_N; n++) {
            heard = heard || (n != me && obs->direct[n][b]);
        }
        if (!heard) {
            bus_up_since[b] = -1;
        } else if (bus_up_since[b] < 0) {
            bus_up_since[b] = now_ms;
        }
        if (bus_up_since[b] < 0 || now_ms - bus_up_since[b] < NETWATCH_SETTLE_MS) {
            for (uint8_t n = 1; n <= RAFT_N; n++) {
                o.fresh[n][b] = false;
            }
        }
    }
    if (now_ms - start_ms < NETWATCH_GRACE_MS) {
        return 0;
    }
    struct netwatch_view v;
    struct network g = getNetwork();
    int sent = 0;

    netwatch_view(&o, me, &v);

    /* How long each DOWN fact has held */
    for (uint8_t n = 1; n <= RAFT_N; n++) {
        note_down(normalized((struct net_event){NODE_DOWN, n, 0}), v.node[n] == FACT_DOWN, now_ms);
        for (uint8_t b = 0; b < NET_BUSES; b++) {
            note_down((struct net_event){CANLINK_DOWN, n, b}, v.link[n][b] == FACT_DOWN, now_ms);
        }
    }
    note_down(normalized((struct net_event){COCKPIT_ERROR, 0, 0}), v.cockpit == FACT_DOWN, now_ms);

    /* Nodes first (their links follow once the node is in the graph), then the cockpit,
     * then links */
    for (uint8_t n = 1; n <= RAFT_N; n++) {
        if (v.node[n] != FACT_UNKNOWN && (v.node[n] == FACT_UP) != g.node_up[n - 1]) {
            sent += report((struct net_event){v.node[n] == FACT_UP ? NODE_UP : NODE_DOWN, n, 0}, now_ms);
        }
    }
    if (v.cockpit != FACT_UNKNOWN && (v.cockpit == FACT_UP) != g.cockpit_ok) {
        sent += report((struct net_event){v.cockpit == FACT_UP ? COCKPIT_OK : COCKPIT_ERROR, 0, 0}, now_ms);
    }
    for (uint8_t n = 1; n <= RAFT_N; n++) {
        if (!g.node_up[n - 1]) {
            continue; /* links once NODE_UP has committed */
        }
        for (uint8_t b = 0; b < NET_BUSES; b++) {
            enum fact f = v.link[n][b];
            if (f != FACT_UNKNOWN && (f == FACT_UP) != g.link_up[n - 1][b]) {
                sent += report((struct net_event){f == FACT_UP ? CANLINK_UP : CANLINK_DOWN, n, b}, now_ms);
            }
        }
    }
    return sent;
}

/* ---- Routing ---- */

static bool on(const struct network *g, uint8_t n, int b) {
    return g->node_up[n - 1] && g->link_up[n - 1][b];
}

void route_build(const struct network *g, uint8_t self, struct route_table *t) {
    *t = (struct route_table){0};
    if (!on(g, self, NET_BUS_A) || !on(g, self, NET_BUS_B)) {
        return; /* only a node on both buses can bridge them */
    }
    for (uint8_t s = 1; s <= RAFT_N; s++) {
        if (s == self) {
            continue;
        }
        for (int from = 0; from < NET_BUSES; from++) {
            int to = 1 - from;
            if (!on(g, s, from) || on(g, s, to)) {
                continue; /* S isn't on `from`, or reaches `to` itself */
            }
            for (uint8_t d = 1; d <= RAFT_N; d++) {
                if (d != self && d != s && on(g, d, to) && !on(g, d, from)) {
                    t->relay[s][from] = true; /* D hears S only through us */
                }
            }
        }
    }
}

bool route_relay_observed(const struct netwatch_obs *obs, uint8_t self, uint8_t sender, uint8_t from_bus) {
    int to = 1 - from_bus;
    if (sender == self || obs->direct[sender][to]) {
        return false;
    }
    for (uint8_t d = 1; d <= RAFT_N; d++) {
        if (d != self && d != sender && obs->direct[d][to] && !obs->direct[d][from_bus]) {
            return true;
        }
    }
    return false;
}

void route_format(const struct route_table *t, char *buf, unsigned len) {
    int n = snprintf(buf, len, "relay");
    bool any = false;
    for (uint8_t s = 1; s <= RAFT_N; s++) {
        for (int from = 0; from < NET_BUSES; from++) {
            if (t->relay[s][from] && n > 0 && (unsigned)n < len) {
                n += snprintf(buf + n, len - (unsigned)n, "%s N%u %c->%c", any ? "," : "", s, 'A' + from,
                              'A' + (1 - from));
                any = true;
            }
        }
    }
    if (!any && n > 0 && (unsigned)n < len) {
        snprintf(buf + n, len - (unsigned)n, " none");
    }
}
