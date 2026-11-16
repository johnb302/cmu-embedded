#ifndef SHARED_TIMESYNC_H_
#define SHARED_TIMESYNC_H_

/*
 * Global time (Part 6, step 7), the same code on all three nodes.
 *
 * Every node keeps one mapping from its local clock (µs, monotonic) to global time:
 *     global(t) = base_global + (t - base_local) * (1 + ppb / 1e9)
 * The time master (the Raft leader) never changes its mapping: at boot it is the identity
 * (global = local), and a node that wins an election keeps the mapping it was synced with,
 * so global time runs on without a step when the leader changes (checkoff step 6: no step
 * larger than 100 µs at the handover).
 *
 * Followers re-anchor the mapping at every REF (Figure 4):
 *     follower_clock = L + network_delay
 * L is the master time in the REF (stamped by the sender just before it hands the frame to
 * its CAN controller), network_delay the TTCAN-style estimate from the bus itself: the
 * frame's time on the wire at 500 kbit/s plus the sender's and the receiver's fixed
 * controller latency (both MCP2515s over SPI). It depends only on the frame and the two
 * node types, never on a round trip.
 *
 * The drift (6.2): relative_drift = (L2 - L1) / (F2 - F1), measured over >= 1 s between
 * REFs (one 20 ms period would be all jitter), smoothed, and applied between REFs as the
 * ppb term. Each REF's offset correction is applied half at a time (the clock slews
 * instead of stepping), and a REF that looks delayed by more than a frame (it waited for
 * the bus) is ignored, unless several in a row say the same, which is a real step.
 *
 * Portable C, no locking: the platform holds its time lock around every call.
 */

#include <stdbool.h>
#include <stdint.h>

/* An 8-byte standard frame at 500 kbit/s: 108 bits + ~7 stuff bits + 3 IFS ≈ 236 µs */
#define TIMESYNC_FRAME_US 236

/* Fixed latencies, measured on the boards (PLAN.md step 7) and adjustable at run time
 * with `time delay` on every console. TX: from the stamp to the frame's start on the
 * wire; RX: from the end of the frame to the receive stamp. */
#define TIMESYNC_TX_US_STM32 70  /* Zephyr mcp2515 driver: SPI load + RTS */
#define TIMESYNC_RX_US_STM32 70  /* INT -> driver thread -> SPI read -> rx callback */
#define TIMESYNC_TX_US_PI    150 /* SocketCAN queue -> mcp251x worker -> SPI */
#define TIMESYNC_RX_US_PI    60  /* kernel receive timestamp (SO_TIMESTAMPNS) */

#define TIMESYNC_DRIFT_WINDOW_US 2000000 /* drift measured over >= 2 s: REF jitter is ±100 µs */
#define TIMESYNC_MAX_PPB         500000  /* 500 ppm: anything more is a bad measurement */
#define TIMESYNC_OUTLIER_US      150     /* REF this much later than expected: it queued */
#define TIMESYNC_RESYNC_US       2000    /* beyond this, step at once (boot, new master) */

struct timesync_stats {
    bool synced;          /* a REF has set the clock, or this node is the master */
    bool master;          /* this node's clock is the global clock */
    uint8_t source;       /* the master of the last REF used */
    int32_t ppb;          /* global runs this many parts per billion faster than local */
    int64_t last_step_us; /* the offset error at the last REF: measured - predicted */
    int64_t last_ref_local_us;
    int32_t delay_us;     /* network delay assumed for the last REF */
    uint32_t refs;        /* REFs used */
    uint32_t outliers;    /* REFs ignored as delayed */
    uint32_t resyncs;     /* hard steps (boot, a new master far off, a real jump) */
    uint32_t handovers;   /* REFs from a new master (after an election) */
    uint8_t prev_source;  /* the master before the last handover */
    int64_t handover_step_us; /* how far the new master's clock was from ours at its first REF */
};

/* my_id's node type sets the RX latency; the sender's sets the TX latency */
void timesync_init(uint8_t my_id);

/* The global clock at local time local_us */
int64_t timesync_global_us(int64_t local_us);

/* The local time at which the global clock reads global_us (for PPS and blinker edges) */
int64_t timesync_local_us(int64_t global_us);

/* Follower: a REF from master carrying master_us arrived at local time rx_us */
void timesync_on_ref(uint8_t master, int64_t master_us, int64_t rx_us);

/* This node is (true) or isn't (false) the time master: the Raft leader */
void timesync_set_master(bool master);

/* A relay forwarding a REF (step 6): the master time it should carry on the next hop.
 * Adds the first hop's delay and the time the frame spent here, so the receiver's usual
 * one-hop correction comes out right. tx_us is when the relay hands it to its controller. */
int64_t timesync_relay_master_us(uint8_t master, int64_t master_us, int64_t rx_us, int64_t tx_us);

/* Network delay from node `from` to this node (µs), and the override for `time delay` */
int32_t timesync_delay_us(uint8_t from);
void timesync_set_delay_us(uint8_t from, int32_t us);

/* The drift correction applied since the last REF at local time local_us (PPS line) */
int64_t timesync_correction_since_ref_us(int64_t local_us);

struct timesync_stats timesync_get_stats(void);

/* One line for the consoles' `time status` and the PPS print */
void timesync_format(char *buf, unsigned len, int64_t local_us);

#endif /* SHARED_TIMESYNC_H_ */
