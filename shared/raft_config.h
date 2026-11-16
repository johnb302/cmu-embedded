#ifndef SHARED_RAFT_CONFIG_H_
#define SHARED_RAFT_CONFIG_H_

/*
 * Part 2 configuration. One file compiled into every node (both STM32s and the Pi),
 * so "identical on every node" holds by construction.
 */

/* nodes: a unique ID per node, used in log entries (same numbers as the Lab 3 zones) */
#define RAFT_NODE_NONE       0
#define RAFT_NODE_COCKPIT    1
#define RAFT_NODE_STEERING   2
#define RAFT_NODE_DRIVETRAIN 3

#define RAFT_N                  3                 /* N: cluster size */
#define RAFT_MAJORITY           (RAFT_N / 2 + 1)  /* = 2 */

/*
 * default_leader: the node every node boots following in term 1, or RAFT_NODE_NONE
 * (LEADER=random: no preset leader, the nodes elect one at boot). The build sets it, the
 * same for all three nodes, from RAFT_LEADER=cockpit|steering|drivetrain|random:
 *   STM32: west build ... -- ... -DRAFT_LEADER=random
 *   Pi:    RAFT_LEADER=random cockpit/deploy.sh
 */
#ifndef RAFT_DEFAULT_LEADER
#define RAFT_DEFAULT_LEADER RAFT_NODE_COCKPIT
#endif

/* Not a Part 2 key: the seed of the election timeout jitter (raft_election_timeout_ms).
 * Checkoff step 8 asks for "the seed that makes the Pi win". Set like default_leader:
 * -DRAFT_SEED=n on the STM32s, RAFT_SEED=n cockpit/deploy.sh on the Pi. */
#ifndef RAFT_SEED
#define RAFT_SEED 1
#endif

#define RAFT_HEARTBEAT_MS       20                /* heartbeat_ms: REF period, also the sync period */
#define RAFT_MISSED_HEARTBEATS  3                 /* missed_heartbeats */
#define RAFT_ELECTION_JITTER_MS 39                /* election_timeout_jitter_ms */
#define RAFT_LOG_SIZE           30000             /* log_size */
#define RAFT_DEBUG              1                 /* Debug: print election and append messages */

/* What the build passed is checked here, so a typo fails on every node alike */
#if RAFT_DEFAULT_LEADER < RAFT_NODE_NONE || RAFT_DEFAULT_LEADER > RAFT_N
#error "RAFT_DEFAULT_LEADER must be a node ID 1..N, or RAFT_NODE_NONE (LEADER=random)"
#endif

/* Not a Part 2 key, but every node must agree: checkoff step 2 */
#define RAFT_APPEND_PERIOD_MS 100 /* empty APPEND from the leader */

#endif /* SHARED_RAFT_CONFIG_H_ */
