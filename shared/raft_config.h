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
#define RAFT_DEFAULT_LEADER     RAFT_NODE_COCKPIT /* default_leader; RAFT_NODE_NONE = elect (step 5) */
#define RAFT_HEARTBEAT_MS       20                /* heartbeat_ms: REF period, also the sync period */
#define RAFT_MISSED_HEARTBEATS  3                 /* missed_heartbeats */
#define RAFT_ELECTION_JITTER_MS 39                /* election_timeout_jitter_ms */
#define RAFT_LOG_SIZE           30000             /* log_size */
#define RAFT_DEBUG              1                 /* Debug: print election and append messages */

/* Not a Part 2 key, but every node must agree: checkoff step 2 */
#define RAFT_APPEND_PERIOD_MS 100 /* empty APPEND from the leader */

#endif /* SHARED_RAFT_CONFIG_H_ */
