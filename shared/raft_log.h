#ifndef SHARED_RAFT_LOG_H_
#define SHARED_RAFT_LOG_H_

/*
 * The replicated log (Part 3): entries {term, index, event}, indexes from 1.
 * An entry changes the network graph (netstate.c) only once committed: raft_log_commit()
 * applies lastApplied+1 ... commitIndex through netstate_apply(), each entry once and in
 * index order, so every node's graph is a replay of the same committed prefix.
 *
 * Two byte arrays, term[] and event[] (the event as a net_event_encode() code). The index
 * is the array slot, so log_size = 30000 entries take 60 KB.
 *
 * Index 0 is the empty prefix before entry 1, with term 0: what an APPEND's prevLogIndex 0
 * and prevLogTerm 0 refer to. Real entries have term >= 1, so term 0 always means
 * "no entry there".
 *
 * Portable C, not thread-safe: called with the platform's Raft lock held, like raft.c.
 * Zeroed statics are an empty log, so nothing needs calling at boot.
 */

#include <stdint.h>

/* Empties the log and resets the graph to all down: no entries, nothing applied */
void raft_log_init(void);

/* Appends {term, event} at index last + 1 and returns that index.
 * -EINVAL if term is 0 or event doesn't decode (NET_EVENT_NONE included); -ENOSPC if full. */
int raft_log_append(uint8_t term, uint8_t event);

/* Deletes entries index ... last. In Raft a follower does this when its entry at index
 * conflicts with the leader's (step 4). -EINVAL if index is 0, past the end, or committed:
 * a committed entry is already in the graph and can't be taken back. */
int raft_log_truncate(uint16_t index);

/* Raises commitIndex to index and applies the newly committed entries to the graph.
 * Returns how many it applied. Never goes backwards: index <= commitIndex returns 0 (the
 * second bus's copy of an APPEND, or a stale leaderCommit). -EINVAL past the last entry.
 * A follower passes min(leaderCommit, index of the last entry it knows matches the
 * leader's), not just leaderCommit: entries past that may be stale (Raft Figure 2). */
int raft_log_commit(uint16_t index);

uint16_t raft_log_last_index(void);     /* 0 when empty */
uint8_t raft_log_last_term(void);       /* term of the last entry; 0 when empty */
uint8_t raft_log_term(uint16_t index);  /* 0 for index 0 or past the end */
uint8_t raft_log_event(uint16_t index); /* NET_EVENT_NONE for index 0 or past the end */
uint16_t raft_log_commit_index(void);   /* Figure 2 commitIndex; 0 = nothing committed */

#endif /* SHARED_RAFT_LOG_H_ */
