#include "raft_log.h"
#include "netstate.h"
#include "raft_config.h"
#include <errno.h>
#include <stdio.h>

/* APPEND carries log indexes in 16 bits (prevLogIndex, leaderCommit) */
_Static_assert(RAFT_LOG_SIZE <= UINT16_MAX, "log indexes must fit in 16 bits");

/* Slot i holds entry i. Slot 0 is the empty prefix: never written, so always term 0 and
 * NET_EVENT_NONE. */
static uint8_t log_term[RAFT_LOG_SIZE + 1];
static uint8_t log_event[RAFT_LOG_SIZE + 1];

/* Names from Figure 2 of the Raft paper. Always last_applied == commit_index <= last_index
 * between calls: commit applies right away. */
static uint16_t last_index;   /* entries 1 ... last_index exist */
static uint16_t commit_index; /* commitIndex: entries 1 ... commit_index are committed */
static uint16_t last_applied; /* lastApplied: entries 1 ... last_applied are in the graph */

void raft_log_init(void) {
    last_index = 0;
    commit_index = 0;
    last_applied = 0;
    netstate_reset();
}

int raft_log_append(uint8_t term, uint8_t event) {
    struct net_event ev;
    if (term == 0 || net_event_decode(event, &ev) < 0) {
        return -EINVAL;
    }
    if (last_index == RAFT_LOG_SIZE) {
        return -ENOSPC;
    }
    last_index++;
    log_term[last_index] = term;
    log_event[last_index] = event;
    return last_index;
}

int raft_log_truncate(uint16_t index) {
    if (index <= commit_index || index > last_index) {
        return -EINVAL; /* also index 0: commit_index is never below 0 */
    }
    last_index = index - 1;
    return 0;
}

int raft_log_commit(uint16_t index) {
    if (index <= commit_index) {
        return 0;
    }
    if (index > last_index) {
        return -EINVAL;
    }
    commit_index = index;

    int applied = 0;
    while (last_applied < commit_index) {
        last_applied++;
        /* Can't fail: append only stores codes that decode, and every decoded event applies */
        struct net_event ev;
        net_event_decode(log_event[last_applied], &ev);
        netstate_apply(&ev);
        applied++;
    }
    return applied;
}

uint16_t raft_log_last_index(void) {
    return last_index;
}

uint8_t raft_log_last_term(void) {
    return log_term[last_index];
}

uint8_t raft_log_term(uint16_t index) {
    return index <= last_index ? log_term[index] : 0;
}

uint8_t raft_log_event(uint16_t index) {
    return index <= last_index ? log_event[index] : NET_EVENT_NONE;
}

uint16_t raft_log_commit_index(void) {
    return commit_index;
}

uint16_t raft_log_find_committed(uint8_t event, uint16_t after) {
    for (uint32_t index = (uint32_t)after + 1; index <= commit_index; index++) {
        if (log_event[index] == event) {
            return (uint16_t)index;
        }
    }
    return 0;
}

int raft_log_format(uint16_t index, char *buf, size_t len) {
    struct net_event ev;
    if (index == 0 || index > last_index || net_event_decode(log_event[index], &ev) < 0) {
        return -EINVAL;
    }
    char name[24];
    net_event_format(&ev, name, sizeof(name));
    snprintf(buf, len, "%5u  term %-3u %s%s", index, log_term[index], name,
             index > commit_index ? "  (not committed)" : "");
    return 0;
}
