/*
 * STM32 shell for the Raft client (Part 1) and the network state machine (shared/netstate.c):
 *   net show                      the committed graph (getNetwork)
 *   net node up|down <1-3>        submit an event: the leader appends it, a follower
 *   net link up|down <1-3> <A|B>  sends it to the leader (raft_submit). Then wait for it
 *   net cockpit ok|error          to commit here (Figure 2's "ok") and show the graph.
 * Nothing applies an event directly: the graph changes only when the log commits an entry,
 * so `net show` agrees on every node. The Pi takes the same commands on stdin (cockpit.c).
 */

#include "netstate.h"
#include "raft.h"
#include "raft_log.h"
#include "raft_node.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define COMMIT_WAIT_MS 500 /* the leader resends every 100 ms, so several tries */
#define COMMIT_POLL_MS 10

static int cmd_show(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    raft_node_lock();
    struct network net = getNetwork();
    raft_node_unlock();

    char str[96];
    netstate_format(&net, str, sizeof(str));
    shell_print(sh, "%s", str);
    return 0;
}

/* The first entry after index `after` holding event, once it commits here; 0 if it hasn't
 * within COMMIT_WAIT_MS */
static uint16_t wait_committed(uint8_t event, uint16_t after) {
    for (int waited = 0;; waited += COMMIT_POLL_MS) {
        raft_node_lock();
        uint16_t index = raft_log_find_committed(event, after);
        raft_node_unlock();
        if (index != 0 || waited >= COMMIT_WAIT_MS) {
            return index;
        }
        k_msleep(COMMIT_POLL_MS);
    }
}

/* net node|link|cockpit ...: argv[0] is the subcommand */
static int cmd_submit(const struct shell *sh, size_t argc, char **argv) {
    struct net_event ev;
    if (net_event_parse((int)argc, argv, &ev) < 0) {
        shell_error(sh, "usage: net %s", NET_EVENT_USAGE);
        return -EINVAL;
    }
    uint8_t event = net_event_encode(&ev);
    char name[24];
    net_event_format(&ev, name, sizeof(name));

    raft_node_lock();
    uint16_t committed = raft_log_commit_index();
    int ret = raft_submit(event);
    uint8_t leader = raft_get_leader();
    raft_node_unlock();

    if (ret < 0) {
        const char *why = ret == -EAGAIN ? "no leader known" : (ret == -ENOSPC ? "log full" : "bad event");
        shell_error(sh, "%s: not submitted, %s", name, why);
        return ret;
    }
    if (ret > 0) {
        shell_print(sh, "%s: appended as entry %d (this node leads)", name, ret);
    } else {
        shell_print(sh, "%s: sent to leader N%u", name, leader);
    }

    /* The leader knows the index. A follower looks past what was committed when it sent. */
    uint16_t index = wait_committed(event, ret > 0 ? (uint16_t)(ret - 1) : committed);
    if (index == 0) {
        shell_warn(sh, "not committed after %d ms: is a majority up?", COMMIT_WAIT_MS);
        return -ETIMEDOUT;
    }
    shell_print(sh, "ok: committed as entry %u", index);
    return cmd_show(sh, 0, NULL);
}

SHELL_STATIC_SUBCMD_SET_CREATE(net_cmds,
    SHELL_CMD_ARG(show, NULL, "Print the committed network graph (getNetwork)", cmd_show, 1, 0),
    SHELL_CMD_ARG(node, NULL, "node up|down <1-3>: submit NODE_UP/NODE_DOWN", cmd_submit, 3, 0),
    SHELL_CMD_ARG(link, NULL, "link up|down <1-3> <A|B>: submit CANLINK_UP/CANLINK_DOWN", cmd_submit, 4, 0),
    SHELL_CMD_ARG(cockpit, NULL, "cockpit ok|error: submit COCKPIT_OK/COCKPIT_ERROR", cmd_submit, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(net, &net_cmds, "Raft client: submit events, show the network", NULL);
