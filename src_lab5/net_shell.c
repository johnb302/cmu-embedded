/*
 * STM32 shell for the network state machine (shared/netstate.c):
 *   net show | net node up|down <1-3> | net link up|down <1-3> <A|B> | net cockpit ok|error
 *
 * TODO(step 2): append these events to the log instead of applying them, so
 * `net show` only changes after `log commit`. Then no command bypasses Raft.
 */

#include "netstate.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/shell/shell.h>

static int cmd_show(const struct shell *sh, size_t argc, char **argv) {
    struct network net = getNetwork();
    char str[96];

    netstate_format(&net, str, sizeof(str));
    shell_print(sh, "%s", str);
    return 0;
}

/* "up" / "down" -> true / false; -EINVAL otherwise */
static int parse_up(const struct shell *sh, const char *arg, bool *up) {
    if (strcmp(arg, "up") == 0) {
        *up = true;
        return 0;
    }
    if (strcmp(arg, "down") == 0) {
        *up = false;
        return 0;
    }
    shell_error(sh, "expected up or down, got %s", arg);
    return -EINVAL;
}

/* Applies the event, reports a bad argument, then shows the new graph */
static int apply_and_show(const struct shell *sh, const struct net_event *ev) {
    if (netstate_apply(ev) < 0) {
        shell_error(sh, "node must be 1-3, bus A or B");
        return -EINVAL;
    }
    return cmd_show(sh, 0, NULL);
}

static int cmd_node(const struct shell *sh, size_t argc, char **argv) {
    bool up;
    if (parse_up(sh, argv[1], &up) < 0) {
        return -EINVAL;
    }
    struct net_event ev = {up ? NODE_UP : NODE_DOWN, (uint8_t)atoi(argv[2]), 0};
    return apply_and_show(sh, &ev);
}

static int cmd_link(const struct shell *sh, size_t argc, char **argv) {
    bool up;
    if (parse_up(sh, argv[1], &up) < 0) {
        return -EINVAL;
    }
    bool is_a = strcmp(argv[3], "A") == 0 || strcmp(argv[3], "a") == 0;
    bool is_b = strcmp(argv[3], "B") == 0 || strcmp(argv[3], "b") == 0;
    uint8_t bus = is_a ? NET_BUS_A : (is_b ? NET_BUS_B : NET_BUSES); /* NET_BUSES = invalid, rejected by apply */
    struct net_event ev = {up ? CANLINK_UP : CANLINK_DOWN, (uint8_t)atoi(argv[2]), bus};
    return apply_and_show(sh, &ev);
}

static int cmd_cockpit(const struct shell *sh, size_t argc, char **argv) {
    bool ok = strcmp(argv[1], "ok") == 0;
    bool error = strcmp(argv[1], "error") == 0;
    if (!ok && !error) {
        shell_error(sh, "expected ok or error, got %s", argv[1]);
        return -EINVAL;
    }
    struct net_event ev = {ok ? COCKPIT_OK : COCKPIT_ERROR, 0, 0};
    return apply_and_show(sh, &ev);
}

SHELL_STATIC_SUBCMD_SET_CREATE(net_cmds,
    SHELL_CMD_ARG(show, NULL, "Print the network graph (getNetwork)", cmd_show, 1, 0),
    SHELL_CMD_ARG(node, NULL, "node up|down <1-3>", cmd_node, 3, 0),
    SHELL_CMD_ARG(link, NULL, "link up|down <1-3> <A|B>", cmd_link, 4, 0),
    SHELL_CMD_ARG(cockpit, NULL, "cockpit ok|error", cmd_cockpit, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(net, &net_cmds, "Raft network state machine", NULL);
