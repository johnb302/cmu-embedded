/*
 * STM32 side of the Raft agent: the RAFT thread, Raft frames in and out, the LEADER
 * and SYNC LEDs, the REF_RX test point, and `raft status|log|config` in the shell.
 * The protocol is in shared/raft.c; this file only feeds it.
 */

#include "raft_node.h"
#include "raft.h"
#include "raft_config.h"
#include "raft_log.h"
#include "config.h"
#include "can_bus.h"
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

/* Pins in boards/nucleo_f401re_lab3.overlay (zephyr,user). Skipped if missing there. */
static const struct gpio_dt_spec leader_led = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), leader_gpios, {0});
static const struct gpio_dt_spec sync_led = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), sync_gpios, {0});
static const struct gpio_dt_spec ref_rx_tp = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), ref_rx_gpios, {0});

/* One lock around every call into shared/ (raft.c, netstate.c, the step 2 log) */
static K_MUTEX_DEFINE(raft_lock);

/*
 * Raft frames from the CAN RX path. Unlike CTRL and BLINK this is never keep-newest:
 * every APPEND and vote counts. 16 deep holds both bus copies of a burst.
 */
K_MSGQ_DEFINE_STATIC(raft_q, sizeof(struct can_frame), 16, 4);
static atomic_t raft_q_dropped;

static void pin_set(const struct gpio_dt_spec *pin, bool on) {
    if (pin->port != NULL) {
        gpio_pin_set_dt(pin, on);
    }
}

/* From rx_dispatch(), in the CAN driver's RX thread: keep it short */
void raft_node_post(const struct can_frame *f) {
    if (RAFT_ID_TYPE(f->id) == RAFT_ID_REF && ref_rx_tp.port != NULL) {
        gpio_pin_toggle_dt(&ref_rx_tp); /* checkoff step 9. Step 7 stamps the receive time here too. */
    }
    if (k_msgq_put(&raft_q, f, K_NO_WAIT) != 0) {
        atomic_inc(&raft_q_dropped);
    }
}

/* Called by raft.c, with raft_lock held */
void raft_platform_send(uint16_t id, uint8_t data[8]) {
    if (RAFT_ID_TYPE(id) == RAFT_ID_REF) {
        /* Master time in us, 48-bit little-endian, stamped as late as possible.
         * Uptime at tick resolution for now; step 7 replaces it with global_gettime(). */
        uint64_t t = k_ticks_to_us_floor64(k_uptime_ticks());

        for (int i = 0; i < 6; i++) {
            data[2 + i] = (uint8_t)(t >> (8 * i));
        }
    }
    (void)can_io_send_raw(id, data);
}

static void raft_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next_tick = k_uptime_get();

    while (1) {
        /* Sleep until a frame arrives or the next tick is due, whichever comes first */
        struct can_frame f;
        int64_t wait = next_tick - k_uptime_get();

        if (k_msgq_get(&raft_q, &f, K_MSEC(wait > 0 ? wait : 0)) == 0) {
            k_mutex_lock(&raft_lock, K_FOREVER);
            raft_on_frame(f.id, f.data, k_uptime_get());
            k_mutex_unlock(&raft_lock);
        }

        int64_t now = k_uptime_get();
        if (now < next_tick) {
            continue;
        }

        k_mutex_lock(&raft_lock, K_FOREVER);
        raft_tick(now);
        bool leader = raft_get_role() == RAFT_LEADER;
        bool sync = raft_in_sync(now);
        k_mutex_unlock(&raft_lock);

        pin_set(&leader_led, leader);
        pin_set(&sync_led, sync);

        /* raft.c keeps REF on its own 20 ms grid, so a late tick only needs to catch up once */
        next_tick += RAFT_TICK_MS;
        if (next_tick <= now) {
            next_tick = now + RAFT_TICK_MS;
        }
    }
}

/* 2 KB, not TASK_STACK_SIZE: RAFT_DEBUG prints with printf from this thread */
#define RAFT_STACK_SIZE 2048
K_THREAD_DEFINE(raft_tid, RAFT_STACK_SIZE, raft_thread, NULL, NULL, NULL, RAFT_PRIO, 0, SYS_FOREVER_MS);

static int pin_init(const struct gpio_dt_spec *pin) {
    if (pin->port == NULL) {
        return 0; /* not in the overlay */
    }
    if (!gpio_is_ready_dt(pin)) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(pin, GPIO_OUTPUT_INACTIVE);
}

int raft_node_start(void) {
    const struct gpio_dt_spec *pins[] = {&leader_led, &sync_led, &ref_rx_tp};

    for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
        int ret = pin_init(pins[i]);
        if (ret < 0) {
            printk("RAFT pin %zu can't be configured.\n", i);
            return ret;
        }
    }

    k_mutex_lock(&raft_lock, K_FOREVER);
    raft_init(NODE_ID, k_uptime_get());
    k_mutex_unlock(&raft_lock);

    k_thread_start(raft_tid);
    return 0;
}

void raft_node_lock(void) {
    k_mutex_lock(&raft_lock, K_FOREVER);
}

void raft_node_unlock(void) {
    k_mutex_unlock(&raft_lock);
}

/* ---- Shell: raft status | log | config ---- */

static int cmd_status(const struct shell *sh, size_t argc, char **argv) {
    char line[128];

    k_mutex_lock(&raft_lock, K_FOREVER);
    raft_status_format(line, sizeof(line), k_uptime_get());
    k_mutex_unlock(&raft_lock);

    shell_print(sh, "N%d %s | rx dropped %ld", NODE_ID, line, (long)atomic_get(&raft_q_dropped));
    return 0;
}

#define LOG_SHOW_DEFAULT 10 /* entries `raft log` prints without a count */

/* raft log [count|all]: the newest entries. The lock is taken per entry, so a long log
 * printing over the console doesn't hold up the RAFT thread. */
static int cmd_log(const struct shell *sh, size_t argc, char **argv) {
    int count = LOG_SHOW_DEFAULT;
    if (argc > 1) {
        count = strcmp(argv[1], "all") == 0 ? RAFT_LOG_SIZE : atoi(argv[1]);
        if (count <= 0) {
            shell_error(sh, "usage: raft log [count|all]");
            return -EINVAL;
        }
    }
    k_mutex_lock(&raft_lock, K_FOREVER);
    uint16_t last = raft_log_last_index();
    uint16_t commit = raft_log_commit_index();
    uint8_t term = raft_get_term();
    k_mutex_unlock(&raft_lock);

    shell_print(sh, "N%d log: %u entries, commit %u, term %u", NODE_ID, last, commit, term);
    for (int i = last > count ? last - count + 1 : 1; i <= last; i++) {
        char line[64];
        k_mutex_lock(&raft_lock, K_FOREVER);
        int ret = raft_log_format((uint16_t)i, line, sizeof(line));
        k_mutex_unlock(&raft_lock);
        if (ret < 0) {
            break; /* replaced by the leader's entries meanwhile */
        }
        shell_print(sh, "%s", line);
    }
    return 0;
}

static int cmd_config(const struct shell *sh, size_t argc, char **argv) {
    char line[192];
    raft_config_format(line, sizeof(line));
    shell_print(sh, "node %d | %s", NODE_ID, line);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(raft_cmds,
    SHELL_CMD_ARG(status, NULL, "Role, term, leader, SYNC", cmd_status, 1, 0),
    SHELL_CMD_ARG(log, NULL, "log [count|all]: the newest log entries (default 10)", cmd_log, 1, 1),
    SHELL_CMD_ARG(config, NULL, "The Part 2 config (shared/raft_config.h)", cmd_config, 1, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(raft, &raft_cmds, "Raft agent", NULL);
