/*
 * Global time on the STM32s: shared/timesync.c (the mapping and the drift) behind one
 * spinlock, so the CAN RX path, RAFT, RELAY, BLINK and PPS can all read it. PPS (6.4):
 * a rising edge at every whole second of global_gettime() on PC9 (CN10-1), and one console
 * line per edge with the global and local time and the drift.
 */

#include "time_node.h"
#include "config.h"
#include "raft.h"
#include "raft_config.h"
#include "timesync.h"
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/spinlock.h>
#include <time.h>
#include <zephyr/sys/clock.h>

static const struct gpio_dt_spec pps = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), pps_gpios, {0});

static struct k_spinlock time_lock;

int64_t local_us(void) {
    return (int64_t)(k_cycle_get_64() / (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000));
}

int64_t global_gettime(void) {
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    int64_t g = timesync_global_us(local_us());
    k_spin_unlock(&time_lock, key);
    return g;
}

int64_t time_local_for_global(int64_t global_us) {
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    int64_t l = timesync_local_us(global_us);
    k_spin_unlock(&time_lock, key);
    return l;
}

#define SPIN_US 200 /* the tick is 100 µs: sleep to within 2 ticks, then spin */

void time_wait_global(int64_t global_us) {
    while (1) {
        int64_t target = time_local_for_global(global_us); /* again after each sleep: a REF may move it */
        int64_t left = target - local_us();
        if (left <= SPIN_US) {
            while (local_us() < target) {
            }
            return;
        }
        k_sleep(K_USEC(left - SPIN_US));
    }
}

static int64_t get_ref_time(const uint8_t data[8]) {
    int64_t t = 0;
    for (int i = 0; i < 6; i++) {
        t |= (int64_t)data[2 + i] << (8 * i);
    }
    return t;
}

void time_put_ref(uint8_t data[8], int64_t global_us) {
    for (int i = 0; i < 6; i++) {
        data[2 + i] = (uint8_t)(global_us >> (8 * i));
    }
}

void time_on_ref(uint8_t master, const uint8_t data[8], int64_t rx_us) {
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    uint32_t before = timesync_get_stats().handovers;
    timesync_on_ref(master, get_ref_time(data), rx_us);
    struct timesync_stats st = timesync_get_stats();
    k_spin_unlock(&time_lock, key);
    if (st.handovers != before) { /* printed outside the lock: it masks interrupts */
        char from[12] = "own clock"; /* prev_source 0: this node was the master itself */
        if (st.prev_source != 0) {
            snprintk(from, sizeof(from), "N%u", st.prev_source);
        }
        printk("time N%d: master %s -> N%u, clocks %+lld us apart at its first REF\n", NODE_ID, from, master,
               (long long)st.handover_step_us);
    }
}

void time_set_master(bool master) {
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    timesync_set_master(master);
    k_spin_unlock(&time_lock, key);
}

void time_relay_ref(uint8_t master, uint8_t data[8], int64_t rx_us) {
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    int64_t fwd = timesync_relay_master_us(master, get_ref_time(data), rx_us, local_us());
    k_spin_unlock(&time_lock, key);
    time_put_ref(data, fwd);
}

/* ---- PPS ---- */

struct pps_line {
    int64_t global_us; /* global_gettime() at the edge */
    int64_t local_us;  /* the local clock at the same instant */
    struct timesync_stats st;
    int64_t since_ref_us; /* drift correction applied since the last REF */
};
static struct pps_line pps_last;
static K_SEM_DEFINE(pps_sem, 0, 1);

static void pps_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        int64_t next = (global_gettime() / 1000000 + 1) * 1000000;
        time_wait_global(next);
        gpio_pin_set_dt(&pps, 1);

        k_spinlock_key_t key = k_spin_lock(&time_lock);
        int64_t l = local_us();
        pps_last.local_us = l;
        pps_last.global_us = timesync_global_us(l);
        pps_last.st = timesync_get_stats();
        pps_last.since_ref_us = timesync_correction_since_ref_us(l);
        k_spin_unlock(&time_lock, key);
        k_sem_give(&pps_sem);

        /* 6.1 "Updating Zephyr on followers": the system wall clock (CLOCK_REALTIME) follows
         * global time too, set once a second at the edge. It has the kernel tick's 100 µs
         * resolution and no drift correction between sets, so the PPS, the blinkers and REF
         * use global_gettime() itself. */
        struct timespec ts = {.tv_sec = pps_last.global_us / 1000000, .tv_nsec = (pps_last.global_us % 1000000) * 1000};
        (void)sys_clock_settime(SYS_CLOCK_REALTIME, &ts);

        k_msleep(PPS_WIDTH_MS);
        gpio_pin_set_dt(&pps, 0);
    }
}

K_THREAD_DEFINE(pps_tid, TASK_STACK_SIZE, pps_thread, NULL, NULL, NULL, PPS_PRIO, 0, SYS_FOREVER_MS);

int time_node_start(void) {
    if (pps.port != NULL) {
        if (!gpio_is_ready_dt(&pps) || gpio_pin_configure_dt(&pps, GPIO_OUTPUT_INACTIVE) < 0) {
            printk("PPS pin can't be configured.\n");
            return -ENODEV;
        }
    }
    k_thread_start(pps_tid);
    return 0;
}

/* "%lld.%06lld" for a µs count (picolibc here has no float printf) */
static void fmt_us(char *buf, size_t len, int64_t us) {
    int64_t a = us < 0 ? -us : us;
    snprintk(buf, len, "%s%lld.%06lld", us < 0 ? "-" : "", (long long)(a / 1000000), (long long)(a % 1000000));
}

bool time_print_pps(int timeout_ms) {
    if (k_sem_take(&pps_sem, K_MSEC(timeout_ms)) != 0) {
        return false;
    }
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    struct pps_line p = pps_last;
    k_spin_unlock(&time_lock, key);

    char g[24];
    char l[24];
    int32_t ppb_abs = p.st.ppb < 0 ? -p.st.ppb : p.st.ppb;
    int64_t ratio = 1000000000LL + p.st.ppb; /* relative_drift = (L2 - L1) / (F2 - F1), x 1e9 */
    fmt_us(g, sizeof(g), p.global_us);
    fmt_us(l, sizeof(l), p.local_us);
    printk("PPS N%d | global %s s | local %s s | drift ratio %lld.%09lld (%c%d.%03d ppm) | %s %+lld us | "
           "last step %+lld us | %s N%u\n",
           NODE_ID, g, l, (long long)(ratio / 1000000000), (long long)(ratio % 1000000000), p.st.ppb < 0 ? '-' : '+',
           (int)(ppb_abs / 1000), (int)(ppb_abs % 1000), p.st.master ? "since last sync" : "since REF",
           (long long)p.since_ref_us, (long long)p.st.last_step_us,
           p.st.master ? "master" : "synced to", p.st.master ? NODE_ID : p.st.source);
    return true;
}

/* ---- Shell: time status | time delay <node> <us> ---- */

static int cmd_status(const struct shell *sh, size_t argc, char **argv) {
    char line[200];
    char g[24];
    char wall[24];
    struct timespec rt;
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    int64_t l = local_us();
    timesync_format(line, sizeof(line), l);
    int64_t gl = timesync_global_us(l);
    k_spin_unlock(&time_lock, key);
    (void)sys_clock_gettime(SYS_CLOCK_REALTIME, &rt);
    fmt_us(g, sizeof(g), gl);
    fmt_us(wall, sizeof(wall), (int64_t)rt.tv_sec * 1000000 + rt.tv_nsec / 1000);
    shell_print(sh, "N%d global %s s (CLOCK_REALTIME %s s) | %s", NODE_ID, g, wall, line);
    return 0;
}

static int cmd_delay(const struct shell *sh, size_t argc, char **argv) {
    if (argc == 1) {
        for (uint8_t n = 1; n <= RAFT_N; n++) {
            if (n != NODE_ID) {
                shell_print(sh, "delay from N%u: %d us", n, (int)timesync_delay_us(n));
            }
        }
        return 0;
    }
    int node = atoi(argv[1]);
    int us = argc > 2 ? atoi(argv[2]) : 0;
    if (node < 1 || node > RAFT_N || node == NODE_ID) {
        shell_error(sh, "usage: time delay [<node> <us>]   (0 us = back to the model)");
        return -EINVAL;
    }
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    timesync_set_delay_us((uint8_t)node, us);
    k_spin_unlock(&time_lock, key);
    shell_print(sh, "delay from N%d: %d us", node, (int)timesync_delay_us((uint8_t)node));
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(time_cmds,
    SHELL_CMD_ARG(status, NULL, "Global time and sync state", cmd_status, 1, 0),
    SHELL_CMD_ARG(delay, NULL, "delay [<node> <us>]: network delay from a master (tuning)", cmd_delay, 1, 2),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(time, &time_cmds, "Global time (Lab 5 Part 6)", NULL);
