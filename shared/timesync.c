#include "timesync.h"
#include "raft_config.h"
#include <stdio.h>

/* global(t) = base_global + (t - base_local) * (1 + ppb / 1e9). Zero = the identity: a
 * node's global clock is its local clock until a REF says otherwise. */
static int64_t base_local;
static int64_t base_global;

/* Drift: the REF (measured global, local) at the start of the current >= 1 s window */
static int64_t anchor_local;
static int64_t anchor_global;
static bool have_ppb;

static uint8_t me;
static int32_t delay_override[RAFT_N + 1]; /* `time delay`; 0 = the model */
static int late_streak;                     /* REFs in a row that look delayed */
static int far_streak;                      /* REFs in a row more than RESYNC_US off */
static struct timesync_stats st;

static int32_t tx_latency(uint8_t node) {
    return node == RAFT_NODE_COCKPIT ? TIMESYNC_TX_US_PI : TIMESYNC_TX_US_STM32;
}

static int32_t rx_latency(void) {
    return me == RAFT_NODE_COCKPIT ? TIMESYNC_RX_US_PI : TIMESYNC_RX_US_STM32;
}

void timesync_init(uint8_t my_id) {
    me = my_id;
    base_local = 0;
    base_global = 0;
    anchor_local = 0;
    anchor_global = 0;
    have_ppb = false;
    late_streak = 0;
    far_streak = 0;
    for (int i = 0; i <= RAFT_N; i++) {
        delay_override[i] = 0;
    }
    st = (struct timesync_stats){0};
}

int32_t timesync_delay_us(uint8_t from) {
    if (from <= RAFT_N && delay_override[from] != 0) {
        return delay_override[from];
    }
    return tx_latency(from) + TIMESYNC_FRAME_US + rx_latency();
}

void timesync_set_delay_us(uint8_t from, int32_t us) {
    if (from >= 1 && from <= RAFT_N) {
        delay_override[from] = us;
    }
}

/* (dt * ppb) / 1e9 without overflow for dt up to ~5 hours of µs */
static int64_t scale_ppb(int64_t dt) {
    return dt * st.ppb / 1000000000;
}

int64_t timesync_global_us(int64_t local_us) {
    int64_t dt = local_us - base_local;
    return base_global + dt + scale_ppb(dt);
}

int64_t timesync_local_us(int64_t global_us) {
    int64_t dg = global_us - base_global;
    return base_local + dg - scale_ppb(dg); /* first order: ppb^2 is below a ns per hour */
}

int64_t timesync_correction_since_ref_us(int64_t local_us) {
    return scale_ppb(local_us - base_local);
}

/* A REF from a new master was used (not discarded): checkoff step 6, "no step larger than
 * 100 us at the handover". Only REFs that are used count: the new leader's very first REF
 * often waits for the bus behind the election's last frames and is discarded as late. */
static void note_handover(uint8_t master, int64_t err) {
    if (st.synced && master != st.source) {
        st.handovers++;
        st.prev_source = st.source;
        st.handover_step_us = err;
    }
}

/* Start over from this REF: the clock reads exactly measured at rx */
static void hard_set(uint8_t master, int64_t measured, int64_t rx_us) {
    base_local = rx_us;
    base_global = measured;
    anchor_local = rx_us;
    anchor_global = measured;
    st.source = master;
    st.synced = true;
    st.resyncs++;
}

void timesync_on_ref(uint8_t master, int64_t master_us, int64_t rx_us) {
    if (st.master) {
        return; /* the master's clock is the reference: never pulled by a stale REF */
    }
    int32_t delay = timesync_delay_us(master);
    int64_t measured = master_us + delay;
    int64_t err = measured - timesync_global_us(rx_us);

    st.delay_us = delay;
    st.last_step_us = err;
    st.last_ref_local_us = rx_us;

    if (!st.synced) {
        hard_set(master, measured, rx_us);
        st.refs++;
        return;
    }

    /* Far off: boot of a new master, or a real jump. One REF could be garbage; three in a
     * row that agree are not. */
    if (err > TIMESYNC_RESYNC_US || err < -TIMESYNC_RESYNC_US) {
        if (++far_streak < 3) {
            st.outliers++;
            return;
        }
        far_streak = 0;
        late_streak = 0;
        note_handover(master, err);
        hard_set(master, measured, rx_us);
        st.refs++;
        return;
    }
    far_streak = 0;

    /* Arrived later than the model allows: it waited for the bus behind another frame.
     * Its time is fine; our receive stamp isn't. */
    if (err < -TIMESYNC_OUTLIER_US && ++late_streak < 3) {
        st.outliers++;
        return;
    }
    late_streak = 0;
    note_handover(master, err);

    /* Slew: take half the error now, the rest over the next REFs (20 ms each) */
    int64_t predicted = timesync_global_us(rx_us);
    base_global = predicted + err / 2;
    base_local = rx_us;
    st.refs++;

    /* Drift over >= 1 s. A new master restarts the window but keeps the rate: its clock
     * continues the old master's. */
    if (master != st.source) {
        st.source = master;
        anchor_local = rx_us;
        anchor_global = measured;
        return;
    }
    int64_t dl = rx_us - anchor_local;
    if (dl < TIMESYNC_DRIFT_WINDOW_US) {
        return;
    }
    int64_t dg = measured - anchor_global;
    int64_t sample = (dg - dl) * 1000000000 / dl; /* relative_drift - 1, in ppb */
    if (sample > TIMESYNC_MAX_PPB || sample < -TIMESYNC_MAX_PPB) {
        anchor_local = rx_us; /* nonsense (a gap or a step): start a new window */
        anchor_global = measured;
        return;
    }
    if (!have_ppb) {
        st.ppb = (int32_t)sample;
        have_ppb = true;
    } else {
        st.ppb += (int32_t)((sample - st.ppb) / 4);
    }
    anchor_local = rx_us;
    anchor_global = measured;
}

void timesync_set_master(bool master) {
    if (master && !st.master) {
        st.synced = true; /* never synced: the identity mapping, its own clock, is global */
        late_streak = 0;
        far_streak = 0;
    }
    st.master = master;
}

int64_t timesync_relay_master_us(uint8_t master, int64_t master_us, int64_t rx_us, int64_t tx_us) {
    return master_us + timesync_delay_us(master) + (tx_us - rx_us) + tx_latency(me) - tx_latency(master);
}

struct timesync_stats timesync_get_stats(void) {
    return st;
}

void timesync_format(char *buf, unsigned len, int64_t local_us) {
    int32_t ppb = st.ppb;
    int32_t ppm_abs = (ppb < 0 ? -ppb : ppb);
    char drift[24];

    snprintf(drift, sizeof(drift), "%c%d.%03d ppm", ppb < 0 ? '-' : '+', (int)(ppm_abs / 1000),
             (int)(ppm_abs % 1000));
    if (st.master) {
        snprintf(buf, len, "time master | drift %s (kept from the last sync) | global %lld us", drift,
                 (long long)timesync_global_us(local_us));
    } else if (!st.synced) {
        snprintf(buf, len, "not synced (no REF yet): global = local");
    } else {
        snprintf(buf, len,
                 "synced to N%u | drift %s | last step %+lld us | since REF %+lld us | delay %d us | "
                 "REFs %lu, ignored %lu, resyncs %lu",
                 st.source, drift, (long long)st.last_step_us,
                 (long long)timesync_correction_since_ref_us(local_us), (int)st.delay_us,
                 (unsigned long)st.refs, (unsigned long)st.outliers, (unsigned long)st.resyncs);
    }
}
