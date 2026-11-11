#include "state.h"
#include "config.h"
#include "can_bus.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static K_MUTEX_DEFINE(state_lock);

static const struct gpio_dt_spec err_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), err_gpios);
static uint8_t state = HB_STATE_ERROR;
static bool ctrl_seen;
static bool test_latched;
static int64_t bad_ctrl_until;
static bool peer_fault[ZONE_N3 + 1];
static uint8_t last_reason;

/* Helper fxn */
static void set_state(uint8_t st) {
    state = st;
    if (st == HB_STATE_ERROR) {
        gpio_pin_set_dt(&err_gpio, 1);
    }
    if (st == HB_STATE_OK) {
        gpio_pin_set_dt(&err_gpio, 0);
    }
}

/* Helper fxn */
static void enter_error(uint8_t reason) {
    set_state(HB_STATE_ERROR);
    ctrl_seen = false;
    last_reason = reason;
}

/* Initialization of state machine */
int state_init(void) {
    if (!gpio_is_ready_dt(&err_gpio)) {
        printk("Err GPIO is not ready.\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&err_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Err GPIO unable to be configured.\n");
        return ret;
    }

    return 0;
}

/* Get current state */
uint8_t state_get(void) {
    k_mutex_lock(&state_lock, K_FOREVER);
    uint8_t st = state;
    k_mutex_unlock(&state_lock);
    return st;
}

void state_fail(uint8_t reason) {
    int64_t now = k_uptime_get();
    k_mutex_lock(&state_lock, K_FOREVER);
    enter_error(reason);
    if (reason == FAULT_TEST_BUTTON) {
        test_latched = true;
    }

    if (reason == FAULT_BAD_CTRL) {
        bad_ctrl_until = now + BAD_CTRL_HOLD_MS;
    }
    k_mutex_unlock(&state_lock);

}

/* Time of the last FAULT this zone sent. Guarded by state_lock. */
static int64_t last_fault_tx = -100000;

void state_report_fault(uint8_t reason) {
    state_fail(reason);

    /* Catalog: at most one FAULT per FAULT_MIN_INTERVAL_MS from a zone.
     * Decide under the lock, but never hold it across the CAN send. */
    int64_t now = k_uptime_get();
    bool send = false;
    k_mutex_lock(&state_lock, K_FOREVER);
    if (now - last_fault_tx >= FAULT_MIN_INTERVAL_MS) {
        last_fault_tx = now;
        send = true;
    }
    k_mutex_unlock(&state_lock);
    if (!send) {
        return;
    }

#if NODE_ID == ZONE_N3
    /* A bad CTRL makes N2 and N3 fault at the same instant: same ID, different
     * data = bit error and retries in lockstep. N3 waits a few frame times. */
    k_msleep(FAULT_N3_DELAY_MS);
#endif

    uint8_t data[FRAME_DLC] = {0};
    data[1] = NODE_ID; /* failing zone: us */
    data[2] = reason;
    can_io_send(ID_FAULT, data);
}

void state_note_valid_ctrl(void) {
    k_mutex_lock(&state_lock, K_FOREVER);
    ctrl_seen = true;
    k_mutex_unlock(&state_lock);
}

void state_clear_test(void) {
    k_mutex_lock(&state_lock, K_FOREVER);
    test_latched = false;
    k_mutex_unlock(&state_lock);
}

bool state_local_fault(void) {
    int64_t now = k_uptime_get();
    bool local = false;
    k_mutex_lock(&state_lock, K_FOREVER);
    if (test_latched || now < bad_ctrl_until) {local = true;}
    k_mutex_unlock(&state_lock);
    return local;
}

void state_peer_report(uint8_t zone, bool fault) {
    if (zone < ZONE_N1 || zone > ZONE_N3 || zone == NODE_ID) {
        return;
    }

    k_mutex_lock(&state_lock, K_FOREVER);
    peer_fault[zone] = fault;
    k_mutex_unlock(&state_lock);
}

void state_remote_fault(uint8_t zone) {
    if (zone < ZONE_N1 || zone > ZONE_N3 || zone == NODE_ID) {
        return;
    }

    k_mutex_lock(&state_lock, K_FOREVER);
    peer_fault[zone] = true;
    enter_error(FAULT_REMOTE);
    k_mutex_unlock(&state_lock);
}

void state_update(bool all_peers_alive) {
    k_mutex_lock(&state_lock, K_FOREVER);
    int64_t now = k_uptime_get();
    bool any_peer_fault = false;
    for (int i = 0; i < ARRAY_SIZE(peer_fault); i++) {
        if (peer_fault[i]) {any_peer_fault = true; break;}
    }

    if (state == HB_STATE_OK) {
        if (!all_peers_alive) {
            enter_error(FAULT_HB_LOSS);
        } else if (any_peer_fault) {
            enter_error(FAULT_REMOTE);
        }
    } else {
        if (all_peers_alive && ctrl_seen && !test_latched && now >= bad_ctrl_until && !any_peer_fault) {
            set_state(HB_STATE_OK);
        }
    }
    k_mutex_unlock(&state_lock);
}