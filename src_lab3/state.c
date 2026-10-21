#include "state.h"
#include "config.h"
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

/*  */
void state_fail(uint8_t reason) {

}

void state_note_valid_ctrl(void) {

}

void state_clear_test(void) {

}

bool state_local_fault(void) {
    return false;
}

void state_peer_report(uint8_t zone, bool fault) {

}

void state_remote_fault(uint8_t zone) {

}

void state_update(bool all_peers_alive) {
    
}