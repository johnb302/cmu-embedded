#include "n3.h"
#include "config.h"
#include "ctrl.h"
#include "state.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

/* Outputs from devicetree (zephyr,user) */
static const struct gpio_dt_spec dir_a = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), dir_a_gpios);
static const struct gpio_dt_spec pwm_set_tp = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwm_set_gpios);
static const struct pwm_dt_spec pwm_out = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), pwm_out);

/* DRIVE and PID both write the motor outputs */
static K_MUTEX_DEFINE(drive_lock);
static bool braking; /* guarded by drive_lock */

/* Caller holds drive_lock.
 * DIR_A high = drive, low = brake: every brake press is a falling edge on DIR_A
 * (the end of the brake chain, checkoff step 4). PWM_SET toggles each time the
 * timer gets a new duty (the end of the throttle chain). */
static void apply(bool brake, uint8_t duty) {
    braking = brake;
    if (brake) {
        duty = 0;
    }
    gpio_pin_set_dt(&dir_a, !brake);
    pwm_set_pulse_dt(&pwm_out, (pwm_out.period * duty) / 100U);
    gpio_pin_toggle_dt(&pwm_set_tp);
}

static void drive_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        struct can_frame frame;
        struct ctrl c;

        ctrl_get(&frame);

        if (ctrl_decode(&frame, &c) != 0) {
            /* Bad input: fail safe first, then tell everyone */
            k_mutex_lock(&drive_lock, K_FOREVER);
            apply(true, 0);
            k_mutex_unlock(&drive_lock);
            state_report_fault(FAULT_BAD_CTRL);
            continue;
        }

        state_note_valid_ctrl();
        bool brake = (c.brake >= BRAKE_ON_PCT) || (state_get() == HB_STATE_ERROR);

        k_mutex_lock(&drive_lock, K_FOREVER);
        apply(brake, c.throttle);
        k_mutex_unlock(&drive_lock);
    }
}

/* No encoder in this lab, so the control law is a stub. Its real job: brake as
 * soon as the zone is in ERROR, even when no CTRL arrives (e.g. a FAULT with
 * the proxy stopped). */
static void pid_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next = k_uptime_get();

    while (1) {
        if (state_get() == HB_STATE_ERROR) {
            k_mutex_lock(&drive_lock, K_FOREVER);
            if (!braking) {
                apply(true, 0);
            }
            k_mutex_unlock(&drive_lock);
        }

        next += PID_INTERVAL_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(drive_tid, TASK_STACK_SIZE, drive_thread, NULL, NULL, NULL, DRIVE_PRIO, 0, SYS_FOREVER_MS);
K_THREAD_DEFINE(pid_tid, TASK_STACK_SIZE, pid_thread, NULL, NULL, NULL, PID_PRIO, 0, SYS_FOREVER_MS);

int n3_start(void) {
    if (!pwm_is_ready_dt(&pwm_out)) {
        printk("PWM_OUT not ready.\n");
        return -ENODEV;
    }
    if (!gpio_is_ready_dt(&dir_a) || !gpio_is_ready_dt(&pwm_set_tp)) {
        printk("DIR_A / PWM_SET not ready.\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&dir_a, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&pwm_set_tp, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    /* Power up braked */
    k_mutex_lock(&drive_lock, K_FOREVER);
    apply(true, 0);
    k_mutex_unlock(&drive_lock);

    k_thread_start(drive_tid);
    k_thread_start(pid_tid);
    return 0;
}
