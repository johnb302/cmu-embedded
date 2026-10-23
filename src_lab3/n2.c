#include "n2.h"
#include "config.h"
#include "can_bus.h"
#include "ctrl.h"
#include "state.h"
#include "wcet.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

/* SRV: TIM4 CH3 on PB8, 20 ms period (overlay: pwms / pwm-names "srv") */
static const struct pwm_dt_spec srv = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), srv);

static void steer_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        struct can_frame frame;
        struct ctrl c;

        ctrl_get(&frame);
        uint32_t t0 = wcet_start();

        if (ctrl_decode(&frame, &c) != 0) {
            /* Keep the last good angle: a jump to center isn't safer than holding */
            state_report_fault(FAULT_BAD_CTRL);
            wcet_record(WCET_STEER, t0);
            continue;
        }

        state_note_valid_ctrl();

        /* Steering keeps following the wheel in ERROR: losing steering is the
         * hazard, not having it. -32767..32766 -> 1000..2000 us */
        int32_t pulse_us = SERVO_CENTER_US + ((int32_t)c.steer * SERVO_SPAN_US) / 32767;
        pwm_set_pulse_dt(&srv, PWM_USEC(pulse_us));
        wcet_record(WCET_STEER, t0);
    }
}

K_THREAD_DEFINE(steer_tid, TASK_STACK_SIZE, steer_thread, NULL, NULL, NULL, STEER_PRIO, 0, SYS_FOREVER_MS);

static void force_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next = k_uptime_get();
    int8_t counter = 0;

    while (1) {
        uint32_t t0 = wcet_start();
        uint8_t data[FRAME_DLC] = {0};
        data[1] = counter++;

        can_io_send(ID_FORCE, data);
        wcet_record(WCET_FORCE_TX, t0);

        next += FORCE_INTERVAL_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(force_tid, TASK_STACK_SIZE, force_thread, NULL, NULL, NULL, FORCE_TX_PRIO, 0, SYS_FOREVER_MS);

int n2_start(void) {
    if (!pwm_is_ready_dt(&srv)) {
        printk("SRV PWM not ready.\n");
        return -ENODEV;
    }

    /* Power up centered */
    pwm_set_pulse_dt(&srv, PWM_USEC(SERVO_CENTER_US));

    k_thread_start(steer_tid);
    k_thread_start(force_tid);
    return 0;
}
