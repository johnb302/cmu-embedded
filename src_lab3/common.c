#include "common.h"
#include "config.h"
#include "can_bus.h"
#include <zephyr/kernel.h>

void hb_tx_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next = k_uptime_get();

    uint16_t id = 0;
    if (NODE_ID == ZONE_N2) {
        id = ID_HB_N2;
    } else {
        id = ID_HB_N3;
    }

    while(1) {
        uint8_t data[FRAME_DLC] = {0};
        data[0] = 0;
        data[1] = HB_STATE_ERROR;
        /* Set flags to show both buses healthy */
        data[2] = 0x03;
        /* Set local fault */
        data[3] = 0;

        can_io_send(id, data);
        next += HB_INTERVAL_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(hb_tid, TASK_STACK_SIZE, hb_tx_thread, NULL, NULL, NULL, HB_TX_PRIO, 0, SYS_FOREVER_MS);

/* Start the thread */
int common_start(void) {
    k_thread_start(hb_tid);

    return 0;
}
