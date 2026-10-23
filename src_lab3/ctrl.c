#include "ctrl.h"
#include "config.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

/* Message Queue */
K_MSGQ_DEFINE_STATIC(msg_q, sizeof(struct can_frame), 1, 4);

uint8_t crc8(const uint8_t *buf, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int ctrl_decode(const struct can_frame *f, struct ctrl *out) {
    uint8_t local[7];
    memcpy(local, f->data, 7);
    /* Flags (relayed) are set in transit by N2: not part of the command */
    local[0] &= SEQ_MASK;

    uint8_t crc = crc8(local, 7);
    if (crc != f->data[7]) {
        return -EBADMSG;
    }

    /* bN is data[N]. Only steer is 16 bits; throttle and brake are one byte each */
    int16_t steer = (int16_t)sys_get_le16(&f->data[1]);
    uint8_t throttle = f->data[3];
    uint8_t brake = f->data[4];
    uint8_t buttons = f->data[5];
    if (steer < STEER_RAW_MIN || steer > STEER_RAW_MAX) {return -EBADMSG;}
    if (throttle > CTRL_PCT_MAX || brake > CTRL_PCT_MAX) {return -EBADMSG;}

    /* No bits outside the allowed buttons */
    if (buttons & ~BUTTON_MASK) {return -EBADMSG;}

    if (f->data[6] != 0) {return -EBADMSG;}

    out->steer = steer;
    out->throttle = throttle;
    out->brake = brake;
    out->buttons = buttons;

    return 0;
}

/* Driver-thread context: never block. A stale CTRL is worthless, so keep the newest. */
void ctrl_post(const struct can_frame *f) {
    if (k_msgq_put(&msg_q, f, K_NO_WAIT) != 0) {
        k_msgq_purge(&msg_q);
        (void)k_msgq_put(&msg_q, f, K_NO_WAIT);
    }
}

void ctrl_get(struct can_frame *f) {
    k_msgq_get(&msg_q, f, K_FOREVER);
}
