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

/* TODO 6.3: decode + validate. Return -EBADMSG on any failure, 0 on success.
 *   1. CRC: copy b0..b6 into a local array, clear the flag bits of the copy's b0
 *      (keep only SEQ_MASK), crc8() the 7 bytes, compare with b7.
 *   2. Ranges:
 *        steer = (int16_t)sys_get_le16(&f->data[1]), within STEER_RAW_MIN..STEER_RAW_MAX
 *        b3 (throttle) and b4 (brake) <= CTRL_PCT_MAX
 *        b5 has no bits outside BUTTON_MASK
 *        b6 == 0
 *   3. Fill *out. */
int ctrl_decode(const struct can_frame *f, struct ctrl *out) {
    return -EBADMSG;
}

/* TODO 6.4: driver-thread context: never block.
 *   k_msgq_put(..., K_NO_WAIT); if it fails (full), k_msgq_purge() and put again. */
void ctrl_post(const struct can_frame *f) {

}

/* TODO 6.5: task context: block until a CTRL arrives.
 *   k_msgq_get(..., K_FOREVER) */
void ctrl_get(struct can_frame *f) {

}
