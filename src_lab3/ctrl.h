#ifndef APP_CTRL_H_
#define APP_CTRL_H_

/*
 * CTRL (0x010, Pi -> all) decode and hand-off, shared by N3 (DRIVE) and N2 (STEER).
 *
 * Payload: b0 seq/flags | b1-2 steer int16 LE | b3 throttle % | b4 brake % |
 *          b5 buttons   | b6 0                | b7 CRC-8 over b0..b6, with b0's flag bits cleared
 */

#include <stdint.h>
#include <stddef.h>
#include <zephyr/drivers/can.h>

/* A decoded, validated CTRL */
struct ctrl {
    int16_t steer;
    uint8_t throttle;
    uint8_t brake;
    uint8_t buttons;
};

/* rx_cb (driver thread): queue this CTRL frame, keeping only the newest. Never blocks. */
void ctrl_post(const struct can_frame *f);

/* Task: wait for the next CTRL frame. Blocks until one arrives. */
void ctrl_get(struct can_frame *f);

/* 0 and *out filled if the frame is a valid CTRL; -EBADMSG otherwise. */
int ctrl_decode(const struct can_frame *f, struct ctrl *out);

/* CRC-8, polynomial 0x07, initial value 0. */
uint8_t crc8(const uint8_t *buf, size_t len);

#endif /* APP_CTRL_H_ */
