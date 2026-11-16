#ifndef APP_CAN_BUS_H_
#define APP_CAN_BUS_H_

#include "config.h"
#include <stdint.h>

/* Initialize CAN buses */
int can_io_init(void);

/* Send frame on both CAN buses */
int can_io_send(uint16_t id, const uint8_t data[FRAME_DLC]);

/* Lab 5 (Raft frames): send on both buses as-is, with no seq stamped into byte 0 */
int can_io_send_raw(uint16_t id, const uint8_t data[FRAME_DLC]);

/* Get rx callback counts */
int can_io_get_rx(int bus);

/* Get accepted frame count */
int can_io_get_accepted(void);

/* Get dropped frame count */
int can_io_get_dropped(void);

/* Compute age of heartbeat (any copy, relayed too: is the zone alive?) */
int64_t can_io_hb_age_ms(uint8_t zone, int bus);

/* Lab 5: age of the zone's own heartbeat on this bus, relayed copies not counted (is the
 * zone's link on this bus up?) */
int64_t can_io_hb_direct_age_ms(uint8_t zone, int bus);

/* Lab 5: send on one bus only, as-is (a REF, stamped per copy) */
int can_io_send_one(int bus, uint16_t id, const uint8_t data[FRAME_DLC]);

/* RELAY only: send a received frame on one bus with `flag` set in b0 (seq kept) */
struct can_frame;
int can_io_forward(int bus, const struct can_frame *f, uint8_t flag);

#endif