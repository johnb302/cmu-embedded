#ifndef APP_CAN_BUS_H_
#define APP_CAN_BUS_H_

#include "config.h"
#include <stdint.h>

/* Initialize CAN buses */
int can_io_init(void);

/* Send frame on both CAN buses */
int can_io_send(uint16_t id, const uint8_t data[FRAME_DLC]);

/* Get rx callback counts */
int can_io_get_rx(int bus);

/* Get accepted frame count */
int can_io_get_accepted(void);

/* Get dropped frame count */
int can_io_get_dropped(void);

/* Compute age of heartbeat */
int64_t can_io_hb_age_ms(uint8_t zone, int bus);

/* RELAY only: send a received frame on one bus with the relayed flag set (seq kept) */
struct can_frame;
int can_io_forward(int bus, const struct can_frame *f);

#endif