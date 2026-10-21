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

#endif