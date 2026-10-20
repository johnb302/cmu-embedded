#ifndef N2_TASKS_H_
#define N2_TASKS_H_

/*
 * Tasks only the steering zone runs. On N3 these compile to nothing.
 *
 * FORCE_TX: every 50 ms, FORCE (0x300) with a dummy int8 counter in b1.
 * RELAY:    the hop of R4.8 scenario (c). See relay notes in n2_tasks.c.
 */

#include <stdint.h>

/* Register the relay tap. Call before can_bus_init(). */
void n2_tasks_init(void);

/* Start FORCE_TX and RELAY. After can_bus_init(). */
int n2_tasks_start(void);

/* Frames RELAY forwarded since the last call (0 on N3). */
uint32_t relay_take_count(void);

#endif /* N2_TASKS_H_ */
