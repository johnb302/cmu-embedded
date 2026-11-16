#ifndef APP_RAFT_NODE_H_
#define APP_RAFT_NODE_H_

/* STM32 side of the Raft agent (shared/raft.c): the RAFT thread and its CAN frames */

struct can_frame;

/* After can_io_init(): LEDs, raft_init(), then the RAFT thread */
int raft_node_start(void);

/* From the CAN RX path: queue one Raft frame, with its receive time, for the RAFT thread */
#include <stdint.h>
void raft_node_post(const struct can_frame *f, int64_t rx_us);

/* The Raft lock, for shell commands that call into shared/ (net_shell.c). Hold it only
 * around the calls: the RAFT thread takes it every tick. */
void raft_node_lock(void);
void raft_node_unlock(void);

#endif /* APP_RAFT_NODE_H_ */
