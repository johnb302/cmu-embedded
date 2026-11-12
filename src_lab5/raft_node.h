#ifndef APP_RAFT_NODE_H_
#define APP_RAFT_NODE_H_

/* STM32 side of the Raft agent (shared/raft.c): the RAFT thread and its CAN frames */

struct can_frame;

/* After can_io_init(): LEDs, raft_init(), then the RAFT thread */
int raft_node_start(void);

/* From the CAN RX path: queue one Raft frame for the RAFT thread */
void raft_node_post(const struct can_frame *f);

#endif /* APP_RAFT_NODE_H_ */
