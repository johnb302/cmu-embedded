#ifndef TASKS_H_
#define TASKS_H_

/*
 * Thread priorities: the Part 2 task table, one place, per node. These are
 * the numbers the Part 3/4 RTA uses. Zephyr: lower number = higher
 * priority; 0..14 are preemptible, negative numbers are cooperative (the
 * MCP2515 driver's RX threads run at -14, above all of these).
 */

#include "can_bus.h"

#if NODE_ID == ZONE_N2
#define RELAY_PRIO    1 /* on the relayed brake path: above everything else we run */
#define BLINK_PRIO    2 /* BLINK_F */
#define STEER_PRIO    3
#define TEST_BTN_PRIO 4
#define FAULT_TX_PRIO 4
#define HB_TX_PRIO    5
#define HB_MON_PRIO   6
#define FORCE_TX_PRIO 7
#else
#define DRIVE_PRIO    2
#define BLINK_PRIO    3 /* BLINK_R */
#define TEST_BTN_PRIO 4
#define FAULT_TX_PRIO 4
#define PID_PRIO      5
#define HB_TX_PRIO    6
#define HB_MON_PRIO   7
#endif

#define WDT_PRIO      11 /* lowest real task: starves first if anything hogs the CPU */
#define RECOVER_PRIO  12 /* CAN stall recovery (can_bus.c) */
#define REPORT_PRIO   13 /* main() after init: the console line */
/* the shell runs at 14 */

#define TASK_STACK_SIZE 1024

#endif /* TASKS_H_ */
