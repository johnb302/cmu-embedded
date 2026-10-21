#ifndef APP_COMMON_H_
#define APP_COMMON_H_

/* This file is called common because the heartbeat logic is common between both STM nodes. */

/* Start the thread */
int common_start(void);

/* Heartbeat thread */
void hb_tx_thread(void *p1, void *p2, void *p3);

#endif