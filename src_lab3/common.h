#ifndef APP_COMMON_H_
#define APP_COMMON_H_

/* This file is called common because the heartbeat logic is common between both STM nodes. */

/* Start the threads */
int common_start(void);

/* Start the watchdog */
int common_wdg_start(void);

#endif