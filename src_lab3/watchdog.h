#ifndef WATCHDOG_H_
#define WATCHDOG_H_

/* Start the IWDG; fed while HB_TX and HB_MON keep running. Call last. */
int watchdog_start(void);

#endif /* WATCHDOG_H_ */
