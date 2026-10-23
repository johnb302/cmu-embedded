#ifndef APP_BUTTON_H_
#define APP_BUTTON_H_

/*
 * TEST_BTN: the Nucleo's blue B1 (sw0, PC13) is this zone's test button.
 *   single press               -> this zone fails (latched) and sends FAULT
 *   double press while latched -> latch cleared (the zone still has to meet the exit rule)
 */

/* Configure B1 + its interrupt and start TEST_BTN. After can_io_init(): a press sends FAULT. */
int button_start(void);

#endif /* APP_BUTTON_H_ */
