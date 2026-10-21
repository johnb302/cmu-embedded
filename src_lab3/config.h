#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/* Every number two nodes must agree on, in one place. */

/* Zones */
#define ZONE_N1 1
#define ZONE_N2 2
#define ZONE_N3 3

#ifndef NODE_ID
#error "NODE_ID not passed. Build failed."
#endif
#if NODE_ID != ZONE_N2 && NODE_ID != ZONE_N3
#error "NODE_ID must be 2 or 3 (N1 is the Pi)."
#endif

/* CAN IDs (lower ID wins arbitration) */
#define ID_CTRL  0x010
#define ID_FAULT 0x020
#define ID_HB_N1 0x100
#define ID_HB_N2 0x101
#define ID_HB_N3 0x102
#define ID_BLINK 0x200
#define ID_FORCE 0x300

/* Every frame is 8 bytes. Byte 0: low nibble = seq, bit 4 = relayed by N2. */
#define FRAME_DLC    8
#define SEQ_MASK     0x0F
#define FLAG_RELAYED 0x10

/*
 * CTRL (Pi -> all): b1-2 steer int16 LE | b3 throttle % | b4 brake % |
 *                   b5 buttons | b6 0 | b7 CRC-8 (poly 0x07, init 0) over
 *                   b0..b6 with b0's flag bits masked to 0
 */
#define STEER_RAW_MIN    (-32767)
#define STEER_RAW_MAX    32766
#define CTRL_PCT_MAX     100   /* throttle and brake are 0..100 % */
#define BRAKE_ON_PCT     50    /* brake pedal past half travel = brake */
#define BUTTON_SELF_TEST (1U << 0)
#define BUTTON_RIGHT     (1U << 1)
#define BUTTON_LEFT      (1U << 2)
#define BUTTON_MASK      (BUTTON_SELF_TEST | BUTTON_RIGHT | BUTTON_LEFT)
#define CRC8_POLY        0x07

/* STEER: SRV pulse = center + steer * span / 32767 */
#define SERVO_CENTER_US 1500
#define SERVO_SPAN_US   500

/* HB (every zone): b1 state | b2 health bits | b3 1 = I have a fault of my own */
#define HB_STATE_OK    0
#define HB_STATE_ERROR 1
#define HB_HEALTH_A    (1U << 0) /* set = bus A not degraded */
#define HB_HEALTH_B    (1U << 1)

/* FAULT (any zone): b1 failing zone (the sender) | b2 reason */
#define FAULT_TEST_BUTTON 1
#define FAULT_BAD_CTRL    2
#define FAULT_HB_LOSS     3
#define FAULT_REMOTE      4

/* BLINK (Pi -> all): b1 state | b2 bit 0 = phase (1 = lamps on) */
#define BLINK_OFF    0
#define BLINK_LEFT   1
#define BLINK_RIGHT  2
#define BLINK_HAZARD 3

/* Timing */
#define HB_INTERVAL_MS 20
#define HB_MISS_LIMIT  3
#define HB_TIMEOUT_MS  (HB_MISS_LIMIT * HB_INTERVAL_MS) /* DEG on / zone failed */
#define HB_FAILSAFE_RESPONSE_BUDGET_MS 100              /* R4.5 system error */

#define PID_INTERVAL_MS   10
#define FORCE_INTERVAL_MS 50
#define DEDUP_WINDOW_MS   10  /* < 20 ms: the shortest period of any one ID */

#define BAD_CTRL_HOLD_MS     500 /* stay in ERROR after a bad CTRL, so it's visible */
#define FAULT_MIN_INTERVAL_MS 20 /* catalog: at most one FAULT per 20 ms per zone */
#define FAULT_N3_DELAY_MS      2 /* N2 and N3 both FAULT on a bad CTRL: same ID,
                                  * different data, same instant = bit error. N3 waits. */

#define BUTTON_DEBOUNCE_MS       50  /* = TEST_BTN's 50 ms min interarrival in the RTA */
#define SELF_TEST_EXIT_WINDOW_MS 500 /* B1 double press within this clears the test */

#define WDT_TIMEOUT_MS 500
#define CAN_TX_WAIT_MS 2              /* MCP2515 driver has ONE TX buffer: don't wait longer */
#define TX_STALL_MS    HB_TIMEOUT_MS  /* no frame ACKed this long: restart the controller */

/* Threads. CAN_RX_A_B is the MCP2515 driver's thread: its priority is
 * CONFIG_CAN_MCP2515_INT_THREAD_PRIO (Kconfig), cooperative -14, not set here. */
#define TASK_STACK_SIZE 1024

#if NODE_ID == ZONE_N2
/* Node 2 priorities (deadline monotonic; 0 is main during init) */
#define RELAY_PRIO    1 /* on the relayed brake chain: N3's only copy of CTRL */
#define BLINK_PRIO    2 /* BLINK_F: same code as BLINK_R (blink.c) */
#define STEER_PRIO    3
#define TEST_BTN_PRIO 4
#define HB_TX_PRIO    5
#define HB_MON_PRIO   6
#define FORCE_TX_PRIO 7
#else
/* Node 3 priorities */
#define DRIVE_PRIO    1 /* ties BLINK_R's 0.5 ms, but brake beats lights */
#define BLINK_PRIO    2 /* BLINK_R */
#define TEST_BTN_PRIO 3
#define PID_PRIO      4
#define HB_TX_PRIO    5
#define HB_MON_PRIO   6
#endif

#define MAIN_PRIO 13

#endif /* APP_CONFIG_H_ */
