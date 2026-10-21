#ifndef APP_STATE_H_
#define APP_STATE_H_
#include <stdint.h>
#include <stdbool.h>

/* Initialization of state machine */
int state_init(void);

/* Get current state */
uint8_t state_get(void);

void state_fail(uint8_t reason);

/* state_fail(reason) AND broadcast FAULT (bad CTRL, test button). Blocks up to ~6 ms on N3. */
void state_report_fault(uint8_t reason);

void state_note_valid_ctrl(void);

void state_clear_test(void);

bool state_local_fault(void);

void state_peer_report(uint8_t zone, bool fault);

void state_remote_fault(uint8_t zone);

void state_update(bool all_peers_alive);

#endif