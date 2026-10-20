/*
 * CTRL consumers.
 *
 * Concepts:
 *  - Handler -> k_msgq -> task. The CAN handler runs in the MCP2515
 *    driver's thread with the driver mutex held, so it only copies the
 *    frame into a message queue. The task (STEER or DRIVE) blocks in
 *    k_msgq_get() and runs at its own priority from tasks.h. That is the
 *    "ISR/driver + dispatch + task" chain the RTA models.
 *  - "Keep newest" queue: if the task hasn't consumed the last CTRL yet,
 *    the handler throws it away and queues the new one. A stale steering
 *    or brake command is worth nothing.
 *  - PWM via devicetree: PWM_DT_SPEC_GET_BY_NAME(zephyr_user, srv) bundles
 *    the timer, channel, period and polarity from the overlay's pwms /
 *    pwm-names properties. pwm_set_pulse_dt() changes only the pulse.
 *  - Shared outputs: DRIVE and PID both write the motor outputs, so they
 *    take drive_lock (a k_mutex) around it.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "can_bus.h"
#include "control.h"
#include "tasks.h"
#include "zone_state.h"

#define ZEPHYR_USER DT_PATH(zephyr_user)

static atomic_t n_ok;
static atomic_t n_bad;

K_MSGQ_DEFINE(ctrl_q, sizeof(struct can_frame), 1, 4);

int ctrl_decode(const struct can_frame *frame, struct ctrl *out)
{
	const uint8_t *d = frame->data;
	int16_t steer = (int16_t)sys_get_le16(&d[1]);
	uint8_t crc_in[7];

	memcpy(crc_in, d, sizeof(crc_in));
	crc_in[0] &= CAN_SEQ_MASK; /* flags (e.g. relayed) are not covered: see control.h */
	if (can_bus_crc8(crc_in, sizeof(crc_in)) != d[7]) {
		return -EBADMSG;
	}
	if (steer < CTRL_STEER_MIN || steer > CTRL_STEER_MAX ||
	    d[3] > CTRL_PCT_MAX || d[4] > CTRL_PCT_MAX ||
	    (d[5] & ~CTRL_BUTTON_MASK) != 0 || d[6] != 0) {
		return -EBADMSG;
	}

	out->steer = steer;
	out->throttle = d[3];
	out->brake = d[4];
	out->buttons = d[5];
	return 0;
}

/* Driver-thread context: queue the newest CTRL, nothing else. */
static void on_ctrl(const struct can_frame *frame, enum can_bus_id bus)
{
	ARG_UNUSED(bus);

	if (k_msgq_put(&ctrl_q, frame, K_NO_WAIT) != 0) {
		k_msgq_purge(&ctrl_q);
		(void)k_msgq_put(&ctrl_q, frame, K_NO_WAIT);
	}
}

/* Wait for the next CTRL; returns 0 with a valid one, or -EBADMSG after faulting. */
static int next_ctrl(struct ctrl *c)
{
	struct can_frame frame;

	k_msgq_get(&ctrl_q, &frame, K_FOREVER);

	if (ctrl_decode(&frame, c) != 0) {
		atomic_inc(&n_bad);
		zone_state_report_fault(FAULT_BAD_CTRL); /* error state + FAULT frame */
		return -EBADMSG;
	}
	atomic_inc(&n_ok);
	zone_state_note_valid_ctrl();
	return 0;
}

#if NODE_ID == ZONE_N2
/* ------------------------------ N2: STEER ------------------------------ */

#define SERVO_CENTER_US 1500
#define SERVO_SPAN_US   500 /* full lock = center +/- 500 us (Lab 2 LEFT/RIGHT) */

static const struct pwm_dt_spec srv = PWM_DT_SPEC_GET_BY_NAME(ZEPHYR_USER, srv);

static void steer_thread(void *p1, void *p2, void *p3)
{
	struct ctrl c;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (next_ctrl(&c) != 0) {
			continue; /* keep the last good steering angle */
		}

		/* Steering keeps following the wheel in ERROR: it's not a hazard. */
		int32_t pulse_us = SERVO_CENTER_US + ((int32_t)c.steer * SERVO_SPAN_US) / 32767;

		pwm_set_pulse_dt(&srv, PWM_USEC(pulse_us));
	}
}

K_THREAD_DEFINE(steer_tid, TASK_STACK_SIZE, steer_thread, NULL, NULL, NULL,
		STEER_PRIO, 0, SYS_FOREVER_MS);

int control_start(void)
{
	if (!pwm_is_ready_dt(&srv)) {
		return -ENODEV;
	}
	pwm_set_pulse_dt(&srv, PWM_USEC(SERVO_CENTER_US));
	k_thread_start(steer_tid);
	return 0;
}

#else
/* --------------------------- N3: DRIVE + PID --------------------------- */

#define PID_PERIOD_MS 10

static const struct pwm_dt_spec pwm_out = PWM_DT_SPEC_GET_BY_NAME(ZEPHYR_USER, pwm_out);
static const struct gpio_dt_spec dir_a = GPIO_DT_SPEC_GET(ZEPHYR_USER, dir_a_gpios);
static const struct gpio_dt_spec pwm_set_tp = GPIO_DT_SPEC_GET(ZEPHYR_USER, pwm_set_gpios);

static K_MUTEX_DEFINE(drive_lock);
static uint8_t duty_pct; /* guarded by drive_lock */
static bool braking;     /* guarded by drive_lock */

/*
 * Caller holds drive_lock. DIR_A high = drive, low = brake: every brake
 * press is a falling edge on DIR_A (the brake-chain end point, checkoff
 * step 4). PWM_SET toggles right after the timer gets a new duty (Lab 2's
 * definition), so UDP_RX -> PWM_SET is the throttle chain.
 */
static void apply(bool brake, uint8_t duty)
{
	braking = brake;
	duty_pct = brake ? 0 : duty;
	gpio_pin_set_dt(&dir_a, !brake);
	pwm_set_pulse_dt(&pwm_out, (pwm_out.period * duty_pct) / 100U);
	gpio_pin_toggle_dt(&pwm_set_tp);
}

static void drive_thread(void *p1, void *p2, void *p3)
{
	struct ctrl c;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (next_ctrl(&c) != 0) {
			k_mutex_lock(&drive_lock, K_FOREVER);
			apply(true, 0); /* bad input: fail safe */
			k_mutex_unlock(&drive_lock);
			continue;
		}

		bool brake = (c.brake >= BRAKE_ON_PCT) || (zone_state_get() == ZONE_ERROR);

		k_mutex_lock(&drive_lock, K_FOREVER);
		apply(brake, c.throttle);
		k_mutex_unlock(&drive_lock);
	}
}

/*
 * PID, every 10 ms. No encoder in this lab (test points only), so the
 * control law is a stub that holds the commanded duty. What it does do:
 * brake as soon as the zone is in ERROR, even if no CTRL arrives (e.g. a
 * FAULT from N2 with the proxy stopped): "braked motor on N3 within 100 ms".
 */
static void pid_thread(void *p1, void *p2, void *p3)
{
	int64_t next = k_uptime_get();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (zone_state_get() == ZONE_ERROR) {
			k_mutex_lock(&drive_lock, K_FOREVER);
			if (!braking) {
				apply(true, 0);
			}
			k_mutex_unlock(&drive_lock);
		}

		next += PID_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}

K_THREAD_DEFINE(drive_tid, TASK_STACK_SIZE, drive_thread, NULL, NULL, NULL,
		DRIVE_PRIO, 0, SYS_FOREVER_MS);
K_THREAD_DEFINE(pid_tid, TASK_STACK_SIZE, pid_thread, NULL, NULL, NULL,
		PID_PRIO, 0, SYS_FOREVER_MS);

int control_start(void)
{
	int ret;

	if (!pwm_is_ready_dt(&pwm_out)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&dir_a, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&pwm_set_tp, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return -EIO;
	}

	k_mutex_lock(&drive_lock, K_FOREVER);
	apply(true, 0); /* power up braked */
	k_mutex_unlock(&drive_lock);

	k_thread_start(drive_tid);
	k_thread_start(pid_tid);
	return 0;
}
#endif

void control_init(void)
{
	can_bus_set_handler(CAN_ID_CTRL, on_ctrl);
}

uint32_t control_take_ok(void)
{
	return (uint32_t)atomic_clear(&n_ok);
}

uint32_t control_take_bad(void)
{
	return (uint32_t)atomic_clear(&n_bad);
}
