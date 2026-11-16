/*
 * N1 cockpit zone (Raspberry Pi), Lab 3 Part 5.
 *
 * UDP_RX : proxy UDP state (4-byte counter + DIJOYSTATE2_t) -> CTRL on both
 *          buses, every update (R4.4). Test points UDP_RX, CMD_TX,
 *          CAN_TX_A, CAN_TX_B toggle along the way (brake chain, stage 1).
 * PERIODIC (20 ms): HB_N1 on both buses; HB_MON (DEG_A/DEG_B, ERR, zone
 *          state); blinker master (BLINK on change and every 500 ms toggle).
 * CAN_RX : both interfaces; (ID, seq) dedup like the STM32s; heartbeats,
 *          FAULT and FORCE. FORCE goes back to the proxy over UDP.
 * main   : console, one command per line on stdin: "b" = corrupt the next CTRL
 *          (bad-input test), "q" = quit, and Lab 5's net and raft commands, the
 *          same as the STM32 shell ("h" lists them). run_cockpit.sh feeds stdin
 *          from cmd.fifo. Status line once a second. `kill -USR1 <pid>` also
 *          injects a bad CTRL.
 *
 * Bus naming: the bus is decided by the MCP2515's chip select, not by the
 * Linux name. CE0 (spi0.0) is bus A, CE1 (spi0.1) is bus B; the names
 * can0/can1 follow probe order, so we look them up in sysfs.
 *
 * Build: make   (cockpit/deploy.sh from the Mac copies to ~/lab5 and builds there)
 * Run:   ./cockpit            (sudo ./cockpit adds SCHED_FIFO + mlockall)
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "state.h"
#include "netstate.h"    /* ../shared: the network graph the log builds */
#include "raft.h"        /* ../shared: the Raft agent, same code as the STM32s */
#include "raft_config.h"
#include "raft_log.h"
#include "netwatch.h"    /* ../shared: link watching and the routing table (step 6) */
#include "timesync.h"    /* ../shared: global time (step 7) */

/* ------------------------------ catalog ------------------------------- */

#define ID_CTRL  0x010
#define ID_FAULT 0x020
#define ID_HB_N1 0x100
#define ID_HB_N2 0x101
#define ID_HB_N3 0x102
#define ID_BLINK 0x200
#define ID_FORCE 0x300

#define ZONE_N1 1
#define ZONE_N2 2
#define ZONE_N3 3

#define SEQ_MASK      0x0F
#define FLAG_RELAYED  0x10

#define FAULT_TEST_BUTTON 1
#define FAULT_BAD_CTRL    2
#define FAULT_HB_LOSS     3
#define FAULT_REMOTE      4 /* local display only */
#define FAULT_INPUT_LOST  5 /* local display only: proxy stopped */

#define HB_STATE_OK    0
#define HB_STATE_ERROR 1
#define HB_HEALTH_A    0x01
#define HB_HEALTH_B    0x02

#define BLINK_OFF    0
#define BLINK_LEFT   1
#define BLINK_RIGHT  2
#define BLINK_HAZARD 3

/* -------------------------- timing (one place) ------------------------- */

#define PERIOD_MS       20   /* HB_N1, blinker master */
#define MON_PERIOD_MS   10   /* HB_MON */
#define HB_TIMEOUT_MS   50   /* 2 missed + 10 ms margin; + MON_PERIOD_MS = 60 ms from the fault */
#define DUP_WINDOW_MS   10
#define BLINK_HALF_MS   500  /* 1 Hz (R2.4): turn signals */
#define BLINK_HAZARD_HALF_MS 250 /* Lab 5: hazards at 2 Hz (STM32 config.h) */
#define UDP_TIMEOUT_MS  200  /* proxy considered stopped */
#define REMOTE_CLEAR_MS (3 * PERIOD_MS)
#define RELAY_AFTER_MS  30   /* Lab 5 relay: the sender counts as off a bus after this (STM32 config.h) */
#define DOUBLE_MS       500  /* test button double press */
#define FAULT_MIN_MS    20

/* ------------------------- wheel / CTRL values ------------------------- */

#define R_PORT 8000 /* proxy -> Pi */
#define S_PORT 8001 /* Pi -> proxy (force) */

#define WHEEL_BTN_SELF_TEST 10
/* The G920's turn buttons as SDL reports them with SDL_JOYSTICK_MFI=0 (the IOKit backend):
 * the left button is index 5, the right 4. With 4/5 the signals came out mirrored. */
#define WHEEL_BTN_LEFT      5
#define WHEEL_BTN_RIGHT     4

#define BTN_SELF_TEST 0x01
#define BTN_RIGHT     0x02
#define BTN_LEFT      0x04

#define STEER_MIN       (-32767)
#define STEER_MAX       32766
#define THROTTLE_MAX    32767
/*
 * mac_proxy.py convention: lRz = 0 released .. 32767 fully pressed. Our G920's brake
 * pedal doesn't always rest at 0: depending on its power-up calibration it rests at 0
 * or around 29426..30582 (2026-11-16: 30582), always rising to 32767 when pressed. So the
 * rest point is learned: the lowest lRz seen (build_ctrl), plus a 3 % dead band of the
 * remaining range so a released pedal reads 0 %. Starting the proxy with the brake held
 * reads 0 % until the pedal is first released.
 */
#define BRAKE_FULL      32767
#define BRAKE_DEADBAND_PCT 3
#define TURN_ARM        1000  /* steer past this in the turn direction arms self-cancel */
#define TURN_CENTER     500   /* back inside this: cancel */

/* ------------------------------ GPIO (BCM) ----------------------------- */

#define GPIO_UDP_RX   5
#define GPIO_CMD_TX   6
#define GPIO_CAN_TX_A 13
#define GPIO_CAN_TX_B 19
#define GPIO_DEG_A    20
#define GPIO_DEG_B    21
#define GPIO_ERR      26
#define GPIO_LEADER   16 /* Lab 5, pin 36: on while this node is the Raft leader */
#define GPIO_SYNC     12 /* Lab 5, pin 32: on while time sync is fresh (raft_in_sync) */
#define GPIO_REF_RX   27 /* Lab 5 test point, pin 13: toggles on every REF received */
#define GPIO_PPS      22 /* Lab 5, pin 15: rising edge at every whole second of global time */
#define GPIO_HZ       23 /* Lab 5, pin 16: the hazard lamp, timed from global time like FL/RL */

static volatile uint32_t *gpio; /* /dev/gpiomem: memory-mapped hardware registers */

#define GPFSEL(n) (gpio[(n)])
#define GPSET0    (gpio[0x1C / 4])
#define GPCLR0    (gpio[0x28 / 4])
#define GPLEV0    (gpio[0x34 / 4])

static void gpio_output(int pin)
{
	uint32_t v = GPFSEL(pin / 10);

	v &= ~(7u << ((pin % 10) * 3));
	v |= (1u << ((pin % 10) * 3));
	GPFSEL(pin / 10) = v;
	GPCLR0 = 1u << pin;
}

static void gpio_put(int pin, bool on)
{
	if (on) {
		GPSET0 = 1u << pin;
	} else {
		GPCLR0 = 1u << pin;
	}
}

static void gpio_toggle(int pin)
{
	gpio_put(pin, (GPLEV0 & (1u << pin)) == 0);
}

static int gpio_init(void)
{
	int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);

	if (fd < 0) {
		perror("open /dev/gpiomem");
		return -1;
	}
	gpio = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (gpio == MAP_FAILED) {
		perror("mmap gpiomem");
		return -1;
	}
	int pins[] = { GPIO_UDP_RX, GPIO_CMD_TX, GPIO_CAN_TX_A, GPIO_CAN_TX_B,
		       GPIO_DEG_A, GPIO_DEG_B, GPIO_ERR, GPIO_LEADER, GPIO_SYNC, GPIO_REF_RX,
		       GPIO_PPS, GPIO_HZ };
	for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		gpio_output(pins[i]);
	}
	gpio_put(GPIO_ERR, true); /* power up in ERROR */
	return 0;
}

/* -------------------------------- time --------------------------------- */

/* The local clock in µs (CLOCK_MONOTONIC). Global time maps from it (global_gettime). */
static int64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static int64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void sleep_until_ms(int64_t t)
{
	struct timespec ts = { .tv_sec = t / 1000, .tv_nsec = (t % 1000) * 1000000 };

	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {
	}
}

static void sleep_until_us(int64_t t)
{
	struct timespec ts = { .tv_sec = t / 1000000, .tv_nsec = (t % 1000000) * 1000 };

	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {
	}
}

/* ---------------------- global time (Lab 5, Part 6) ---------------------- */
/*
 * shared/timesync.c behind one lock: the RAFT, CAN_RX, PPS and BLINK threads all read it.
 * The Pi doesn't call clock_settime(): it runs without root and the system clock belongs to
 * NTP. global_gettime() is the synchronized clock instead.
 */
static pthread_mutex_t time_lock = PTHREAD_MUTEX_INITIALIZER;

/* Part 6.3: the current global time, µs */
static int64_t global_gettime(void)
{
	pthread_mutex_lock(&time_lock);
	int64_t g = timesync_global_us(now_us());
	pthread_mutex_unlock(&time_lock);
	return g;
}

/* Sleep until global time reaches g, then spin the last 300 µs */
static void wait_global(int64_t g)
{
	while (1) {
		pthread_mutex_lock(&time_lock);
		int64_t target = timesync_local_us(g);
		pthread_mutex_unlock(&time_lock);
		int64_t left = target - now_us();

		if (left <= 300) {
			while (now_us() < target) {
			}
			return;
		}
		sleep_until_us(target - 300);
	}
}

static int64_t ref_time(const uint8_t *data)
{
	int64_t t = 0;

	for (int i = 0; i < 6; i++) {
		t |= (int64_t)data[2 + i] << (8 * i);
	}
	return t;
}

static void put_ref_time(uint8_t *data, int64_t t)
{
	for (int i = 0; i < 6; i++) {
		data[2 + i] = (uint8_t)(t >> (8 * i));
	}
}

static uint8_t crc8(const uint8_t *buf, size_t len)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

/* --------------------------------- CAN --------------------------------- */

enum { BUS_A, BUS_B, BUS_COUNT };

static int can_fd[BUS_COUNT] = { -1, -1 };
static char can_name[BUS_COUNT][IFNAMSIZ];
static const int tx_tp[BUS_COUNT] = { GPIO_CAN_TX_A, GPIO_CAN_TX_B };
static atomic_uint tx_fail[BUS_COUNT];

/* Lab 5: held around every call into shared/ (raft.c, netstate.c). CAN_RX and RAFT both call in. */
static pthread_mutex_t raft_lock = PTHREAD_MUTEX_INITIALIZER;

/* Find canX whose parent is spi0.<cs>: CE0 = bus A, CE1 = bus B. */
static int find_iface(int cs, char *out)
{
	char want[16], path[128], link[PATH_MAX];

	snprintf(want, sizeof(want), "spi0.%d", cs);
	for (int i = 0; i < 4; i++) {
		snprintf(path, sizeof(path), "/sys/class/net/can%d/device", i);
		ssize_t n = readlink(path, link, sizeof(link) - 1);

		if (n <= 0) {
			continue;
		}
		link[n] = '\0';
		if (strstr(link, want) != NULL) {
			snprintf(out, IFNAMSIZ, "can%d", i);
			return 0;
		}
	}
	return -1;
}

static int can_open(const char *name)
{
	struct sockaddr_can addr = { .can_family = AF_CAN };
	struct can_filter rfilter[] = {
		{ ID_FAULT, CAN_SFF_MASK }, { ID_HB_N2, CAN_SFF_MASK },
		{ ID_HB_N3, CAN_SFF_MASK }, { ID_FORCE, CAN_SFF_MASK },
		/* Lab 5: every Raft frame, 0x000-0x003 (REF) and 0x040-0x07F (the rest) */
		{ RAFT_FILTER_REF_ID, RAFT_FILTER_REF_MASK }, { RAFT_FILTER_ID, RAFT_FILTER_MASK },
	};
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (fd < 0) {
		perror("socket(PF_CAN)");
		return -1;
	}
	addr.can_ifindex = if_nametoindex(name);
	if (addr.can_ifindex == 0) {
		fprintf(stderr, "no interface %s\n", name);
		return -1;
	}
	/* Like can_add_rx_filter() on the STM32s: only the IDs we consume. */
	setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(rfilter));
	/* Lab 5: the kernel stamps each frame as the driver hands it up: a REF's receive time
	 * without our thread's wake-up latency */
	int one = 1;

	setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one));
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind can");
		return -1;
	}
	return fd;
}

/* One seq counter per ID we send, shared by both copies. */
enum { TX_CTRL, TX_BLINK, TX_HB, TX_FAULT, TX_COUNT };
static atomic_uint tx_seq[TX_COUNT];

static int send_both(uint32_t id, int slot, uint8_t data[8], bool set_crc)
{
	struct can_frame f = { .can_id = id, .can_dlc = 8 };
	int ok = 0;

	data[0] = (data[0] & ~SEQ_MASK) | (atomic_fetch_add(&tx_seq[slot], 1) & SEQ_MASK);
	if (set_crc) {
		/* CTRL CRC: b0..b6 with b0's flag nibble masked (RELAY sets a flag in transit). */
		uint8_t c[7];

		memcpy(c, data, 7);
		c[0] &= SEQ_MASK;
		data[7] = crc8(c, 7);
	}
	memcpy(f.data, data, 8);

	for (int b = 0; b < BUS_COUNT; b++) {
		gpio_toggle(tx_tp[b]);
		/* MSG_DONTWAIT: a pulled jumper fills that queue; never stall the other bus. */
		if (send(can_fd[b], &f, sizeof(f), MSG_DONTWAIT) == sizeof(f)) {
			ok++;
		} else {
			atomic_fetch_add(&tx_fail[b], 1);
		}
		/* Pi stage ends when the first CAN write returns (TA clarification, measurement #1). */
		if (b == 0 && id == ID_CTRL) {
			gpio_toggle(GPIO_CMD_TX);
		}
	}
	return ok > 0 ? 0 : -1;
}

/* ------------------------------ shared state ---------------------------- */

static pthread_mutex_t st_lock = PTHREAD_MUTEX_INITIALIZER;

/* Guarded by st_lock. */
static int64_t hb_seen[ZONE_N3 + 1][BUS_COUNT]; /* -1 never */
static int64_t hb_direct[ZONE_N3 + 1][BUS_COUNT]; /* Lab 5: own copies only, not relayed; -1 never */
static struct route_table routes;                 /* Lab 5: rebuilt from getNetwork() every 10 ms */
static uint8_t peer_state[ZONE_N3 + 1];
static int64_t remote_since[ZONE_N3 + 1];       /* -1 none */
static bool zone_ok;
static uint8_t last_reason;
static bool test_latched;
static int64_t last_udp_ms = -1;
static bool degraded[BUS_COUNT];
static uint8_t blink_req = BLINK_OFF; /* driver's turn request (off/left/right) */
static int8_t force_value;

static atomic_uint udp_count, ctrl_count, force_count, relayed_count, relayed_tx;
static atomic_bool inject_bad;
static atomic_int hz_state; /* Lab 5: the blinker state the HZ lamp shows */

static int udp_fd = -1;
static struct sockaddr_in proxy_addr; /* where the last UDP came from */
static bool proxy_known;

static int64_t hb_age(int zone, int bus)
{
	return (hb_seen[zone][bus] < 0) ? INT64_MAX : now_ms() - hb_seen[zone][bus];
}

/* Caller holds st_lock */
static int64_t hb_direct_age(int zone, int bus)
{
	return (hb_direct[zone][bus] < 0) ? INT64_MAX : now_ms() - hb_direct[zone][bus];
}

/* Caller holds st_lock. */
static void set_ok(bool ok)
{
	zone_ok = ok;
	gpio_put(GPIO_ERR, !ok);
}

/* Caller holds st_lock. */
static void fail(uint8_t reason)
{
	set_ok(false);
	last_reason = reason;
	if (reason == FAULT_TEST_BUTTON) {
		test_latched = true;
	}
}

static int64_t last_fault_tx = -FAULT_MIN_MS;

static void send_fault(uint8_t reason)
{
	uint8_t d[8] = { [1] = ZONE_N1, [2] = reason };
	int64_t wait = last_fault_tx + FAULT_MIN_MS - now_ms();

	if (wait > 0) {
		usleep((useconds_t)wait * 1000);
	}
	send_both(ID_FAULT, TX_FAULT, d, false);
	last_fault_tx = now_ms();
}

/* ------------------------------ UDP_RX thread --------------------------- */

static int16_t steer_last;
static atomic_int raw_steer, raw_thr, raw_brk; /* last proxy values, for calibration */

static void build_ctrl(const DIJOYSTATE2_t *s, uint8_t d[8])
{
	int steer = s->lX;
	int thr = s->lY;
	int brk = s->lRz;
	uint8_t buttons = 0;

	atomic_store(&raw_steer, steer);
	atomic_store(&raw_thr, thr);
	atomic_store(&raw_brk, brk);

	/* Out-of-range raw input stays out of range on CTRL: the zone faults. */
	if (steer < STEER_MIN || steer > STEER_MAX) {
		steer = -32768;
	}
	uint8_t thr_pct = (thr < 0 || thr > THROTTLE_MAX) ? 0xFF : (uint8_t)(thr * 100 / THROTTLE_MAX);
	uint8_t brk_pct;
	static int brk_rest = BRAKE_FULL; /* the lowest lRz seen: the released pedal */

	if (brk >= 0 && brk < brk_rest) {
		brk_rest = brk;
	}
	int brk_zero = brk_rest + (BRAKE_FULL - brk_rest) * BRAKE_DEADBAND_PCT / 100;

	if (brk < 0 || brk > BRAKE_FULL) {
		brk_pct = 0xFF;
	} else if (brk <= brk_zero || brk_zero >= BRAKE_FULL) {
		brk_pct = 0;
	} else {
		brk_pct = (uint8_t)((brk - brk_zero) * 100 / (BRAKE_FULL - brk_zero));
	}

	if (s->rgbButtons[WHEEL_BTN_SELF_TEST]) {
		buttons |= BTN_SELF_TEST;
	}
	if (s->rgbButtons[WHEEL_BTN_RIGHT]) {
		buttons |= BTN_RIGHT;
	}
	if (s->rgbButtons[WHEEL_BTN_LEFT]) {
		buttons |= BTN_LEFT;
	}

	if (atomic_exchange(&inject_bad, false)) {
		steer = -32768; /* 0x8000: outside the team's steer range, CRC still valid */
		printf(">>> injected out-of-range CTRL (steer 0x8000)\n");
	}

	memset(d, 0, 8);
	d[1] = (uint8_t)(steer & 0xFF);
	d[2] = (uint8_t)((steer >> 8) & 0xFF);
	d[3] = thr_pct;
	d[4] = brk_pct;
	d[5] = buttons;
	steer_last = (int16_t)steer;
}

static void handle_buttons(uint8_t buttons, int16_t steer)
{
	static uint8_t prev;
	static bool armed;
	static int64_t clear_press = -1;
	uint8_t pressed = buttons & ~prev;
	int64_t t = now_ms();
	bool raise = false;

	prev = buttons;

	pthread_mutex_lock(&st_lock);
	/* Turn signals: press toggles, steering past TURN_ARM then back to center cancels. */
	if (pressed & BTN_LEFT) {
		blink_req = (blink_req == BLINK_LEFT) ? BLINK_OFF : BLINK_LEFT;
		armed = false;
	}
	if (pressed & BTN_RIGHT) {
		blink_req = (blink_req == BLINK_RIGHT) ? BLINK_OFF : BLINK_RIGHT;
		armed = false;
	}
	if ((blink_req == BLINK_LEFT && steer < -TURN_ARM) ||
	    (blink_req == BLINK_RIGHT && steer > TURN_ARM)) {
		armed = true;
	}
	if (armed && steer > -TURN_CENTER && steer < TURN_CENTER) {
		blink_req = BLINK_OFF;
		armed = false;
	}

	/* Wheel test button: single press = cockpit zone fails, double press = clear. */
	if (pressed & BTN_SELF_TEST) {
		if (!test_latched) {
			fail(FAULT_TEST_BUTTON);
			raise = true;
			clear_press = -1;
		} else if (clear_press >= 0 && t - clear_press <= DOUBLE_MS) {
			test_latched = false;
			clear_press = -1;
		} else {
			clear_press = t;
		}
	}
	pthread_mutex_unlock(&st_lock);

	if (raise) {
		send_fault(FAULT_TEST_BUTTON);
	}
}

static void *udp_thread(void *arg)
{
	uint8_t buf[sizeof(DIJOYSTATE2_t) + 4];
	DIJOYSTATE2_t s;

	(void)arg;
	while (1) {
		struct sockaddr_in from;
		socklen_t flen = sizeof(from);
		ssize_t n = recvfrom(udp_fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &flen);

		if (n < (ssize_t)(4 + sizeof(s) - 4)) {
			continue;
		}
		gpio_toggle(GPIO_UDP_RX);

		memset(&s, 0, sizeof(s));
		memcpy(&s, buf + 4, (size_t)n - 4 < sizeof(s) ? (size_t)n - 4 : sizeof(s));

		uint8_t d[8];

		build_ctrl(&s, d);
		send_both(ID_CTRL, TX_CTRL, d, true); /* toggles CMD_TX after the first CAN write */
		atomic_fetch_add(&udp_count, 1);
		atomic_fetch_add(&ctrl_count, 1);

		pthread_mutex_lock(&st_lock);
		last_udp_ms = now_ms();
		proxy_addr = from;
		proxy_addr.sin_port = htons(S_PORT);
		proxy_known = true;
		pthread_mutex_unlock(&st_lock);

		handle_buttons(d[5], steer_last);
	}
	return NULL;
}

/* ------------------------------ CAN_RX thread --------------------------- */

struct seen {
	int64_t at;
	uint8_t seq;
};

static int slot_of(const struct can_frame *f)
{
	switch (f->can_id) {
	case ID_HB_N2: return 0;
	case ID_HB_N3: return 1;
	case ID_FORCE: return 2;
	case ID_FAULT:
		if (f->data[1] >= ZONE_N1 && f->data[1] <= ZONE_N3) {
			return 2 + f->data[1]; /* 3..5 */
		}
		return -1;
	default: return -1;
	}
}

/*
 * Lab 5 RELAY (step 6, like src_lab5/relay.c): forward a frame from an STM32, heard on bus
 * b, to the other bus when the routing table (from getNetwork()) or what we hear says the
 * other STM32 can't hear it any other way: checkoff step 9, the Pi bridging N2 and N3.
 * A relayed REF gets the first hop and its time here added to its master time.
 */
static void relay_maybe(const struct can_frame *f, int b, int64_t rx_us)
{
	bool raft = raft_is_frame((uint16_t)f->can_id);
	uint8_t flag = raft ? RAFT_FLAG_RELAYED : FLAG_RELAYED;
	int sender = -1;

	if (f->data[0] & flag) {
		return; /* never relay a relayed copy */
	}
	if (raft) {
		sender = RAFT_ID_SENDER(f->can_id);
	} else if (f->can_id == ID_HB_N2 || f->can_id == ID_HB_N3) {
		sender = (int)(f->can_id - ID_HB_N1) + ZONE_N1;
	} else if (f->can_id == ID_FAULT) {
		sender = f->data[1];
	}
	if (sender < ZONE_N2 || sender > ZONE_N3) {
		return;
	}

	pthread_mutex_lock(&st_lock);
	bool go = routes.relay[sender][b];

	if (!go) {
		struct netwatch_obs o = { 0 };

		for (int z = ZONE_N2; z <= ZONE_N3; z++) {
			int64_t limit = (z == sender) ? RELAY_AFTER_MS : HB_TIMEOUT_MS;

			for (int x = 0; x < BUS_COUNT; x++) {
				o.direct[z][x] = hb_direct_age(z, x) < limit;
				o.fresh[z][x] = o.direct[z][x];
			}
		}
		go = route_relay_observed(&o, ZONE_N1, (uint8_t)sender, (uint8_t)b);
	}
	pthread_mutex_unlock(&st_lock);
	if (!go) {
		return;
	}

	struct can_frame out = *f;

	out.data[0] |= flag;
	if (raft && RAFT_ID_TYPE(out.can_id) == RAFT_ID_REF) {
		pthread_mutex_lock(&time_lock);
		int64_t t = timesync_relay_master_us((uint8_t)sender, ref_time(out.data), rx_us, now_us());

		pthread_mutex_unlock(&time_lock);
		put_ref_time(out.data, t);
	}
	gpio_toggle(tx_tp[1 - b]);
	if (send(can_fd[1 - b], &out, sizeof(out), MSG_DONTWAIT) == sizeof(out)) {
		atomic_fetch_add(&relayed_tx, 1);
	} else {
		atomic_fetch_add(&tx_fail[1 - b], 1);
	}
}

/* Read one frame and the kernel's receive time, converted to CLOCK_MONOTONIC µs */
static bool read_frame(int fd, struct can_frame *f, int64_t *rx_us)
{
	char ctrl[CMSG_SPACE(sizeof(struct timespec))];
	struct iovec iov = { .iov_base = f, .iov_len = sizeof(*f) };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1, .msg_control = ctrl,
			      .msg_controllen = sizeof(ctrl) };

	if (recvmsg(fd, &msg, 0) != sizeof(*f)) {
		return false;
	}
	int64_t mono = now_us();

	*rx_us = mono;
	for (struct cmsghdr *c = CMSG_FIRSTHDR(&msg); c != NULL; c = CMSG_NXTHDR(&msg, c)) {
		if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
			struct timespec k, real;

			memcpy(&k, CMSG_DATA(c), sizeof(k));
			clock_gettime(CLOCK_REALTIME, &real);
			int64_t ago = ((int64_t)real.tv_sec - k.tv_sec) * 1000000 + (real.tv_nsec - k.tv_nsec) / 1000;

			if (ago >= 0 && ago < 100000) {
				*rx_us = mono - ago;
			}
		}
	}
	return true;
}

static void *can_rx_thread(void *arg)
{
	struct pollfd pfd[BUS_COUNT];
	struct seen seen[6];

	(void)arg;
	for (int i = 0; i < 6; i++) {
		seen[i].at = -1;
	}
	for (int b = 0; b < BUS_COUNT; b++) {
		pfd[b].fd = can_fd[b];
		pfd[b].events = POLLIN;
	}

	while (1) {
		if (poll(pfd, BUS_COUNT, 1000) <= 0) {
			continue;
		}
		for (int b = 0; b < BUS_COUNT; b++) {
			struct can_frame f;

			int64_t rx_us;

			if (!(pfd[b].revents & POLLIN) || !read_frame(can_fd[b], &f, &rx_us)) {
				continue;
			}
			if ((f.can_id & CAN_EFF_FLAG) || f.can_dlc != 8) {
				continue;
			}
			/* Lab 5: Raft frames go to the agent. Byte 0 is a term, not a seq, so
			 * they skip the dedup below; raft.c is fine seeing both bus copies. */
			if (raft_is_frame((uint16_t)f.can_id)) {
				static int last_ref_key = -1;

				if (RAFT_ID_TYPE(f.can_id) == RAFT_ID_REF) {
					int key = RAFT_ID_SENDER(f.can_id) << 8 | f.data[1];

					if (key != last_ref_key) {
						gpio_toggle(GPIO_REF_RX); /* checkoff step 9: once per REF */
						last_ref_key = key;
					}
				}
				pthread_mutex_lock(&raft_lock);
				bool fresh_ref = raft_on_frame((uint16_t)f.can_id, f.data, now_ms());
				pthread_mutex_unlock(&raft_lock);
				if (fresh_ref) {
					pthread_mutex_lock(&time_lock);
					uint32_t before = timesync_get_stats().handovers;

					timesync_on_ref(RAFT_ID_SENDER(f.can_id), ref_time(f.data), rx_us);
					struct timesync_stats ts = timesync_get_stats();

					pthread_mutex_unlock(&time_lock);
					if (ts.handovers != before) {
						printf("time N1: master N%u -> N%u, clocks %+lld us apart at its first REF\n",
						       ts.prev_source, RAFT_ID_SENDER(f.can_id),
						       (long long)ts.handover_step_us);
					}
				}
				relay_maybe(&f, b, rx_us);
				continue;
			}
			int slot = slot_of(&f);

			if (slot < 0) {
				continue;
			}
			int64_t t = now_ms();
			bool dup;
			bool raise_remote = false;

			pthread_mutex_lock(&st_lock);
			if (f.can_id == ID_HB_N2 || f.can_id == ID_HB_N3) {
				hb_seen[f.can_id - ID_HB_N1 + ZONE_N1][b] = t; /* every copy */
				if (!(f.data[0] & FLAG_RELAYED)) {
					hb_direct[f.can_id - ID_HB_N1 + ZONE_N1][b] = t;
				}
			}
			dup = (seen[slot].at >= 0 && seen[slot].seq == (f.data[0] & SEQ_MASK) &&
			       t - seen[slot].at < DUP_WINDOW_MS);
			if (!dup) {
				seen[slot].at = t;
				seen[slot].seq = f.data[0] & SEQ_MASK;
				if (f.data[0] & FLAG_RELAYED) {
					atomic_fetch_add(&relayed_count, 1);
				}

				if (f.can_id == ID_HB_N2 || f.can_id == ID_HB_N3) {
					int z = f.can_id - ID_HB_N1 + ZONE_N1;

					peer_state[z] = f.data[1];
					/* Byte 3: that zone's own fault is gone (see zone_state.h). */
					if (f.data[3] == 0 && remote_since[z] >= 0 &&
					    t - remote_since[z] >= REMOTE_CLEAR_MS) {
						remote_since[z] = -1;
					}
				} else if (f.can_id == ID_FAULT) {
					int z = f.data[1];

					if (z != ZONE_N1) {
						remote_since[z] = t;
						fail(FAULT_REMOTE);
						raise_remote = true;
					}
				} else if (f.can_id == ID_FORCE) {
					force_value = (int8_t)f.data[1];
					atomic_fetch_add(&force_count, 1);
				}
			}
			struct sockaddr_in dst = proxy_addr;
			bool have_proxy = proxy_known;
			int8_t fv = force_value;
			pthread_mutex_unlock(&st_lock);

			(void)raise_remote;
			relay_maybe(&f, b, rx_us);
			if (!dup && f.can_id == ID_FORCE && have_proxy) {
				/* Dummy force feedback back to the proxy, like Lab 2's send_force. */
				sendto(udp_fd, &fv, 1, 0, (struct sockaddr *)&dst, sizeof(dst));
			}
		}
	}
	return NULL;
}

/* ------------------------------ Raft (Lab 5) ----------------------------- */
/*
 * The Pi's side of shared/raft.c, like src_lab5/raft_node.c on the STM32s:
 * frames out (raft_platform_send), frames in (can_rx_thread), the tick, and the
 * LEADER and SYNC LEDs. The protocol itself is all in shared/raft.c.
 */

/* Called by raft.c, with raft_lock held. Not send_both(): byte 0 is the term, not a seq. */
void raft_platform_send(uint16_t id, uint8_t data[8])
{
	struct can_frame f = { .can_id = id, .can_dlc = 8 };

	memcpy(f.data, data, 8);
	for (int b = 0; b < BUS_COUNT; b++) {
		if (RAFT_ID_TYPE(id) == RAFT_ID_REF) {
			/* The master time, global_gettime() in µs (48-bit LE), stamped as late as
			 * possible: once per bus, just before that copy is sent */
			put_ref_time(f.data, global_gettime());
		}
		gpio_toggle(tx_tp[b]);
		/* MSG_DONTWAIT: a pulled jumper fills that queue; never stall the other bus. */
		if (send(can_fd[b], &f, sizeof(f), MSG_DONTWAIT) != sizeof(f)) {
			atomic_fetch_add(&tx_fail[b], 1);
		}
	}
}

/* Step 6, every 10 ms: what we hear, for link watching, and the routing table from the
 * committed graph for relay_maybe(). Caller holds raft_lock. */
static void watch_links(int64_t t)
{
	struct netwatch_obs o = { 0 };
	struct route_table r;

	pthread_mutex_lock(&st_lock);
	for (int z = ZONE_N2; z <= ZONE_N3; z++) {
		for (int b = 0; b < BUS_COUNT; b++) {
			o.direct[z][b] = hb_direct_age(z, b) < HB_TIMEOUT_MS;
			o.fresh[z][b] = hb_direct_age(z, b) < NETWATCH_FRESH_MS;
			o.alive[z] = o.alive[z] || hb_age(z, b) < HB_TIMEOUT_MS;
		}
	}
	pthread_mutex_unlock(&st_lock);
	o.alive[ZONE_N1] = true;
	netwatch_tick(&o, t);

	struct network g = getNetwork();

	route_build(&g, ZONE_N1, &r);
	pthread_mutex_lock(&st_lock);
	routes = r;
	pthread_mutex_unlock(&st_lock);
}

/* RAFT: raft_tick() every RAFT_TICK_MS (raft.c decides what's due), then the LEDs */
static void *raft_thread(void *arg)
{
	int64_t next = now_ms();
	unsigned int ticks = 0;

	(void)arg;
	while (1) {
		int64_t t = now_ms();

		pthread_mutex_lock(&raft_lock);
		raft_tick(t);
		bool leader = raft_get_role() == RAFT_LEADER;
		bool sync = raft_in_sync(t);

		if (++ticks % 2 == 0) {
			watch_links(t);
		}
		pthread_mutex_unlock(&raft_lock);
		pthread_mutex_lock(&time_lock);
		timesync_set_master(leader); /* the leader's clock is global time */
		pthread_mutex_unlock(&time_lock);

		gpio_put(GPIO_LEADER, leader);
		gpio_put(GPIO_SYNC, sync);

		next += RAFT_TICK_MS;
		sleep_until_ms(next);
	}
	return NULL;
}

/* ------------------- PPS and the HZ lamp (Lab 5, Part 6) ------------------ */

struct pps_line {
	int64_t global_us, local_us, since_ref_us;
	struct timesync_stats st;
};
static pthread_mutex_t pps_lock = PTHREAD_MUTEX_INITIALIZER;
static struct pps_line pps_last;
static bool pps_new;

/* PPS (6.4): a rising edge on BCM 22 at every whole second of global time */
static void *pps_thread(void *arg)
{
	(void)arg;
	while (1) {
		int64_t next = (global_gettime() / 1000000 + 1) * 1000000;

		wait_global(next);
		gpio_put(GPIO_PPS, true);

		struct pps_line p;

		pthread_mutex_lock(&time_lock);
		p.local_us = now_us();
		p.global_us = timesync_global_us(p.local_us);
		p.st = timesync_get_stats();
		p.since_ref_us = timesync_correction_since_ref_us(p.local_us);
		pthread_mutex_unlock(&time_lock);
		pthread_mutex_lock(&pps_lock);
		pps_last = p;
		pps_new = true;
		pthread_mutex_unlock(&pps_lock);

		sleep_until_us(p.local_us + 100000);
		gpio_put(GPIO_PPS, false);
	}
	return NULL;
}

/* HZ (BCM 23): the hazard lamp, switched at the same global-time edges as FL and RL */
static void *hz_thread(void *arg)
{
	(void)arg;
	while (1) {
		int64_t half = 1000LL * BLINK_HAZARD_HALF_MS;
		int64_t g = global_gettime();
		bool on = atomic_load(&hz_state) == BLINK_HAZARD && (g / half) % 2 == 0;

		gpio_put(GPIO_HZ, on);
		int64_t edge = (g / half + 1) * half;

		if (edge - g > 20000) {
			sleep_until_us(now_us() + 20000); /* re-check the state every 20 ms */
		} else {
			wait_global(edge);
		}
	}
	return NULL;
}

static void fmt_us(char *buf, size_t len, int64_t us)
{
	int64_t a = us < 0 ? -us : us;

	snprintf(buf, len, "%s%lld.%06lld", us < 0 ? "-" : "", (long long)(a / 1000000),
		 (long long)(a % 1000000));
}

/* One line per PPS edge (6.4), printed by main */
static void print_pps(void)
{
	pthread_mutex_lock(&pps_lock);
	struct pps_line p = pps_last;
	bool fresh = pps_new;

	pps_new = false;
	pthread_mutex_unlock(&pps_lock);
	if (!fresh) {
		return;
	}
	char g[24], l[24];
	int32_t ppb_abs = p.st.ppb < 0 ? -p.st.ppb : p.st.ppb;
	int64_t ratio = 1000000000LL + p.st.ppb;

	fmt_us(g, sizeof(g), p.global_us);
	fmt_us(l, sizeof(l), p.local_us);
	printf("PPS N1 | global %s s | local %s s | drift ratio %lld.%09lld (%c%d.%03d ppm) | %s %+lld us | "
	       "last step %+lld us | %s N%u\n",
	       g, l, (long long)(ratio / 1000000000), (long long)(ratio % 1000000000), p.st.ppb < 0 ? '-' : '+',
	       (int)(ppb_abs / 1000), (int)(ppb_abs % 1000), p.st.master ? "since last sync" : "since REF",
	       (long long)p.since_ref_us, (long long)p.st.last_step_us,
	       p.st.master ? "master" : "synced to", p.st.master ? ZONE_N1 : p.st.source);
}

/* ---------- PERIODIC thread: HB_MON every 10 ms, HB and BLINK every 20 ms ---------- */

static void *periodic_thread(void *arg)
{
	int64_t next = now_ms();
	uint8_t blink_sent = 0xFF;
	bool phase = false;
	int64_t last_toggle = 0;
	unsigned int tick = 0;

	(void)arg;
	while (1) {
		int64_t t = now_ms();
		uint8_t hb[8] = { 0 };
		uint8_t blink_state;

		pthread_mutex_lock(&st_lock);

		/* HB_MON: DEG_X when neither N2 nor N3 heard on X for 60 ms (R4.7). */
		for (int b = 0; b < BUS_COUNT; b++) {
			degraded[b] = hb_age(ZONE_N2, b) >= HB_TIMEOUT_MS &&
				      hb_age(ZONE_N3, b) >= HB_TIMEOUT_MS;
		}
		gpio_put(GPIO_DEG_A, degraded[BUS_A]);
		gpio_put(GPIO_DEG_B, degraded[BUS_B]);

		bool peers_alive = true;

		for (int z = ZONE_N2; z <= ZONE_N3; z++) {
			if (hb_age(z, BUS_A) >= HB_TIMEOUT_MS && hb_age(z, BUS_B) >= HB_TIMEOUT_MS) {
				peers_alive = false;
			}
		}
		bool udp_fresh = last_udp_ms >= 0 && t - last_udp_ms < UDP_TIMEOUT_MS;
		bool remote = remote_since[ZONE_N2] >= 0 || remote_since[ZONE_N3] >= 0;

		if (zone_ok) {
			if (!peers_alive) {
				fail(FAULT_HB_LOSS);
			} else if (!udp_fresh) {
				fail(FAULT_INPUT_LOST);
			}
		} else if (peers_alive && udp_fresh && !test_latched && !remote) {
			set_ok(true);
		}

		/* HB_N1 */
		hb[1] = zone_ok ? HB_STATE_OK : HB_STATE_ERROR;
		hb[2] = (degraded[BUS_A] ? 0 : HB_HEALTH_A) | (degraded[BUS_B] ? 0 : HB_HEALTH_B);
		hb[3] = test_latched ? 1 : 0; /* our own local fault */

		/* Blinker master: hazards whenever any zone is in error. */
		bool any_error = !zone_ok || peer_state[ZONE_N2] == HB_STATE_ERROR ||
				 peer_state[ZONE_N3] == HB_STATE_ERROR;

		blink_state = any_error ? BLINK_HAZARD : blink_req;
		pthread_mutex_unlock(&st_lock);

		/* HB_N1 and BLINK keep their 20 ms catalog period: every other tick. */
		if (tick++ % (PERIOD_MS / MON_PERIOD_MS) != 0) {
			next += MON_PERIOD_MS;
			sleep_until_ms(next);
			continue;
		}

		send_both(ID_HB_N1, TX_HB, hb, false);

		/* Lab 5: BLINK says which lamps blink, on every change and again every 500 ms
		 * (so a lost frame heals); the lamps time themselves from global time. b2 still
		 * carries the phase for Lab 3's sake, from global time too. */
		if (blink_state != blink_sent || t - last_toggle >= BLINK_HALF_MS) {
			int64_t half = 1000LL * (blink_state == BLINK_HAZARD ? BLINK_HAZARD_HALF_MS : BLINK_HALF_MS);

			phase = blink_state != BLINK_OFF && (global_gettime() / half) % 2 == 0;
			uint8_t bl[8] = { [1] = blink_state, [2] = phase ? 1 : 0 };

			send_both(ID_BLINK, TX_BLINK, bl, false);
			blink_sent = blink_state;
			last_toggle = t;
		}
		atomic_store(&hz_state, blink_state);

		next += MON_PERIOD_MS;
		sleep_until_ms(next);
	}
	return NULL;
}

/* ---------------------------- console (Lab 5) ---------------------------- */
/*
 * The STM32 shell's Raft commands (src_lab5/net_shell.c, raft_node.c), one per line:
 *   net show | net node up|down <1-3> | net link up|down <1-3> <A|B> | net cockpit ok|error
 *   raft status | raft log [count|all] | raft config
 */

#define COMMIT_WAIT_MS   500 /* the leader resends every 100 ms, so several tries */
#define COMMIT_POLL_MS   10
#define LOG_SHOW_DEFAULT 10  /* entries `raft log` prints without a count */

static void console_help(void)
{
	printf("commands: b = inject bad CTRL | q = quit | net show | net %s\n"
	       "          raft status | raft log [count|all] | raft config\n"
	       "          time status | time delay [<node> <us>]\n", NET_EVENT_USAGE);
}

/* time status | time delay [<node> <us>] (the same as the STM32 shell) */
static void console_time(int argc, char **argv)
{
	char line[200], g[24];

	if (argc >= 2 && strcmp(argv[1], "delay") == 0) {
		if (argc >= 4) {
			int node = atoi(argv[2]);

			if (node >= ZONE_N2 && node <= ZONE_N3) {
				pthread_mutex_lock(&time_lock);
				timesync_set_delay_us((uint8_t)node, atoi(argv[3]));
				pthread_mutex_unlock(&time_lock);
			}
		}
		for (int n = ZONE_N2; n <= ZONE_N3; n++) {
			printf("delay from N%d: %d us\n", n, (int)timesync_delay_us((uint8_t)n));
		}
		return;
	}
	pthread_mutex_lock(&time_lock);
	int64_t l = now_us();

	timesync_format(line, sizeof(line), l);
	fmt_us(g, sizeof(g), timesync_global_us(l));
	pthread_mutex_unlock(&time_lock);
	printf("N1 global %s s | %s\n", g, line);
}

/*
 * The cockpit's display of the log (checkoff step 7): every committed entry, once, in
 * order. Entries that reached the Pi already committed (the leader held them while it was
 * away) are marked "replayed"; the rest are live.
 */
static uint16_t shown;

static void show_new_entries(void)
{
	pthread_mutex_lock(&raft_lock);
	uint16_t commit = raft_log_commit_index();
	uint16_t replayed = raft_replayed_through();

	if (commit < shown) {
		shown = 0; /* our log was reset */
	}
	while (shown < commit) {
		char line[64];

		shown++;
		raft_log_format(shown, line, sizeof(line));
		printf("event %s%s\n", line, shown <= replayed ? "  [replayed]" : "  [live]");
	}
	pthread_mutex_unlock(&raft_lock);
}

/* `raft status`, and the last line of the status every second */
static void print_raft_status(void)
{
	char line[128];

	char route[48];

	pthread_mutex_lock(&raft_lock);
	raft_status_format(line, sizeof(line), now_ms());
	pthread_mutex_unlock(&raft_lock);
	pthread_mutex_lock(&st_lock);
	route_format(&routes, route, sizeof(route));
	pthread_mutex_unlock(&st_lock);
	printf("raft %s | %s\n", line, route);
}

static void console_net_show(void)
{
	char str[96];

	pthread_mutex_lock(&raft_lock);
	struct network net = getNetwork();
	pthread_mutex_unlock(&raft_lock);
	netstate_format(&net, str, sizeof(str));
	printf("%s\n", str);
}

/* The first entry after index `after` holding event, once it commits here; 0 if it hasn't
 * within COMMIT_WAIT_MS */
static uint16_t wait_committed(uint8_t event, uint16_t after)
{
	for (int waited = 0;; waited += COMMIT_POLL_MS) {
		pthread_mutex_lock(&raft_lock);
		uint16_t index = raft_log_find_committed(event, after);
		pthread_mutex_unlock(&raft_lock);
		if (index != 0 || waited >= COMMIT_WAIT_MS) {
			return index;
		}
		usleep(COMMIT_POLL_MS * 1000);
	}
}

/* net node|link|cockpit ...: argv[0] is the subcommand. The same steps as net_shell.c. */
static void console_submit(int argc, char **argv)
{
	struct net_event ev;
	char name[24];

	if (net_event_parse(argc, argv, &ev) < 0) {
		printf("usage: net %s\n", NET_EVENT_USAGE);
		return;
	}
	uint8_t event = net_event_encode(&ev);

	net_event_format(&ev, name, sizeof(name));
	pthread_mutex_lock(&raft_lock);
	uint16_t committed = raft_log_commit_index();
	int ret = raft_submit(event);
	uint8_t leader = raft_get_leader();
	pthread_mutex_unlock(&raft_lock);

	if (ret < 0) {
		printf("%s: not submitted, %s\n", name,
		       ret == -EAGAIN ? "no leader known" : (ret == -ENOSPC ? "log full" : "bad event"));
		return;
	}
	if (ret > 0) {
		printf("%s: appended as entry %d (this node leads)\n", name, ret);
	} else {
		printf("%s: sent to leader N%u\n", name, leader);
	}
	/* The leader knows the index. A follower looks past what was committed when it sent. */
	uint16_t index = wait_committed(event, ret > 0 ? (uint16_t)(ret - 1) : committed);

	if (index == 0) {
		printf("not committed after %d ms: is a majority up?\n", COMMIT_WAIT_MS);
		return;
	}
	printf("ok: committed as entry %u\n", index);
	console_net_show();
}

/* raft log [count|all]: the newest entries, the lock taken per entry as on the STM32s */
static void console_log(int argc, char **argv)
{
	int count = LOG_SHOW_DEFAULT;

	if (argc > 1) {
		count = strcmp(argv[1], "all") == 0 ? RAFT_LOG_SIZE : atoi(argv[1]);
		if (count <= 0) {
			printf("usage: raft log [count|all]\n");
			return;
		}
	}
	pthread_mutex_lock(&raft_lock);
	uint16_t last = raft_log_last_index();
	uint16_t commit = raft_log_commit_index();
	uint8_t term = raft_get_term();
	pthread_mutex_unlock(&raft_lock);

	printf("N%d log: %u entries, commit %u, term %u\n", RAFT_NODE_COCKPIT, last, commit, term);
	for (int i = last > count ? last - count + 1 : 1; i <= last; i++) {
		char line[64];

		pthread_mutex_lock(&raft_lock);
		int ret = raft_log_format((uint16_t)i, line, sizeof(line));
		pthread_mutex_unlock(&raft_lock);
		if (ret < 0) {
			break; /* replaced by the leader's entries meanwhile */
		}
		printf("%s\n", line);
	}
}

static void console_config(void)
{
	char line[192];

	raft_config_format(line, sizeof(line));
	printf("node %d | %s\n", RAFT_NODE_COCKPIT, line);
}

/* One console line. Returns false for q. */
static bool console_command(char *line)
{
	char *argv[8];
	char *save;
	int argc = 0;

	for (char *tok = strtok_r(line, " \t\r\n", &save); tok != NULL && argc < 8;
	     tok = strtok_r(NULL, " \t\r\n", &save)) {
		argv[argc++] = tok;
	}
	if (argc == 0) {
		return true;
	}
	bool is_raft = argc >= 2 && strcmp(argv[0], "raft") == 0;

	if (strcmp(argv[0], "q") == 0) {
		return false;
	} else if (strcmp(argv[0], "b") == 0) {
		atomic_store(&inject_bad, true);
	} else if (argc >= 2 && strcmp(argv[0], "net") == 0) {
		if (strcmp(argv[1], "show") == 0) {
			console_net_show();
		} else {
			console_submit(argc - 1, argv + 1);
		}
	} else if (is_raft && strcmp(argv[1], "status") == 0) {
		print_raft_status();
	} else if (is_raft && strcmp(argv[1], "log") == 0) {
		console_log(argc - 1, argv + 1);
	} else if (is_raft && strcmp(argv[1], "config") == 0) {
		console_config();
	} else if (strcmp(argv[0], "time") == 0) {
		console_time(argc, argv);
	} else {
		console_help();
	}
	return true;
}

/* --------------------------------- main -------------------------------- */

static void on_sigusr1(int sig)
{
	(void)sig;
	atomic_store(&inject_bad, true);
}

static const char *reason_name(uint8_t r)
{
	static const char *const names[] = { "-", "test button", "bad CTRL", "HB loss",
					     "remote FAULT", "proxy stopped" };
	return r < sizeof(names) / sizeof(names[0]) ? names[r] : "?";
}

static void start_thread(void *(*fn)(void *), int fifo_prio, bool rt)
{
	pthread_t tid;
	pthread_attr_t attr;
	struct sched_param sp = { .sched_priority = fifo_prio };

	pthread_attr_init(&attr);
	if (rt) {
		pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
		pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
		pthread_attr_setschedparam(&attr, &sp);
	}
	if (pthread_create(&tid, &attr, fn, NULL) != 0) {
		pthread_attr_destroy(&attr);
		pthread_attr_init(&attr); /* no permission for SCHED_FIFO: run normal */
		pthread_create(&tid, &attr, fn, NULL);
	}
	pthread_attr_destroy(&attr);
}

int main(void)
{
	bool rt = (geteuid() == 0);

	setvbuf(stdout, NULL, _IOLBF, 0);
	setvbuf(stdin, NULL, _IONBF, 0); /* no line waits in stdio's buffer where poll() can't see it */
	signal(SIGUSR1, on_sigusr1);
	for (int z = 0; z <= ZONE_N3; z++) {
		remote_since[z] = -1;
		peer_state[z] = HB_STATE_ERROR;
		for (int b = 0; b < BUS_COUNT; b++) {
			hb_seen[z][b] = -1;
			hb_direct[z][b] = -1;
		}
	}

	if (gpio_init() != 0) {
		return 1;
	}
	for (int b = 0; b < BUS_COUNT; b++) {
		if (find_iface(b, can_name[b]) != 0) {
			fprintf(stderr, "no CAN interface on spi0.%d (bus %c): is it up?\n", b, 'A' + b);
			return 1;
		}
		can_fd[b] = can_open(can_name[b]);
		if (can_fd[b] < 0) {
			return 1;
		}
	}
	printf("bus A = %s (spi0.0, CE0), bus B = %s (spi0.1, CE1)\n", can_name[BUS_A],
	       can_name[BUS_B]);

	struct sockaddr_in local = { .sin_family = AF_INET, .sin_port = htons(R_PORT),
				     .sin_addr.s_addr = htonl(INADDR_ANY) };

	udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_fd < 0 || bind(udp_fd, (struct sockaddr *)&local, sizeof(local)) < 0) {
		perror("UDP bind :8000");
		return 1;
	}

	if (rt) {
		/* Pin to core 3 and lock memory: no page faults on the brake path. */
		cpu_set_t set;

		CPU_ZERO(&set);
		CPU_SET(3, &set);
		sched_setaffinity(0, sizeof(set), &set);
		mlockall(MCL_CURRENT | MCL_FUTURE);
	}
	timesync_init(RAFT_NODE_COCKPIT);
	raft_init(RAFT_NODE_COCKPIT, now_ms());
	netwatch_init(RAFT_NODE_COCKPIT, now_ms());
	start_thread(udp_thread, 80, rt);       /* brake chain: highest */
	start_thread(raft_thread, 75, rt);      /* REF on time: below the brake chain, above CAN_RX */
	start_thread(can_rx_thread, 70, rt);
	start_thread(periodic_thread, 60, rt);
	start_thread(pps_thread, 85, rt);       /* Lab 5: the edge on time, above everything */
	start_thread(hz_thread, 50, rt);
	printf("cockpit up%s. Commands: b = inject bad CTRL, q = quit, h = the Raft ones\n",
	       rt ? " (SCHED_FIFO, core 3)" : " (normal priority; sudo for SCHED_FIFO)");

	bool have_stdin = true;
	int64_t next_status = now_ms() + 1000;

	while (1) {
		struct pollfd in = { .fd = STDIN_FILENO, .events = POLLIN };
		int64_t wait = next_status - now_ms();

		print_pps();
		show_new_entries();
		if (wait < 0) {
			wait = 0;
		}
		if (wait > 50) {
			wait = 50; /* the PPS line and new log entries show within 50 ms */
		}
		if (have_stdin && poll(&in, 1, (int)wait) > 0) {
			char line[128];

			if (fgets(line, sizeof(line), stdin) == NULL) {
				have_stdin = false; /* stdin is /dev/null: no console, use SIGUSR1 */
			} else if (!console_command(line)) {
				break; /* q */
			}
			continue;
		}
		if (!have_stdin) {
			sleep_until_ms(now_ms() + wait);
		}
		if (now_ms() < next_status) {
			continue;
		}
		next_status += 1000;

		pthread_mutex_lock(&st_lock);
		printf("zone %s (last: %s) | DEG A %d B %d | N2 %s A %s B %s | N3 %s A %s B %s\n",
		       zone_ok ? "OK" : "ERROR", reason_name(last_reason), degraded[BUS_A],
		       degraded[BUS_B], peer_state[ZONE_N2] ? "ERR" : "OK",
		       hb_age(ZONE_N2, BUS_A) < HB_TIMEOUT_MS ? "ok" : "--",
		       hb_age(ZONE_N2, BUS_B) < HB_TIMEOUT_MS ? "ok" : "--",
		       peer_state[ZONE_N3] ? "ERR" : "OK",
		       hb_age(ZONE_N3, BUS_A) < HB_TIMEOUT_MS ? "ok" : "--",
		       hb_age(ZONE_N3, BUS_B) < HB_TIMEOUT_MS ? "ok" : "--");
		int8_t fv = force_value;
		pthread_mutex_unlock(&st_lock);
		printf("UDP/s %u CTRL/s %u | raw steer %d thr %d brk %d | FORCE/s %u (%d) | "
		       "relayed rx %u tx %u | tx fail A %u B %u\n",
		       atomic_exchange(&udp_count, 0), atomic_exchange(&ctrl_count, 0),
		       atomic_load(&raw_steer), atomic_load(&raw_thr), atomic_load(&raw_brk),
		       atomic_exchange(&force_count, 0), fv, atomic_exchange(&relayed_count, 0),
		       atomic_exchange(&relayed_tx, 0),
		       atomic_exchange(&tx_fail[BUS_A], 0), atomic_exchange(&tx_fail[BUS_B], 0));
		print_raft_status();
	}
	gpio_put(GPIO_ERR, true);
	return 0;
}
