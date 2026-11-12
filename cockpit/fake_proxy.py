#!/usr/bin/env python3
"""
Fake wheel for testing without the Logitech hardware.

Sends the same packets as mac_proxy.py (4-byte counter + 272-byte
DIJOYSTATE2) every --interval-ms to the Pi, and listens for the force byte.
Controls, one command per line on stdin:

    s <-1..1>   steering (negative = left)      t <0..1>   throttle
    b <0..1>    brake                           l / r      tap left / right turn button
    x           tap the wheel's self-test button (twice quickly = double press)
    p           pause sending (simulates the proxy stopping); p again resumes
    q           quit
"""
import argparse, select, socket, struct, sys, time

DIJOYSTATE2 = struct.Struct("<6i2i4I128s8i8i8i")
AXIS_MAX = 32767
BTN_LEFT, BTN_RIGHT, BTN_SELF_TEST = 4, 5, 10   # same indices as the cockpit
TAP_S = 0.1                                      # hold a tapped button 100 ms

def pack(steer, thr, brk, held):
    btn = bytearray(128)
    for i in held:
        btn[i] = 0x80
    return DIJOYSTATE2.pack(int(steer * AXIS_MAX), int(thr * AXIS_MAX), 0, 0, 0,
                            int(brk * AXIS_MAX), 0, 0, *([0xFFFFFFFF] * 4), bytes(btn),
                            *([0] * 24))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--remote", default="172.26.34.232")
    ap.add_argument("--send-port", type=int, default=8000)
    ap.add_argument("--recv-port", type=int, default=8001)
    ap.add_argument("--interval-ms", type=int, default=20)
    a = ap.parse_args()

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", a.recv_port))
    rx.setblocking(False)

    steer = thr = brk = 0.0
    taps = {}            # button index -> release time
    paused = False
    counter, force = 0, None
    period = a.interval_ms / 1000.0
    next_tx = time.monotonic()
    print("fake proxy -> %s:%d every %d ms. Commands: s t b l r x p q"
          % (a.remote, a.send_port, a.interval_ms), flush=True)

    while True:
        if select.select([sys.stdin], [], [], 0.002)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            w = line.split()
            if not w:
                continue
            c = w[0]
            try:
                if c == "s": steer = max(-1.0, min(1.0, float(w[1])))
                elif c == "t": thr = max(0.0, min(1.0, float(w[1])))
                elif c == "b": brk = max(0.0, min(1.0, float(w[1])))
                elif c == "l": taps[BTN_LEFT] = time.monotonic() + TAP_S
                elif c == "r": taps[BTN_RIGHT] = time.monotonic() + TAP_S
                elif c == "x": taps[BTN_SELF_TEST] = time.monotonic() + TAP_S
                elif c == "p": paused = not paused
                elif c == "q": break
            except (IndexError, ValueError):
                print("? " + line.strip(), flush=True)
                continue
            print("steer %+.2f thr %.2f brk %.2f%s" % (steer, thr, brk,
                  " (paused)" if paused else ""), flush=True)

        now = time.monotonic()
        taps = {k: v for k, v in taps.items() if v > now}
        if now >= next_tx:
            next_tx += period
            if not paused:
                tx.sendto(counter.to_bytes(4, "little") + pack(steer, thr, brk, taps.keys()),
                          (a.remote, a.send_port))
                counter += 1
        try:
            while True:
                d, _ = rx.recvfrom(16)
                force = int.from_bytes(d[:1], "little", signed=True)
        except (BlockingIOError, OSError):
            pass

if __name__ == "__main__":
    main()
