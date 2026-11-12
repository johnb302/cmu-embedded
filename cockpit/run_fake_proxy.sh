#!/bin/sh
# Start the fake wheel/pedal proxy on the Pi, sending to the local cockpit.
# Commands: echo "b 1.0" > fp.fifo in this directory   (s/t/b value, l/r/x taps, p pause, q quit)
cd "$(dirname "$0")"
pkill -f "python3 fake_proxy.py"
pkill -f "tail -f fp.fifo"
rm -f fp.fifo
mkfifo fp.fifo
setsid sh -c "tail -f fp.fifo | python3 fake_proxy.py --remote 127.0.0.1 > fp.log 2>&1" < /dev/null > /dev/null 2>&1 &
