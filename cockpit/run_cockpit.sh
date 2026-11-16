#!/bin/sh
# Start the cockpit in the background, logging to cockpit.log in this directory (the previous
# run's log is kept as cockpit.prev.log).
# Console commands go in through cmd.fifo, like the fake proxy's fp.fifo; replies land in cockpit.log:
#   echo "net link down 3 A" > cmd.fifo     (echo h > cmd.fifo lists the commands)
cd "$(dirname "$0")"
pkill -x cockpit 2>/dev/null
pkill -f '^tail -f cmd\.fifo' 2>/dev/null # anchored: not any process that merely mentions it
sleep 0.3
rm -f cmd.fifo
[ -f cockpit.log ] && mv cockpit.log cockpit.prev.log # keep the last run's log (who reported what)
mkfifo cmd.fifo
setsid sh -c "tail -f cmd.fifo | ./cockpit > cockpit.log 2>&1" < /dev/null > /dev/null 2>&1 &
sleep 0.3
echo "cockpit pid $(pgrep -x cockpit)"
