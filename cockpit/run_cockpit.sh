#!/bin/sh
# Start the cockpit in the background; log to ~/lab3/cockpit.log.
cd "$(dirname "$0")"
pkill -x cockpit 2>/dev/null
sleep 0.3
nohup ./cockpit < /dev/null > cockpit.log 2>&1 &
echo "cockpit pid $!"
