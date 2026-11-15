#!/bin/sh
# From the Mac: copy cockpit/ and shared/ to the Pi's ~/lab5 and build there.
# Usage: cockpit/deploy.sh [user@host]   (default team5@team5.local)
# Doesn't restart anything; run ~/lab5/cockpit/run_cockpit.sh on the Pi for that.
set -e
HOST=${1:-team5@team5.local}
cd "$(dirname "$0")/.."

# shared/ only exists once the portable Raft code is there
DIRS="cockpit"
[ -d shared ] && DIRS="$DIRS shared"

# --delete drops files removed from the repo; the excludes keep the Pi's binary and logs,
# and leave out the Mac's test binaries (make -C shared/test on the Pi builds its own)
rsync -a --delete --exclude cockpit/cockpit --exclude '*.log' --exclude fp.fifo --exclude cmd.fifo \
    --exclude shared/test/host_test --exclude shared/test/sim --exclude 'shared/test/*.so' \
    $DIRS "$HOST:lab5/"
ssh "$HOST" 'make -C lab5/cockpit'
