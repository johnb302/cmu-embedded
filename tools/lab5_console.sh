#!/usr/bin/env bash
# Log both Nucleo consoles to timestamped files, and send them commands.
#   tools/lab5_console.sh log            start logging N2 and N3 (Ctrl-C stops)
#   tools/lab5_console.sh send n3 "raft log all"
# Logs land in lab5_report/logs/<node>.log, each line prefixed with host time.
set -euo pipefail

# By ST-Link serial, since /dev/ttyACMx numbering follows plug order.
declare -A PORT=(
  [n2]=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066DFF514988525067184341-if02
  [n3]=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF514988525067182612-if02
)
LOG_DIR="$(dirname "$0")/../lab5_report/logs"

case "${1:-}" in
  log)
    mkdir -p "$LOG_DIR"
    for node in n2 n3; do
      stty -F "${PORT[$node]}" 115200 raw -echo
      ts '%H:%M:%.S' < "${PORT[$node]}" >> "$LOG_DIR/$node.log" &
    done
    echo "logging to $LOG_DIR (Ctrl-C to stop)"
    wait
    ;;
  send)
    # Shell expects CR line endings.
    printf '%s\r' "$3" > "${PORT[$2]}"
    ;;
  *)
    sed -n '2,5p' "$0"
    exit 1
    ;;
esac
