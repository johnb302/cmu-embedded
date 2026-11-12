#!/bin/sh
# One-time Pi setup (run with sudo): bring can0/can1 up at 500 kbit/s with
# automatic bus-off recovery, now and on every boot, via systemd-networkd.
set -e

cat > /etc/systemd/network/80-can.network <<'EOF'
[Match]
Name=can*

[CAN]
BitRate=500K
RestartSec=100ms
EOF

systemctl restart systemd-networkd
sleep 2
for i in can0 can1; do
	ip -details link show "$i" | head -3
done
