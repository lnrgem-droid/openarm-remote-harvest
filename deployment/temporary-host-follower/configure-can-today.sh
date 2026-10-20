#!/usr/bin/env bash
set -euo pipefail

# Bring the two local CAN FD interfaces up. This sends no CAN motor frames.
for iface in can0 can1; do
  /usr/sbin/ip link set dev "$iface" down
  /usr/sbin/ip link set dev "$iface" type can bitrate 1000000 dbitrate 5000000 fd on
  /usr/sbin/ip link set dev "$iface" up
  /usr/sbin/ip -brief link show dev "$iface"
done
