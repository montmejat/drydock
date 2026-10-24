#!/usr/bin/env bash
# Makes a virtual CAN interface (vcan0 by default), up, carrying both
# classic and CAN FD frames (MTU 72). Needs root once; after that, anyone
# can open CAN sockets on it. It lasts until reboot.
#
#   sudo scripts/vcan.sh [name]
set -euo pipefail

name=${1:-vcan0}
modprobe vcan
if ! ip link show "$name" >/dev/null 2>&1; then
  ip link add dev "$name" type vcan
fi
# The MTU can only change while the interface is down.
ip link set down "$name"
ip link set "$name" mtu 72
ip link set up "$name"
ip -br link show "$name"
