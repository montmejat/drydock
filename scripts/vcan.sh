#!/usr/bin/env bash
# Makes a virtual CAN interface (vcan0 by default), up. Needs root once;
# after that, anyone can open CAN sockets on it. It lasts until reboot.
#
#   sudo scripts/vcan.sh [name]
set -euo pipefail

name=${1:-vcan0}
modprobe vcan
if ! ip link show "$name" >/dev/null 2>&1; then
  ip link add dev "$name" type vcan
fi
ip link set up "$name"
ip -br link show "$name"
