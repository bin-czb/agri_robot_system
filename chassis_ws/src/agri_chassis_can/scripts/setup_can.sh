#!/usr/bin/env bash
set -Eeuo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-250000}"

sudo ip link set "${IFACE}" down 2>/dev/null || true
sudo ip link set "${IFACE}" type can bitrate "${BITRATE}"
sudo ip link set "${IFACE}" up
ip -details link show "${IFACE}"
echo "SocketCAN ready: ${IFACE} @ ${BITRATE} bit/s"
