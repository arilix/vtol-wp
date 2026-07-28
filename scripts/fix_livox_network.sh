#!/usr/bin/env bash
# Permanent fix for the Livox MID360 disconnecting every few seconds.
#
# Root cause: eth0 is managed by cloud-init's netplan config
# (/etc/netplan/50-cloud-init.yaml), which sets it to DHCP (dhcp4: true).
# The Livox lidar has no DHCP server, so NetworkManager keeps retrying
# DHCP, failing, and re-cycling the link (this is what looks like
# "disconnect every ~5s, needs manual Connect click"). Any live `nmcli`
# edit gets clobbered because netplan/cloud-init is the real source of
# truth and regenerates the NetworkManager profile on every interface
# state change.
#
# Fix: drop a higher-priority netplan file that pins eth0 to the static
# IP the driver config already expects (192.168.1.50/24, matching
# src/livox_ros_driver2/config/MID360_config.json host_ip), so netplan
# never asks for DHCP on that interface again.
#
# Must be run with sudo.
set -euo pipefail

IFACE="eth0"
STATIC_IP="192.168.1.50/24"
NETPLAN_FILE="/etc/netplan/99-livox-static.yaml"

if [[ $EUID -ne 0 ]]; then
  echo "Run this with sudo: sudo $0" >&2
  exit 1
fi

cat > "$NETPLAN_FILE" <<EOF
network:
  version: 2
  ethernets:
    ${IFACE}:
      dhcp4: no
      dhcp6: no
      addresses: [${STATIC_IP}]
      optional: true
EOF
chmod 600 "$NETPLAN_FILE"

echo "Wrote ${NETPLAN_FILE}, applying netplan..."
netplan apply

sleep 2
echo "--- interface state ---"
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status | grep "$IFACE" || true
ip -4 addr show "$IFACE"

echo "--- pinging lidar (192.168.1.141) ---"
ping -c 3 -W 2 192.168.1.141 || echo "WARNING: lidar not responding to ping, check cable/power"
