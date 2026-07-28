#!/bin/bash
# Share Pi Wi-Fi (wlan0) with Niryo on eth0 (169.254.0.0/16).
set -euo pipefail

ROBOT_IF="${ROBOT_IF:-eth0}"
UPSTREAM_IF="${UPSTREAM_IF:-wlan0}"
ROBOT_NET="${ROBOT_NET:-169.254.0.0/16}"

sysctl -w net.ipv4.ip_forward=1 >/dev/null

# Idempotent: delete then re-add
iptables -t nat -C POSTROUTING -s "$ROBOT_NET" -o "$UPSTREAM_IF" -j MASQUERADE 2>/dev/null \
  || iptables -t nat -A POSTROUTING -s "$ROBOT_NET" -o "$UPSTREAM_IF" -j MASQUERADE

# Docker sets FORWARD policy DROP; use DOCKER-USER so rules survive docker restarts.
if iptables -L DOCKER-USER -n >/dev/null 2>&1; then
  iptables -C DOCKER-USER -i "$ROBOT_IF" -o "$UPSTREAM_IF" -j ACCEPT 2>/dev/null \
    || iptables -I DOCKER-USER 1 -i "$ROBOT_IF" -o "$UPSTREAM_IF" -j ACCEPT
  iptables -C DOCKER-USER -i "$UPSTREAM_IF" -o "$ROBOT_IF" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null \
    || iptables -I DOCKER-USER 1 -i "$UPSTREAM_IF" -o "$ROBOT_IF" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
else
  iptables -C FORWARD -i "$ROBOT_IF" -o "$UPSTREAM_IF" -j ACCEPT 2>/dev/null \
    || iptables -I FORWARD 1 -i "$ROBOT_IF" -o "$UPSTREAM_IF" -j ACCEPT
  iptables -C FORWARD -i "$UPSTREAM_IF" -o "$ROBOT_IF" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null \
    || iptables -I FORWARD 1 -i "$UPSTREAM_IF" -o "$ROBOT_IF" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
fi

# Allow NTP from robot to Pi
iptables -C INPUT -i "$ROBOT_IF" -s "$ROBOT_NET" -p udp --dport 123 -j ACCEPT 2>/dev/null \
  || iptables -I INPUT 1 -i "$ROBOT_IF" -s "$ROBOT_NET" -p udp --dport 123 -j ACCEPT

echo "rhapsodi-robot-netshare: NAT $ROBOT_NET via $UPSTREAM_IF (from $ROBOT_IF)"
