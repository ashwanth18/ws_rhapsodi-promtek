#!/usr/bin/env bash
# Switch laptop Ethernet between arm-link NetworkManager profiles.
#
# Profiles (created by: setup):
#   rhapsodi-niryo-link  — current/other robot link-local  169.254.99.187/16
#   rhapsodi-jaka-link   — JAKA factory subnet             10.5.5.101/24
#                          (controller default 10.5.5.100)
#
# Usage:
#   bash scripts/switch_arm_ethernet.sh setup   # create profiles (once, needs sudo)
#   bash scripts/switch_arm_ethernet.sh niryo   # use Niryo / link-local profile
#   bash scripts/switch_arm_ethernet.sh jaka    # use JAKA 10.5.5.x profile
#   bash scripts/switch_arm_ethernet.sh status
#
# Optional env overrides:
#   IFACE=enp129s0
#   NIRYO_ADDR=169.254.99.187/16
#   JAKA_ADDR=10.5.5.101/24
#   JAKA_ROBOT_IP=10.5.5.100
set -euo pipefail

IFACE="${IFACE:-enp129s0}"
NIRYO_CON="${NIRYO_CON:-rhapsodi-niryo-link}"
JAKA_CON="${JAKA_CON:-rhapsodi-jaka-link}"
NIRYO_ADDR="${NIRYO_ADDR:-169.254.99.187/16}"
JAKA_ADDR="${JAKA_ADDR:-10.5.5.101/24}"
JAKA_ROBOT_IP="${JAKA_ROBOT_IP:-10.5.5.100}"

die() { echo "error: $*" >&2; exit 1; }

need_nmcli() {
  command -v nmcli >/dev/null || die "nmcli not found (install NetworkManager)"
}

con_exists() {
  nmcli -t -f NAME connection show | grep -Fxq "$1"
}

ensure_root_for_setup() {
  if [[ "${EUID}" -ne 0 ]]; then
    exec sudo --preserve-env=IFACE,NIRYO_CON,JAKA_CON,NIRYO_ADDR,JAKA_ADDR,JAKA_ROBOT_IP \
      "$0" "$@"
  fi
}

cmd_setup() {
  need_nmcli
  ensure_root_for_setup setup

  if ! ip link show "${IFACE}" >/dev/null 2>&1; then
    die "interface ${IFACE} not found — set IFACE=..."
  fi

  if con_exists "${NIRYO_CON}"; then
    echo "already exists: ${NIRYO_CON}"
  else
    echo "creating ${NIRYO_CON} (${NIRYO_ADDR} on ${IFACE})"
    nmcli connection add type ethernet ifname "${IFACE}" con-name "${NIRYO_CON}" \
      ipv4.method manual \
      ipv4.addresses "${NIRYO_ADDR}" \
      ipv4.gateway "" \
      ipv4.never-default yes \
      ipv6.method disabled \
      connection.autoconnect no
  fi

  if con_exists "${JAKA_CON}"; then
    echo "already exists: ${JAKA_CON}"
  else
    echo "creating ${JAKA_CON} (${JAKA_ADDR} on ${IFACE})"
    nmcli connection add type ethernet ifname "${IFACE}" con-name "${JAKA_CON}" \
      ipv4.method manual \
      ipv4.addresses "${JAKA_ADDR}" \
      ipv4.gateway "" \
      ipv4.never-default yes \
      ipv6.method disabled \
      connection.autoconnect no
  fi

  # Avoid fighting auto "Wired connection 1" on the same NIC
  if con_exists "Wired connection 1"; then
    nmcli connection modify "Wired connection 1" connection.autoconnect no || true
    echo "disabled autoconnect on 'Wired connection 1' (profiles above own ${IFACE})"
  fi

  echo
  echo "Done. Switch with:"
  echo "  bash scripts/switch_arm_ethernet.sh niryo"
  echo "  bash scripts/switch_arm_ethernet.sh jaka"
}

activate() {
  local con="$1"
  need_nmcli
  con_exists "${con}" || die "profile '${con}' missing — run: bash scripts/switch_arm_ethernet.sh setup"
  echo "Activating ${con} on ${IFACE}..."
  nmcli connection up "${con}" ifname "${IFACE}"
}

cmd_niryo() {
  activate "${NIRYO_CON}"
  echo
  ip -br addr show "${IFACE}"
  echo "Niryo/other link-local profile active (${NIRYO_ADDR})."
}

cmd_jaka() {
  activate "${JAKA_CON}"
  echo
  ip -br addr show "${IFACE}"
  echo "JAKA profile active (laptop ${JAKA_ADDR}, expect robot ${JAKA_ROBOT_IP})."
  echo "Probe:  ping -c 2 -I ${IFACE} ${JAKA_ROBOT_IP}"
  echo "Driver: ros2 launch jaka_driver robot_start.launch.py ip:=${JAKA_ROBOT_IP}"
  if ping -c 1 -W 1 -I "${IFACE}" "${JAKA_ROBOT_IP}" >/dev/null 2>&1; then
    echo "ping ${JAKA_ROBOT_IP}: OK"
  else
    echo "ping ${JAKA_ROBOT_IP}: no reply yet (check cable / controller IP)"
  fi
}

cmd_status() {
  need_nmcli
  echo "Interface: ${IFACE}"
  ip -br addr show "${IFACE}" || true
  echo
  echo "Profiles:"
  for con in "${NIRYO_CON}" "${JAKA_CON}" "Wired connection 1"; do
    if con_exists "${con}"; then
      active="$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: -v n="$con" '$1==n{print $2; exit}')"
      if [[ -n "${active}" ]]; then
        echo "  ${con}: ACTIVE on ${active}"
      else
        echo "  ${con}: present (inactive)"
      fi
    else
      echo "  ${con}: missing"
    fi
  done
  echo
  echo "Addresses on ${IFACE}:"
  ip -4 addr show "${IFACE}" | sed 's/^/  /' || true
}

usage() {
  cat <<EOF
Usage: $0 {setup|niryo|jaka|status}

  setup   Create NM profiles (sudo). Keeps your Niryo/other IP and adds JAKA.
  niryo   Switch Ethernet to ${NIRYO_CON} (${NIRYO_ADDR})
  jaka    Switch Ethernet to ${JAKA_CON} (${JAKA_ADDR})
  status  Show which profile/IP is active

Env: IFACE NIRYO_ADDR JAKA_ADDR JAKA_ROBOT_IP NIRYO_CON JAKA_CON
EOF
}

main() {
  local cmd="${1:-}"
  case "${cmd}" in
    setup) cmd_setup ;;
    niryo|ned|linklocal) cmd_niryo ;;
    jaka|jaka_zu5) cmd_jaka ;;
    status|st) cmd_status ;;
    -h|--help|"") usage; [[ -n "${cmd}" ]] || exit 1 ;;
    *) die "unknown command '${cmd}' (try: setup|niryo|jaka|status)" ;;
  esac
}

main "$@"
