#!/usr/bin/env bash
# Switch laptop Ethernet between arm-link NetworkManager profiles.
#
# Profiles (created by: setup):
#   rhapsodi-niryo-link   — Niryo / other robot link-local  169.254.99.187/16
#   rhapsodi-jaka-link    — Schneider/JAKA cell subnet      192.168.88.101/24
#   rhapsodi-lexium-link  — Lexium Cobot cell subnet        192.168.88.10/24
#                           (controller 192.168.88.82)
#
# Usage:
#   bash scripts/switch_arm_ethernet.sh setup   # create profiles (once, needs sudo)
#   bash scripts/switch_arm_ethernet.sh niryo   # use Niryo / link-local profile
#   bash scripts/switch_arm_ethernet.sh jaka    # use JAKA 192.168.88.101 profile
#   bash scripts/switch_arm_ethernet.sh lexium  # use Lexium 192.168.88.10 profile
#   bash scripts/switch_arm_ethernet.sh status
#
# Optional env overrides:
#   IFACE=enp129s0
#   NIRYO_ADDR=169.254.99.187/16
#   JAKA_ADDR=192.168.88.101/24
#   JAKA_ROBOT_IP=192.168.88.82
#   LEXIUM_ADDR=192.168.88.10/24
#   LEXIUM_ROBOT_IP=192.168.88.82
set -euo pipefail

IFACE="${IFACE:-enp129s0}"
NIRYO_CON="${NIRYO_CON:-rhapsodi-niryo-link}"
JAKA_CON="${JAKA_CON:-rhapsodi-jaka-link}"
LEXIUM_CON="${LEXIUM_CON:-rhapsodi-lexium-link}"
NIRYO_ADDR="${NIRYO_ADDR:-169.254.99.187/16}"
JAKA_ADDR="${JAKA_ADDR:-192.168.88.101/24}"
JAKA_ROBOT_IP="${JAKA_ROBOT_IP:-192.168.88.82}"
LEXIUM_ADDR="${LEXIUM_ADDR:-192.168.88.10/24}"
LEXIUM_ROBOT_IP="${LEXIUM_ROBOT_IP:-192.168.88.82}"

die() { echo "error: $*" >&2; exit 1; }

need_nmcli() {
  command -v nmcli >/dev/null || die "nmcli not found (install NetworkManager)"
}

con_exists() {
  nmcli -t -f NAME connection show | grep -Fxq "$1"
}

ensure_root_for_setup() {
  if [[ "${EUID}" -ne 0 ]]; then
    exec sudo --preserve-env=IFACE,NIRYO_CON,JAKA_CON,LEXIUM_CON,NIRYO_ADDR,JAKA_ADDR,JAKA_ROBOT_IP,LEXIUM_ADDR,LEXIUM_ROBOT_IP \
      "$0" "$@"
  fi
}

_add_manual_eth() {
  local con="$1" addr="$2"
  if con_exists "${con}"; then
    echo "already exists: ${con}"
  else
    echo "creating ${con} (${addr} on ${IFACE})"
    nmcli connection add type ethernet ifname "${IFACE}" con-name "${con}" \
      ipv4.method manual \
      ipv4.addresses "${addr}" \
      ipv4.gateway "" \
      ipv4.never-default yes \
      ipv6.method disabled \
      connection.autoconnect no
  fi
}

cmd_setup() {
  need_nmcli
  ensure_root_for_setup setup

  if ! ip link show "${IFACE}" >/dev/null 2>&1; then
    die "interface ${IFACE} not found — set IFACE=..."
  fi

  _add_manual_eth "${NIRYO_CON}" "${NIRYO_ADDR}"
  _add_manual_eth "${JAKA_CON}" "${JAKA_ADDR}"
  _add_manual_eth "${LEXIUM_CON}" "${LEXIUM_ADDR}"

  # Avoid fighting auto "Wired connection 1" on the same NIC
  if con_exists "Wired connection 1"; then
    nmcli connection modify "Wired connection 1" connection.autoconnect no || true
    echo "disabled autoconnect on 'Wired connection 1' (profiles above own ${IFACE})"
  fi

  echo
  echo "Done. Switch with:"
  echo "  bash scripts/switch_arm_ethernet.sh niryo"
  echo "  bash scripts/switch_arm_ethernet.sh jaka"
  echo "  bash scripts/switch_arm_ethernet.sh lexium"
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

_probe_robot() {
  local robot_ip="$1"
  echo "Probe:  ping -c 2 -I ${IFACE} ${robot_ip}"
  echo "TCP:    nc -vz ${robot_ip} 10001   # Lexium commands"
  echo "        nc -vz ${robot_ip} 10000   # Lexium feedback"
  echo "Driver: ros2 launch lexium_driver lexium_driver.launch.py ip:=${robot_ip}"
  echo "Stack:  make lexium-session   # RViz + Lexium Safety panel on host"
  if ping -c 1 -W 1 -I "${IFACE}" "${robot_ip}" >/dev/null 2>&1; then
    echo "ping ${robot_ip}: OK"
  else
    echo "ping ${robot_ip}: no reply yet (check cable / controller IP)"
  fi
}

cmd_jaka() {
  activate "${JAKA_CON}"
  echo
  ip -br addr show "${IFACE}"
  echo "JAKA profile active (laptop ${JAKA_ADDR}, expect robot ${JAKA_ROBOT_IP})."
  _probe_robot "${JAKA_ROBOT_IP}"
}

cmd_lexium() {
  activate "${LEXIUM_CON}"
  echo
  ip -br addr show "${IFACE}"
  echo "Lexium profile active (laptop ${LEXIUM_ADDR}, expect robot ${LEXIUM_ROBOT_IP})."
  _probe_robot "${LEXIUM_ROBOT_IP}"
}

cmd_status() {
  need_nmcli
  echo "Interface: ${IFACE}"
  ip -br addr show "${IFACE}" || true
  echo
  echo "Profiles:"
  for con in "${NIRYO_CON}" "${JAKA_CON}" "${LEXIUM_CON}" "Wired connection 1"; do
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
Usage: $0 {setup|niryo|jaka|lexium|status}

  setup   Create NM profiles (sudo). Keeps Niryo/other IP and adds JAKA + Lexium.
  niryo   Switch Ethernet to ${NIRYO_CON} (${NIRYO_ADDR})
  jaka    Switch Ethernet to ${JAKA_CON} (${JAKA_ADDR})
  lexium  Switch Ethernet to ${LEXIUM_CON} (${LEXIUM_ADDR})
  status  Show which profile/IP is active

Env: IFACE NIRYO_ADDR JAKA_ADDR JAKA_ROBOT_IP LEXIUM_ADDR LEXIUM_ROBOT_IP
     NIRYO_CON JAKA_CON LEXIUM_CON
EOF
}

main() {
  local cmd="${1:-}"
  case "${cmd}" in
    setup) cmd_setup ;;
    niryo|ned|linklocal) cmd_niryo ;;
    jaka|jaka_zu5) cmd_jaka ;;
    lexium|schneider) cmd_lexium ;;
    status|st) cmd_status ;;
    -h|--help|"") usage; [[ -n "${cmd}" ]] || exit 1 ;;
    *) die "unknown command '${cmd}' (try: setup|niryo|jaka|lexium|status)" ;;
  esac
}

main "$@"
