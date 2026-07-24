#!/usr/bin/env bash
set -euo pipefail

PROMTEK_CONFIG_REL='c:\promtek/config/promtek-condor-rhapsodi-agent'
PROMTEK_CONFIG_DIR="/data/condor-agent/home/${PROMTEK_CONFIG_REL}"
LOG_DIR="/data/condor-agent/logs"

mkdir -p "${PROMTEK_CONFIG_DIR}" "${LOG_DIR}"

if [ ! -f "${PROMTEK_CONFIG_DIR}/appsettings.json" ]; then
  cp /opt/condor-agent/appsettings.json "${PROMTEK_CONFIG_DIR}/appsettings.json"
fi

if [ -f /opt/condor-agent/appsettings.Development.json ] && \
   [ ! -f "${PROMTEK_CONFIG_DIR}/appsettings.Development.json" ]; then
  cp /opt/condor-agent/appsettings.Development.json \
    "${PROMTEK_CONFIG_DIR}/appsettings.Development.json"
fi

cd /data/condor-agent/home
exec dotnet /opt/condor-agent/Promtek.Condor.Rhapsodi.Agent.dll
