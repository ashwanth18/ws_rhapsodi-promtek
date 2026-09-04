FROM mcr.microsoft.com/dotnet/aspnet:8.0-jammy

# Multi-arch Condor bundles:
#   arm64/Pi  -> Rhapsodi Condor Agent July 13 2026 (linux-arm64)
#   amd64/x86 -> Promtek Condor Rhapsodi Agent Aug 6 2026 (linux-x64)
ARG TARGETARCH

WORKDIR /opt/condor-agent

COPY ["Rhapsodi Condor Agent July 13 2026/", "/tmp/condor-agent-arm64/"]
COPY ["Promtek Condor Rhapsodi Agent Aug 6 2026/", "/tmp/condor-agent-amd64/"]
COPY ["docker/condor-agent-entrypoint.sh", "/condor-agent-entrypoint.sh"]

RUN set -eux; \
  arch="${TARGETARCH:-}"; \
  case "$arch" in \
    arm64|aarch64) src=/tmp/condor-agent-arm64 ;; \
    amd64|x86_64|"") src=/tmp/condor-agent-amd64 ;; \
    *) echo "unsupported TARGETARCH=${arch}" >&2; exit 1 ;; \
  esac; \
  cp -a "$src"/. /opt/condor-agent/; \
  rm -rf /tmp/condor-agent-arm64 /tmp/condor-agent-amd64; \
  mkdir -p /data/condor-agent/logs /data/condor-agent/home; \
  chmod +x /condor-agent-entrypoint.sh; \
  test -f /opt/condor-agent/Promtek.Condor.Rhapsodi.Agent.dll

WORKDIR /data/condor-agent/home

ENTRYPOINT ["/condor-agent-entrypoint.sh"]
