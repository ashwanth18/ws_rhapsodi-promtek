FROM mcr.microsoft.com/dotnet/aspnet:8.0-jammy

WORKDIR /opt/condor-agent

COPY ["Rhapsodi Condor Agent WIP Apr 8 2026/", "/opt/condor-agent/"]
COPY ["docker/condor-agent-entrypoint.sh", "/condor-agent-entrypoint.sh"]

RUN mkdir -p /data/condor-agent/logs /data/condor-agent/home
RUN chmod +x /condor-agent-entrypoint.sh

WORKDIR /data/condor-agent/home

ENTRYPOINT ["/condor-agent-entrypoint.sh"]
