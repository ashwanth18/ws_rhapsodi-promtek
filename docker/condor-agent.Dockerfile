FROM mcr.microsoft.com/dotnet/aspnet:8.0-jammy

WORKDIR /opt/condor-agent

COPY ["Rhapsodi Condor Agent WIP Apr 8 2026/", "/opt/condor-agent/"]

RUN mkdir -p /data/condor-agent/logs /data/condor-agent/home

WORKDIR /data/condor-agent/home

ENTRYPOINT ["dotnet", "/opt/condor-agent/Promtek.Condor.Rhapsodi.Agent.dll"]
