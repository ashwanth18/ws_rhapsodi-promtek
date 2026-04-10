FROM mcr.microsoft.com/dotnet/aspnet:8.0-jammy

WORKDIR /app

COPY ["Rhapsodi Condor Agent WIP Apr 8 2026/", "/app/"]

RUN mkdir -p /data/condor-agent/logs

ENTRYPOINT ["dotnet", "/app/Promtek.Condor.Rhapsodi.Agent.dll"]
