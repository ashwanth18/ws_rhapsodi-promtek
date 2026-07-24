# Condor Agent + Robot Pi Deployment Guide

This guide documents the Dockerized physical-robot stack, with special focus on the Promtek Condor agent, so future Pi rollouts are repeatable and do not regress into the earlier config and persistence issues.

## What This Stack Includes

The physical robot deployment lives in `docker-compose.robot-prod.yml` and runs:

- `db`
- `backend`
- `processing`
- `webhook_service`
- `condor_agent`
- `dashboard`
- `rosbridge`
- `robot_start_adapter`
- `scooping_stack`
- `scale_launcher`
- `micro_ros_agent`
- `pouring_controller`
- `data_collection`
- `orchestrator`

Important runtime behavior:

- ROS services run with `network_mode: host` for DDS and robot connectivity.
- The weighing scale is the real serial device, not `weight_sim`.
- The micro-ROS agent is the containerized equivalent of:

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -v6
```

- `scooping_stack` launches `scooping_real.launch.py` in headless mode with `use_rviz:=false`.
- The Condor agent is built into its own image and talks to the local `webhook_service` inside Compose.

## Files To Know

- `docker-compose.robot-prod.yml`
- `robot-prod.env.example`
- `robot-prod.env`
- `docker/condor-agent.Dockerfile`
- `docker/condor-agent-entrypoint.sh`
- `Rhapsodi Condor Agent WIP Apr 8 2026/appsettings.json`

Each file has a different job:

- `robot-prod.env` is the per-Pi runtime contract.
- The Condor `appsettings.json` is the bundled seed config baked into the image.
- The entrypoint copies that seed config into a persistent runtime location on first boot.

## Condor Agent Design

The Condor agent has a Windows-style config path expectation even when running in Linux containers. That is the main thing that caused the earlier failures.

### How it works now

1. The published .NET agent bundle is copied into `/opt/condor-agent` by `docker/condor-agent.Dockerfile`.
1. The container starts through `docker/condor-agent-entrypoint.sh`.
1. On first start, the entrypoint seeds:

```text
/data/condor-agent/home/c:\promtek/config/promtek-condor-rhapsodi-agent/appsettings.json
```

1. The agent then runs with `WORKDIR=/data/condor-agent/home`, so its generated runtime state also lands under the persistent `./data` bind mount.
1. Registration state and generated keys survive container recreation because `./data` is host-mounted.

### Why this matters

Without that persistent seeded config:

- the agent regenerates placeholder `COPY_*` values
- registration state disappears across restarts
- websocket/auth behavior becomes confusing because the running config is not the one you think you baked into the image

## Persistent Data Layout

The important host-side paths are:

- `./data/condor-agent/home/c:\promtek/config/promtek-condor-rhapsodi-agent/appsettings.json`
- `./data/condor-agent/home/c:\promtek/config/promtek-condor-rhapsodi-agent/appsettings.Development.json`
- `./data/condor-agent/home/c:\promtek/config/promtek-condor-rhapsodi-agent/agent.key`
- `./data/condor-agent/logs/...`
- `./data/ros_home/...`

What persists where:

- Condor config and registration keys persist under `./data/condor-agent/home/...`
- Condor logs persist under `./data/condor-agent/logs/...`
- ROS user data such as scoop poses persist under `./data/ros_home/...`

## Env Var Strategy For Many Pis

Use one env file per robot. Do not treat `robot-prod.env` as a single global file forever.

Recommended pattern:

```bash
cp robot-prod.env.example robot-prod.pi-1.env
cp robot-prod.env.example robot-prod.pi-2.env
```

Then customize each file with that robot's values.

### Variables you will usually change per Pi

- `ROBOT_IP`
- `SCALE_DEVICE`
- `SCALE_BAUD`
- `MICRO_ROS_DEVICE`
- `CORS_ORIGINS`
- image tags if you pin releases

### Variables you usually keep the same

- `ROS_DOMAIN_ID`
- `RMW_IMPLEMENTATION`
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- `CONDOR_AGENT_RHAPSODI_URL=http://webhook_service:5000`
- `CONDOR_AGENT_PORT=5002`
- `TARGETS_YAML=/ws/install/share/robot_moveit/targets.yaml`
- `CONTAINER_SCENE_YAML=/ws/install/share/scooping_controller/config/container_scene_real.yaml`

### Condor-specific rollout note

Every Pi should have its own Condor identity:

- unique agent record in Condor
- unique `AgentName`
- unique `AuthorisationToken`
- correct `UserId` / `SiteId` for that robot

If you clone one Pi setup to another without changing Condor identity, you can end up with multiple devices impersonating the same agent.

## Build And Push From Laptop

Log in and ensure `buildx` exists:

```bash
docker login
docker buildx create --use
docker buildx inspect --bootstrap
```

Build the full set of multi-arch images from the laptop:

```bash
docker buildx build --platform linux/amd64,linux/arm64 -f docker/ros/Dockerfile.prod -t iserenity/rhapsodi-promtek:ros-prod --push .
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:backend --push ./src/backend
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:processing --push ./src/backend/processing
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:webhook --push ./src/backend/webhook_service
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:dashboard --push -f docker/dashboard.Dockerfile .
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:condor-agent --push -f docker/condor-agent.Dockerfile .
```

Notes:

- Run each `docker buildx build ... --push` as its own command.
- Do not paste multiple build commands onto one shell line.
- The Condor agent bundle must remain included in the build context. The `.dockerignore` exception for `Rhapsodi Condor Agent WIP Apr 8 2026/` is required.

## First-Time Pi Setup

From the repo on the Pi:

```bash
cp robot-prod.env.example robot-prod.pi-1.env
```

Edit the env file for that Pi:

```bash
nano robot-prod.pi-1.env
```

Minimum things to verify:

- `ROBOT_IP`
- `SCALE_DEVICE`
- `MICRO_ROS_DEVICE`
- `CONDOR_AGENT_IMAGE`
- `ROS_PROD_IMAGE`
- `TARGETS_YAML=/ws/install/share/robot_moveit/targets.yaml`

Bring the stack up:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml pull
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml up -d
```

## Updating An Existing Pi

Typical rollout flow:

```bash
git pull --ff-only
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml pull
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml up -d --remove-orphans
```

If you only changed the Condor image:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml pull condor_agent
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml up -d condor_agent
```

## When You Must Re-Seed Condor Config

If the running agent still shows placeholder values like:

- `COPY_AGENT_NAME_FROM_CONDOR_HERE`
- `COPY_CONDOR_URL_HERE`
- `COPY_AUTHORISATION_TOKEN_FROM_CONDOR_HERE`

then the persistent runtime config was seeded earlier from a bad image and is overriding the new bundled config.

Fix:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml down
rm -rf data/condor-agent/home
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml up -d condor_agent
```

Only do this when you intentionally want the agent to re-seed its config and registration state.

If the agent is already correctly registered and healthy, do not casually delete `data/condor-agent/home`.

## Day-To-Day Commands

Full stack:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml ps
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml logs -f
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml restart
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml down
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml up -d
```

Condor only:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml logs -f condor_agent
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml restart condor_agent
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml exec condor_agent bash
```

Scooping stack only:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml logs -f scooping_stack
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml restart scooping_stack
```

## Health Checks

### Compose status

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml ps
```

### Condor health

Healthy Condor logs usually contain:

- `Register:Not required (already registered)` or a successful registration
- `WebSocket:Connected OK`
- `Active:(200 OK)`

Watch them with:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml logs -f condor_agent
```

### Backend health

```bash
curl http://localhost:8000/host_info
curl http://localhost:5000/health
```

### ROS visibility from the ROS containers

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml exec scooping_stack bash -lc "printenv | grep FASTDDS_BUILTIN_TRANSPORTS; source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic list"
```

### Check persisted Condor runtime files

```bash
find data/condor-agent -type f | sort
```

## Debugging Checklist

### Condor agent cannot connect

Check:

- correct token and agent identity in the runtime config
- the agent was actually authorised in Condor
- outbound internet access from the Pi
- logs for `401 Unauthorized` versus placeholder config versus DNS/network failures

Useful commands:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml logs --tail=200 condor_agent
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml exec condor_agent bash -lc "find /data/condor-agent -type f | sort"
```

### Condor is running the wrong config

Check the persisted runtime file, not just the bundled file:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml exec condor_agent bash -lc "python - <<'PY'\nfrom pathlib import Path\np = Path('/data/condor-agent/home/c:\\\\promtek/config/promtek-condor-rhapsodi-agent/appsettings.json')\nprint(p.read_text())\nPY"
```

### ROS containers see only `/parameter_events` and `/rosout`

This usually means DDS discovery/config is broken. Verify:

- `ROS_LOCALHOST_ONLY=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- host networking is still enabled for ROS services

### `targets.yaml` not found

The correct installed path is:

```text
/ws/install/share/robot_moveit/targets.yaml
```

Not:

```text
/ws/install/share/robot_moveit/config/targets.yaml
```

### Scoop poses missing after restart

Those are expected under:

```text
./data/ros_home
```

If they vanish, check whether `./data/ros_home:/root/.ros` is still mounted in `docker-compose.robot-prod.yml`.

## Pitfalls To Avoid

- Do not assume editing the bundled `Rhapsodi Condor Agent WIP Apr 8 2026/appsettings.json` will change a Pi that already has seeded persistent config.
- Do not delete `data/condor-agent/home` unless you intentionally want to reset Condor runtime state.
- Do not change `CONDOR_AGENT_RHAPSODI_URL` away from `http://webhook_service:5000` for the Compose-internal path.
- Do not revert `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` unless you are deliberately retesting DDS transport behavior.
- Do not use `/ws/install/share/robot_moveit/config/targets.yaml`; that path was wrong.
- Do not expect a Pi-hosted dashboard to magically use the correct Pi URLs if the image was built with `localhost`. The dashboard bakes its API/rosbridge URLs at build time.
- Do not share real Condor credentials casually. Treat agent config values as secrets.
- Do not deploy many Pis with the same Condor token and agent identity.

## Recommended Release Process For Many Pis

1. Build and push versioned images from the laptop.
2. Keep one env file per robot, for example `robot-prod.pi-1.env`.
3. Keep Condor identity unique per robot.
4. Pull and restart on each Pi with its own env file.
5. Confirm `condor_agent`, `scooping_stack`, `rosbridge`, `backend`, and `webhook_service` are healthy before handing over.

Good verification sequence on each Pi:

```bash
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml ps
docker compose --env-file robot-prod.pi-1.env -f docker-compose.robot-prod.yml logs --tail=100 condor_agent
curl http://localhost:8000/host_info
curl http://localhost:5000/health
```

## Current Known-Good Condor Signals

The agent is considered healthy when you see logs like:

- `Register:Not required (already registered)`
- `WebSocket:Connected OK`
- `Ping:Started sending 'ping' message`
- `Active:(200 OK)`

Once you see those, the Condor part of the deployment is in the expected steady state.
