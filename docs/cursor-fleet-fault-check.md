# Cursor Automation draft — Rhapsodi fleet fault check

Use this as the instructions for a scheduled Cursor Automation (hourly recommended).

## Name
Rhapsodi fleet health check

## Trigger
Schedule: every hour (or every 30 minutes during active robot days)

## Tools
- Shell / terminal (SSH over Tailscale)
- Optional: GitHub (open/update an issue when faults persist)

## Instructions for the agent

1. From the repo root `ws_rhapsodi-promtek-dev` (or clone `ashwanth18/ws_rhapsodi-promtek` on branch `main` / current fleet branch), run:

```bash
bash scripts/fleet_health_check.sh
```

2. If the script exits non-zero:
   - Summarize which hosts failed and the FAULT lines.
   - For each failing host, SSH in (`ssh <hostname>` or `ssh admin@<hostname>`) and collect:
     - `docker ps -a`
     - `df -h /`
     - last 100 lines of the unhealthy/exited container logs
   - Propose the smallest safe remediation (restart a single container, free disk, etc.). Do **not** restart the whole stack while a batch may be running unless the user asked for that.
   - Report findings in the automation run summary.

3. If the script exits zero, reply with a one-line healthy summary and list of hosts checked.

4. Prefer Tailscale hostnames (`rhapsodi-pi5`, `jetson`, …). Never require public IPs.

## Notes
- Devices must have been provisioned with `scripts/provision_device.sh` (Tailscale SSH + Docker + exporters).
- Jetson docker socket access may require the user to be in the `docker` group (`sudo usermod -aG docker $USER` then re-login).
