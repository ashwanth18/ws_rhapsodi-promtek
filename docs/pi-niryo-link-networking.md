# Pi ↔ Niryo link networking (production Pi5)

Host-level setup so the Niryo Ned on **link-local Ethernet** can reach the
internet/DNS and sync time from the Pi. This is **not** inside Docker; it must
be applied on the Pi OS (and on a fresh Pi via Ansible).

Canonical IPs:

| Host | Interface | Address |
|------|-----------|---------|
| Niryo | eth (robot) | `169.254.200.200/16` |
| Pi | `eth0` (to arm) | `169.254.200.201/16` |
| Pi | `wlan0` (upstream) | DHCP / site Wi‑Fi |
| Pi | Tailscale | MagicDNS / `100.x` for fleet |

## What we installed on `rhapsodi-pi5` (Jul 2026)

### 1. NAT: arm → Pi Wi‑Fi

- Script: `/usr/local/sbin/rhapsodi-robot-netshare.sh` (repo: `ansible/files/rhapsodi-robot-netshare.sh`)
- Unit: `rhapsodi-robot-netshare.service` (enabled)
- Behavior:
  - `net.ipv4.ip_forward=1`
  - `MASQUERADE` for `169.254.0.0/16` out `wlan0`
  - `DOCKER-USER` forward accept (Docker otherwise drops FORWARD)
  - UDP/123 from the robot net into the Pi (NTP)

Without this, the arm has no default route off the link-local segment (no apt,
no public NTP, broken DNS).

### 2. Chrony on the Pi (NTP server for the arm)

In `/etc/chrony/chrony.conf`:

```text
allow 169.254.0.0/16
local stratum 10
```

Pi stays synced to public NTP over Wi‑Fi/Tailscale; arm uses the Pi as its
server. See also [ROBOT_CLOCK_SYNC.md](../ROBOT_CLOCK_SYNC.md).

### 3. Niryo `systemd-timesyncd`

On the robot (`niryo` @ `169.254.200.200`), `/etc/systemd/timesyncd.conf`:

```ini
[Time]
NTP=169.254.200.201
```

Also give the arm a **default route via the Pi** and DNS (`8.8.8.8` / `1.1.1.1`)
so packages and NTP work after NAT is up.

### 4. Verify

```bash
# On Pi
systemctl status rhapsodi-robot-netshare.service chrony --no-pager
sudo chronyc clients
ping -c 2 169.254.200.200

# On Niryo
ping -c 2 8.8.8.8
timedatectl status   # System clock synchronized: yes
```

Dashboard clock strip: Browser / Pi (`/host_info`) / Niryo (`/joint_states`
stamp). Large skew breaks MoveIt.

## Apply on a new Pi (Ansible)

From the monorepo controller:

```bash
ansible-playbook -i ansible/inventory/tailscale.py ansible/provision.yml \
  --limit rhapsodi-pi5 \
  -e robot_type=niryo ...
```

`provision.yml` includes `tasks/pi_niryo_link.yml` when `robot_type=niryo`.

Manual one-shot:

```bash
sudo install -m 0755 ansible/files/rhapsodi-robot-netshare.sh \
  /usr/local/sbin/rhapsodi-robot-netshare.sh
sudo cp ansible/templates/rhapsodi-robot-netshare.service.j2 \
  /etc/systemd/system/rhapsodi-robot-netshare.service
sudo systemctl daemon-reload
sudo systemctl enable --now rhapsodi-robot-netshare.service
```

## Related

- Link IP plan / Netplan: [README.md](../README.md) (Dual‑NIC network setup)
- Clock sync modes (Pi vs laptop debug): [ROBOT_CLOCK_SYNC.md](../ROBOT_CLOCK_SYNC.md)
- Runtime compose: `compose/devices/pi5.yml` (`ROBOT_IP=169.254.200.200`)
