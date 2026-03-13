# Robot Clock Sync

This note documents the clock sync setup for the Niryo robot when switching
between:

- normal operation through the Raspberry Pi
- direct debugging from the laptop

The goal is to keep the robot clock aligned so MoveIt and `/joint_states` do
not fail due to stale timestamps.

## Why this matters

When the robot clock is not synchronized with the machine running ROS 2,
MoveIt can report errors like:

- `Didn't receive robot state (joint angles) with recent timestamp`
- `Check clock synchronization if you are running ROS across multiple machines`

## Network / IPs

Current link IPs:

- Niryo robot: `169.254.200.200`
- Pi robot-facing IP: `169.254.200.201`
- Laptop robot-facing IP: `169.254.99.187`

## Strategy

Use the directly connected host as the robot's time source:

- Pi connected to robot -> robot syncs to `169.254.200.201`
- Laptop connected directly to robot -> robot syncs to `169.254.99.187`

The Pi and laptop should both be able to act as NTP servers.

## Pi Setup

Install and run `chrony` on the Pi:

```bash
sudo apt-get install -y chrony
echo "allow 169.254.0.0/16" | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
systemctl status chrony --no-pager
```

Useful checks:

```bash
sudo chronyc sources -v
sudo chronyc clients
```

## Laptop Setup

Install and run `chrony` on the laptop too:

```bash
sudo apt-get install -y chrony
echo "allow 169.254.0.0/16" | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
systemctl status chrony --no-pager
```

Useful checks:

```bash
sudo chronyc sources -v
sudo chronyc clients
```

## Robot Setup

Edit the robot time client config:

```bash
sudo nano /etc/systemd/timesyncd.conf
```

## Pi Mode

Use this when the robot is connected to the Raspberry Pi:

```ini
[Time]
NTP=169.254.200.201
FallbackNTP=
```

Then restart:

```bash
sudo systemctl restart systemd-timesyncd
timedatectl status
timedatectl show-timesync --all | grep -E "ServerName|ServerAddress"
```

## Laptop Debug Mode

Use this when the robot is connected directly to the laptop:

```ini
[Time]
NTP=169.254.99.187
FallbackNTP=
```

Then restart:

```bash
sudo systemctl restart systemd-timesyncd
timedatectl status
timedatectl show-timesync --all | grep -E "ServerName|ServerAddress"
```

## Verify Connectivity

When robot is connected to the Pi:

```bash
ping -c 3 169.254.200.201
```

When robot is connected to the laptop:

```bash
ping -c 3 169.254.99.187
```

## Expected Result

On the robot:

```bash
timedatectl status
```

You want:

- `System clock synchronized: yes`

On the active host (Pi or laptop):

```bash
sudo chronyc clients
```

You should eventually see the robot as an NTP client.

## Practical Workflow

Normal operation:

1. Robot connected to Pi
2. Robot `NTP=169.254.200.201`
3. Pi runs the ROS 2 stack

Direct laptop debugging:

1. Robot connected directly to laptop
2. Robot `NTP=169.254.99.187`
3. Laptop runs Niryo Studio / local ROS debugging

## Note

Switching between Pi mode and laptop mode requires changing the `NTP=` line on
the robot, because the robot does not automatically know which host should be
its time source.
