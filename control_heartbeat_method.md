# Control Heartbeat Method

This document describes how the dashboard detects micro-ROS connectivity using a
heartbeat topic and a timeout.

## Overview

- The micro-ROS device publishes a heartbeat topic at 1 Hz.
- The dashboard subscribes to that topic via rosbridge.
- The UI marks the device as **connected** if the last heartbeat is newer than
  3 seconds, otherwise **disconnected**.

## Device-Side Requirement

Publish `std_msgs/msg/Empty` at 1 Hz:

- Topic: `/microros/heartbeat`
- Rate: 1 Hz (or faster)

If the device stops publishing, the dashboard flips to **disconnected** after
3 seconds.

## Dashboard Implementation

Location:

- `src/dashboard/src/pages/ControlsPage.tsx`

Key logic:

- Subscribe to `/microros/heartbeat` (`std_msgs/Empty`) and update
  `lastMicroRosHeartbeat` on each message.
- A 1-second timer updates a local clock so timeouts re-evaluate even when no
  new messages arrive.
- Status rule:
  - `connected` if `now - lastHeartbeat < 3000ms`
  - else fall back to lifecycle/graph status, and show `disconnected` if stale

Topic override:

- Set `VITE_MICROROS_HEARTBEAT_TOPIC` to use a custom heartbeat topic.

## Why This Works

DDS discovery and topic presence can linger after disconnects. The heartbeat
checks *freshness* rather than topic existence, so it reliably detects when the
device stops sending data.

