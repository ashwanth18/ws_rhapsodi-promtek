// Single source of truth for edge-device identity (C++ side).
//
// Mirrors src/rhapsodi_common/rhapsodi_common/device_config.py: reads
// config/device.yaml (or RHAPSODI_DEVICE_CONFIG) and falls back to the
// historical hardcoded "robot-1" single-robot defaults if nothing is
// found or the file fails to parse. Never throws.
#pragma once

#include <optional>
#include <string>

namespace rhapsodi_common_cpp
{

struct DeviceConfig
{
  std::string device_id;
  std::string robot_id;
  std::string robot_type;
  std::string site_id;
  std::string processing_url;
  std::string ingestion_url;
  bool is_fallback {true};
};

// Never throws: any missing file / parse error falls back to the same
// historical defaults as the Python loader, so identity loading is never
// the reason a node fails to start.
DeviceConfig loadDeviceConfig(const std::optional<std::string> & explicit_path = std::nullopt);

}  // namespace rhapsodi_common_cpp
