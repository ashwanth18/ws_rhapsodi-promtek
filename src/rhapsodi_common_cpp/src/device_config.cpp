#include "rhapsodi_common_cpp/device_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <fstream>
#include <vector>

namespace rhapsodi_common_cpp
{

namespace
{

constexpr const char * kEnvVar = "RHAPSODI_DEVICE_CONFIG";
constexpr const char * kFallbackDeviceId = "robot-1";
constexpr const char * kFallbackRobotId = "robot-1";
constexpr const char * kFallbackRobotType = "niryo";
constexpr const char * kFallbackSiteId = "site-1";
constexpr const char * kFallbackProcessingUrl = "http://localhost:8002/process";
constexpr const char * kFallbackIngestionUrl = "http://localhost:8011/ingest";

DeviceConfig fallbackConfig()
{
  DeviceConfig cfg;
  cfg.device_id = kFallbackDeviceId;
  cfg.robot_id = kFallbackRobotId;
  cfg.robot_type = kFallbackRobotType;
  cfg.site_id = kFallbackSiteId;
  cfg.processing_url = kFallbackProcessingUrl;
  cfg.ingestion_url = kFallbackIngestionUrl;
  cfg.is_fallback = true;
  return cfg;
}

std::vector<std::string> candidatePaths(const std::optional<std::string> & explicit_path)
{
  std::vector<std::string> candidates;
  if (explicit_path.has_value()) {
    candidates.push_back(*explicit_path);
  }
  if (const char * env = std::getenv(kEnvVar)) {
    if (env[0] != '\0') {
      candidates.emplace_back(env);
    }
  }
  // Standard in-container mount point used by docker-compose *.yml.
  candidates.emplace_back("/ws/config/device.yaml");
  candidates.emplace_back("config/device.yaml");
  return candidates;
}

}  // namespace

DeviceConfig loadDeviceConfig(const std::optional<std::string> & explicit_path)
{
  for (const auto & path : candidatePaths(explicit_path)) {
    std::ifstream probe(path);
    if (!probe.good()) {
      continue;
    }
    try {
      YAML::Node root = YAML::LoadFile(path);
      YAML::Node device = root["device"];
      if (!device || !device.IsMap()) {
        continue;
      }
      DeviceConfig cfg;
      cfg.device_id = device["device_id"] ? device["device_id"].as<std::string>() : kFallbackDeviceId;
      cfg.robot_id = device["robot_id"] ? device["robot_id"].as<std::string>() : kFallbackRobotId;
      cfg.robot_type = device["robot_type"] ? device["robot_type"].as<std::string>() : kFallbackRobotType;
      cfg.site_id = device["site_id"] ? device["site_id"].as<std::string>() : kFallbackSiteId;
      YAML::Node fleet = device["fleet"];
      if (fleet && fleet.IsMap()) {
        cfg.processing_url = fleet["processing_url"] ? fleet["processing_url"].as<std::string>() : kFallbackProcessingUrl;
        cfg.ingestion_url = fleet["ingestion_url"] ? fleet["ingestion_url"].as<std::string>() : kFallbackIngestionUrl;
      } else {
        cfg.processing_url = kFallbackProcessingUrl;
        cfg.ingestion_url = kFallbackIngestionUrl;
      }
      cfg.is_fallback = false;
      return cfg;
    } catch (const YAML::Exception &) {
      continue;
    }
  }
  return fallbackConfig();
}

}  // namespace rhapsodi_common_cpp
