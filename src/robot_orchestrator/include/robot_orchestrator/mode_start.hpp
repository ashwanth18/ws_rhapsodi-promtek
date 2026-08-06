#pragma once

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_orchestrator {

/// Mode-specific blackboard keys that must not leak across runs.
inline const std::vector<std::string> kWebhookBlackboardKeys = {
  "webhook_run_id",
  "webhook_weightment_id",
  "webhook_batch_id",
  "webhook_ingredient_id",
  "webhook_location_id",
  "webhook_location_code",
  "webhook_pickup_target_name",
  "webhook_weigh_target_name",
  "webhook_return_target_name",
  "webhook_target_weight_g",
  "webhook_tolerance_g",
  "webhook_expected_lot",
};

inline const std::vector<std::string> kLightsoutBlackboardKeys = {
  "lightsout_powder_id",
  "lightsout_powder_name",
  "lightsout_container_name",
  "lightsout_pour_target",
  "lightsout_lot_code",
  "lightsout_operator",
  "lightsout_notes",
  "lightsout_stop_on",
  "lightsout_stop_value",
  "lightsout_stop_requested",
  "lightsout_target_mode",
  "lightsout_fixed_target_g",
  "lightsout_target_fractions_csv",
  "lightsout_target_weight_g",
  "lightsout_target_fraction",
  "lightsout_tolerance_g",
  "lightsout_min_scooped_g",
  "lightsout_target_min_g",
  "lightsout_target_max_g",
  "lightsout_rng_seed",
  "lightsout_episodes",
  "lightsout_batch_id",
  "lightsout_episode_index",
  "scooped_mass_g",
  "lightsout_pour_outcome",
  "enable_scoop",
  "lightsout_purge_enabled",
  "lightsout_purge_incline_deg",
  "lightsout_purge_vibration",
  "lightsout_purge_duration_s",
  "lightsout_purge_target",
  "lightsout_purge_pose_enabled",
};

inline const std::vector<std::string> kBatchBlackboardKeys = {
  "containers",
  "container_index",
  "container_name",
  "expected_lot",
  "ingredients_q",
  "task",
};

/// Unset prior-mode keys so a new start cannot inherit leaked state.
inline void clearModeBlackboardKeys(const BT::Blackboard::Ptr & bb)
{
  for (const auto & key : kWebhookBlackboardKeys) {
    bb->unset(key);
  }
  for (const auto & key : kLightsoutBlackboardKeys) {
    bb->unset(key);
  }
  for (const auto & key : kBatchBlackboardKeys) {
    bb->unset(key);
  }
  bb->unset("phase_topic");
}

/// Register every *.xml under directory (fail-fast on parse/register errors).
inline void registerBehaviorTreesFromDirectory(
  BT::BehaviorTreeFactory & factory,
  const std::filesystem::path & directory)
{
  if (!std::filesystem::is_directory(directory)) {
    throw std::runtime_error(
      "BT trees directory missing or not a directory: " + directory.string());
  }
  std::vector<std::filesystem::path> xml_files;
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".xml") {
      xml_files.push_back(entry.path());
    }
  }
  std::sort(xml_files.begin(), xml_files.end());
  if (xml_files.empty()) {
    throw std::runtime_error(
      "No BehaviorTree XML files found in: " + directory.string());
  }
  for (const auto & path : xml_files) {
    factory.registerBehaviorTreeFromFile(path);
  }
}

inline std::string treeIdHintFromPath(const std::filesystem::path & path)
{
  const std::string stem = path.stem().string();
  if (stem == "webhook_weightment") {
    return "WebhookWeightment";
  }
  if (stem == "lightsout") {
    return "LightsOut";
  }
  if (stem == "main") {
    return "Main";
  }
  if (stem == "scoop_weigh_pour") {
    return "ScoopWeighPour";
  }
  return stem;
}

}  // namespace robot_orchestrator
