#pragma once

#include <behaviortree_cpp/blackboard.h>

#include <string>

namespace robot_orchestrator {

inline constexpr const char * kLastFailureReasonKey = "last_failure_reason";

inline void clearLastFailureReason(const BT::Blackboard::Ptr & bb)
{
  if (bb) {
    bb->set(kLastFailureReasonKey, std::string{});
  }
}

inline void setLastFailureReason(
  const BT::Blackboard::Ptr & bb, const std::string & reason)
{
  if (!bb || reason.empty()) {
    return;
  }
  bb->set(kLastFailureReasonKey, reason);
}

inline std::string getLastFailureReason(const BT::Blackboard::Ptr & bb)
{
  if (!bb) {
    return {};
  }
  try {
    return bb->get<std::string>(kLastFailureReasonKey);
  } catch (...) {
    return {};
  }
}

}  // namespace robot_orchestrator
