#pragma once

#include <string>

namespace pouring_controller {

struct ControlContext {
  double target_weight{0.0};
  double tolerance{0.0};
  double filtered_weight{0.0};
  double raw_weight{0.0};
  std::string phase; // "coarse", "fine", "trickle", "settle"
  double dt_s{0.02};
};

struct ControlCommand {
  double vibration_duty{0.0};   // 0..1
  double incline_deg{0.0};      // degrees
  double valve_open{0.0};       // 0..1
  // PidFlow sets this after the seek ramp hits its cap and flow is still
  // zero. pour_server turns it into need_rescoop without waiting for the
  // no-progress timer. Other laws leave it false.
  bool scoop_empty{false};
};

class ControlLaw {
public:
  virtual ~ControlLaw() = default;
  virtual void configure(double min_cmd, double max_cmd) = 0;
  virtual void reset() = 0;
  virtual ControlCommand update(const ControlContext & ctx) = 0;
};

} // namespace pouring_controller













