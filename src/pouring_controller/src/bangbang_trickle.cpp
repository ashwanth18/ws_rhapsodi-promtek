#include "pouring_controller/bangbang_trickle.hpp"
#include <cmath>

namespace pouring_controller {

ControlCommand BangBangTrickle::update(const ControlContext & ctx)
{
  const double err = ctx.target_weight - ctx.filtered_weight; // kg remaining
  ControlCommand cmd;
  cmd.incline_deg = incline_deg;
  if (err > coarse_threshold) {
    cmd.vibration_duty = coarse_open;
    cmd.valve_open = coarse_open;
  } else if (err > fine_threshold) {
    cmd.vibration_duty = fine_open;
    cmd.valve_open = fine_open;
  } else if (err > 0.0) {
    // trickle; could duty-cycle externally
    cmd.vibration_duty = trickle_open;
    cmd.valve_open = trickle_open;
  } else {
    cmd.vibration_duty = 0.0;
    cmd.valve_open = 0.0;
  }
  return cmd;
}

} // namespace pouring_controller













