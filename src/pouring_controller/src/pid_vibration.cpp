#include "pouring_controller/pid_vibration.hpp"
#include <algorithm>

namespace pouring_controller {

ControlCommand PidVibration::update(const ControlContext & ctx)
{
  const double err = ctx.target_weight - ctx.filtered_weight;
  const double dt = std::max(1e-3, ctx.dt_s);
  integ_ += err * dt;
  integ_ = std::clamp(integ_, -integ_limit, integ_limit);
  const double deriv = (err - prev_err_) / dt;
  prev_err_ = err;

  double u = kp * err + ki * integ_ + kd * deriv;
  u = std::clamp(u, min_, max_);

  ControlCommand cmd;
  cmd.vibration_duty = u;
  cmd.valve_open = std::clamp(valve_bias, 0.0, 1.0);
  cmd.incline_deg = incline_fixed_deg;
  return cmd;
}

} // namespace pouring_controller













