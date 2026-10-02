#include "pouring_controller/pid_vibration.hpp"
#include <algorithm>
#include <cmath>

namespace pouring_controller {

ControlCommand PidVibration::update(const ControlContext & ctx)
{
  // filtered_weight is net poured for this pour goal (pour_server).
  const double err_g = ctx.target_weight - ctx.filtered_weight;
  const double norm = std::max(1e-3, error_norm_g);
  const double err_n = err_g / norm;
  const double dt = std::max(1e-3, ctx.dt_s);
  integ_ += err_n * dt;
  integ_ = std::clamp(integ_, -integ_limit, integ_limit);
  const double deriv = (err_n - prev_err_) / dt;
  prev_err_ = err_n;

  double u = ff_bias + kp * err_n + ki * integ_ + kd * deriv;
  u = std::clamp(u, min_, max_);

  ControlCommand cmd;
  cmd.vibration_duty = u;
  cmd.valve_open = 0.0;
  cmd.incline_deg = incline_fixed_deg;
  return cmd;
}

void PidSmooth::configure(double min_cmd, double max_cmd)
{
  PidVibration::configure(min_cmd, max_cmd);
  min_cmd_ = min_cmd;
  max_cmd_ = max_cmd;
}

void PidSmooth::reset()
{
  PidVibration::reset();
  last_u_ = 0.0;
}

ControlCommand PidSmooth::update(const ControlContext & ctx)
{
  const double err_g = ctx.target_weight - ctx.filtered_weight;
  ControlCommand cmd = PidVibration::update(ctx);
  double target_u = cmd.vibration_duty;
  // Hold the flowing floor on the target, then slew toward it. Applying the
  // floor every tick would step through the ramp.
  if (err_g <= std::max(0.0, ctx.tolerance)) {
    target_u = 0.0;
  } else if (target_u > 0.02 && target_u < min_pour) {
    target_u = min_pour;
  }
  const double dt = std::max(1e-3, ctx.dt_s);
  const double step = std::max(0.0, slew_per_s) * dt;
  double u = target_u;
  if (step > 0.0) {
    u = std::clamp(target_u, last_u_ - step, last_u_ + step);
  }
  u = std::clamp(u, min_cmd_, max_cmd_);
  last_u_ = u;
  cmd.vibration_duty = u;
  return cmd;
}

} // namespace pouring_controller
