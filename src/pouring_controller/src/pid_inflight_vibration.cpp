#include "pouring_controller/pid_inflight_vibration.hpp"

#include <cmath>

namespace pouring_controller {

void PidInflightVibration::configure(double min_cmd, double max_cmd)
{
  min_ = min_cmd;
  max_ = max_cmd;
}

void PidInflightVibration::reset()
{
  integ_ = 0.0;
  prev_err_ = 0.0;
  last_u_ = 0.0;
  last_weight_ = 0.0;
  have_weight_ = false;
  cmd_hist_.clear();
  aged_u_dt_ = 0.0;
  aged_weight_ref_ = 0.0;
  aged_weight_ref_valid_ = false;
}

ControlCommand PidInflightVibration::update(const ControlContext & ctx)
{
  const double dt = std::max(1e-3, ctx.dt_s);
  const double theta = std::max(0.05, inflight_s);
  const double w = ctx.filtered_weight;

  // Age command history; accumulate u*dt that just left the in-flight window.
  double just_landed_u_dt = 0.0;
  for (auto & s : cmd_hist_) {
    s.age += dt;
  }
  while (!cmd_hist_.empty() && cmd_hist_.front().age >= theta) {
    just_landed_u_dt += cmd_hist_.front().u * cmd_hist_.front().dt;
    cmd_hist_.pop_front();
  }

  // Adapt flow_gain when previously commanded mass should now be on the scale.
  if (just_landed_u_dt > 1e-4 && have_weight_) {
    if (!aged_weight_ref_valid_) {
      aged_weight_ref_ = last_weight_;
      aged_weight_ref_valid_ = true;
      aged_u_dt_ = 0.0;
    }
    aged_u_dt_ += just_landed_u_dt;
    // Wait until we have a meaningful delayed command integral, then update.
    if (aged_u_dt_ >= 0.05) {
      const double dw = std::max(0.0, w - aged_weight_ref_);
      const double observed_gain = dw / aged_u_dt_;  // g per (u·s) == g/s at u=1
      if (observed_gain > 0.1) {
        const double clipped = std::clamp(observed_gain, flow_gain_min, flow_gain_max);
        flow_gain = (1.0 - flow_gain_alpha) * flow_gain + flow_gain_alpha * clipped;
      }
      aged_weight_ref_ = w;
      aged_u_dt_ = 0.0;
    }
  }

  // Pending mass still in transit from commands younger than theta.
  double pending_g = 0.0;
  for (const auto & s : cmd_hist_) {
    pending_g += flow_gain * s.u * s.dt;
  }
  // Also include the last issued command's contribution for this tick's hold
  // (last_u_ was applied after the previous update).
  pending_g = std::max(0.0, pending_g);

  const double predicted = w + pending_g;
  const double err_g = ctx.target_weight - predicted;
  const double norm = std::max(1e-3, error_norm_g);
  const double err = err_g / norm;

  // Hard early-stop when predicted mass already meets target (minus margin).
  ControlCommand cmd;
  cmd.valve_open = 0.0;
  cmd.incline_deg = incline_fixed_deg;

  if (ctx.phase == "settle" || err_g <= early_stop_margin_g) {
    integ_ *= 0.5;  // bleed integral when stopped / settling
    last_u_ = 0.0;
    cmd.vibration_duty = 0.0;
  } else {
    integ_ += err * dt;
    integ_ = std::clamp(integ_, -integ_limit, integ_limit);
    const double deriv = (err - prev_err_) / dt;
    prev_err_ = err;
    double u = ff_bias + kp * err + ki * integ_ + kd * deriv;
    u = std::clamp(u, min_, max_);
    last_u_ = u;
    cmd.vibration_duty = u;
  }

  cmd_hist_.push_back(CmdSample{last_u_, dt, 0.0});
  // Bound history length (~2x delay).
  double age_span = 0.0;
  for (auto it = cmd_hist_.rbegin(); it != cmd_hist_.rend(); ++it) {
    age_span += it->dt;
  }
  while (age_span > theta * 2.5 + 0.5 && !cmd_hist_.empty()) {
    age_span -= cmd_hist_.front().dt;
    cmd_hist_.pop_front();
  }

  last_weight_ = w;
  have_weight_ = true;
  return cmd;
}

}  // namespace pouring_controller
