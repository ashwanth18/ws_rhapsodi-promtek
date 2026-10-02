#include "pouring_controller/flow_rate_vibration.hpp"

#include <algorithm>
#include <cmath>

namespace pouring_controller {

void PidFlow::configure(double min_cmd, double max_cmd)
{
  PidSmooth::configure(min_cmd, max_cmd);
  min_cmd_flow_ = min_cmd;
  max_cmd_flow_ = max_cmd;
}

void PidFlow::reset()
{
  PidSmooth::reset();
  hist_.clear();
  t_ = 0.0;
  cmd_u_ = 0.0;
  u_hat_ = std::clamp(u_thresh_init, 0.0, 1.0);
  gain_hat_ = std::max(0.5, gain_init);
  stall_acc_ = 0.0;
  at_cap_acc_ = 0.0;
  flow_integ_ = 0.0;
  dither_phase_ = 0.0;
  dither_sign_ = 1.0;
  seeking_ = false;
}

double PidFlow::seekCap(double err_g) const
{
  const double hi = std::min(std::max(0.0, seek_max_duty), max_cmd_flow_);
  const double frac = std::clamp(err_g / std::max(1.0, seek_taper_g), 0.0, 1.0);
  return u_hat_ + frac * std::max(0.0, hi - u_hat_);
}

void PidFlow::noteMass(double mass)
{
  hist_.push_back(MassSample{t_, mass});
  const double horizon = std::max(0.05, window_s);
  while (hist_.size() > 2 && (t_ - hist_[1].t) >= horizon) {
    hist_.pop_front();
  }
}

double PidFlow::estimateFlow(bool & valid) const
{
  valid = false;
  if (hist_.size() < 2) {
    return 0.0;
  }
  const double span = hist_.back().t - hist_.front().t;
  if (span < 0.8 * std::max(0.05, window_s)) {
    return 0.0;
  }
  valid = true;
  return (hist_.back().mass - hist_.front().mass) / span;
}

double PidFlow::slewToward(double target, double dt) const
{
  const double dts = std::max(0.0, dt);
  const double down = ramp_down_rate <= 0.0
    ? std::abs(target - cmd_u_)
    : ramp_down_rate * dts;
  const double up = ramp_up_rate <= 0.0
    ? std::abs(target - cmd_u_)
    : ramp_up_rate * dts;
  return std::clamp(target, cmd_u_ - down, cmd_u_ + up);
}

ControlCommand PidFlow::update(const ControlContext & ctx)
{
  const double dt = std::max(1e-3, ctx.dt_s);
  t_ += dt;
  noteMass(ctx.filtered_weight);

  const double err_g = ctx.target_weight - ctx.filtered_weight;
  // Optional split. endgame_below_g <= 0 keeps this cascade for the whole pour.
  if (endgame_below_g > 0.0 && err_g > endgame_below_g) {
    seeking_ = false;
    stall_acc_ = 0.0;
    at_cap_acc_ = 0.0;
    ControlCommand cmd = PidSmooth::update(ctx);
    cmd_u_ = cmd.vibration_duty;
    cmd.scoop_empty = false;
    return cmd;
  }

  bool flow_valid = false;
  const double flow = estimateFlow(flow_valid);

  ControlCommand cmd;
  cmd.valve_open = 0.0;
  cmd.incline_deg = incline_fixed_deg;
  cmd.scoop_empty = false;

  if (err_g <= std::max(0.0, ctx.tolerance)) {
    seeking_ = false;
    stall_acc_ = 0.0;
    at_cap_acc_ = 0.0;
    flow_integ_ = 0.0;
    cmd_u_ = 0.0;
    cmd.vibration_duty = 0.0;
    return cmd;
  }

  const double flow_ref = std::clamp(
    (err_g - stop_margin_g) / std::max(1e-3, land_time_s),
    0.0,
    std::max(0.0, flow_max_g_s));

  if (seeking_) {
    const double cap = seekCap(err_g);
    cmd_u_ = std::min(cap, cmd_u_ + std::max(0.0, seek_rate) * dt);
    const bool at_cap = cmd_u_ >= cap - 1e-3;
    if (at_cap) {
      at_cap_acc_ += dt;
    } else {
      at_cap_acc_ = 0.0;
    }
    const bool flowing = flow_valid && flow >= stall_g_s;
    if (flowing) {
      const double alpha = std::clamp(gain_alpha, 0.0, 1.0);
      const double hi = std::min(std::max(0.0, seek_max_duty), max_cmd_flow_);
      u_hat_ = std::clamp((1.0 - alpha) * u_hat_ + alpha * cmd_u_, 0.0, hi);
      seeking_ = false;
      stall_acc_ = 0.0;
      at_cap_acc_ = 0.0;
      flow_integ_ = 0.0;
      // Drop off the seek peak immediately. The next ticks slew from here
      // toward the cascade command, so a broken bridge does not keep dumping.
      cmd_u_ = std::clamp(u_hat_ + std::max(0.0, resume_margin), min_cmd_flow_, max_cmd_flow_);
    } else if (flow_valid && at_cap && at_cap_acc_ >= std::max(0.0, exhausted_time_s)) {
      cmd.scoop_empty = true;
    }
  } else if (flow_valid && flow < stall_g_s) {
    stall_acc_ += dt;
    if (stall_acc_ >= std::max(0.0, stall_time_s) && flow_ref > 0.0) {
      seeking_ = true;
      at_cap_acc_ = 0.0;
      flow_integ_ = 0.0;
    }
  } else {
    stall_acc_ = 0.0;
    const double flow_meas = flow_valid ? flow : 0.0;
    const double flow_err = flow_ref - flow_meas;
    if (flow_valid) {
      flow_integ_ += flow_err * dt;
      flow_integ_ = std::clamp(flow_integ_, -flow_integ_limit_, flow_integ_limit_);
    }
    const double gain = std::max(0.5, gain_hat_);
    const double u_ff = u_hat_ + (flow_ref / gain) + kp_flow * flow_err + ki_flow * flow_integ_;
    cmd_u_ = slewToward(std::clamp(u_ff, min_cmd_flow_, max_cmd_flow_), dt);

    const double excess = cmd_u_ - u_hat_;
    if (flow_valid && flow >= stall_g_s && excess > 0.05) {
      const double g_obs = std::clamp(flow / excess, 1.0, 40.0);
      const double alpha = std::clamp(gain_alpha, 0.0, 1.0);
      gain_hat_ = (1.0 - alpha) * gain_hat_ + alpha * g_obs;
    }
  }

  cmd_u_ = std::clamp(cmd_u_, min_cmd_flow_, max_cmd_flow_);
  double out = cmd_u_;
  if (dither_amp > 0.0 && !seeking_ && out > 0.02 && !cmd.scoop_empty) {
    dither_phase_ += dt;
    if (dither_phase_ >= 0.25) {
      dither_phase_ -= 0.25;
      dither_sign_ = -dither_sign_;
    }
    out += dither_sign_ * dither_amp;
    out = std::clamp(out, min_cmd_flow_, max_cmd_flow_);
  }
  cmd.vibration_duty = out;
  return cmd;
}

}  // namespace pouring_controller
