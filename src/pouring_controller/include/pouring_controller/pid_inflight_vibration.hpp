#pragma once

#include "pouring_controller/control_law.hpp"

#include <algorithm>
#include <deque>

namespace pouring_controller {

/**
 * PID on predicted scale weight: filtered + estimated in-flight mass.
 *
 * In-flight mass is the integral of (flow_gain * vibration_cmd * dt) over the
 * trailing `inflight_s` window — the transport delay from vibration command to
 * scale response (~0.8 s on the laptop Lexium cell).
 *
 * `flow_gain` (g/s at duty=1) adapts online from observed Δweight vs command
 * integral delayed by `inflight_s`.
 */
class PidInflightVibration : public ControlLaw {
public:
  void configure(double min_cmd, double max_cmd) override;
  void reset() override;
  ControlCommand update(const ControlContext & ctx) override;

  double kp{0.7};
  double ki{0.05};
  double kd{0.0};
  double ff_bias{0.0};
  double integ_limit{5.0};
  double incline_fixed_deg{0.0};
  /** Grams of error that map to a unit normalized error (see PidVibration). */
  double error_norm_g{100.0};

  /** Measured vib→weight transport delay (seconds). */
  double inflight_s{0.80};
  /** Initial / fallback flow rate at vibration duty = 1 (g/s). */
  double flow_gain{8.0};
  double flow_gain_min{2.0};
  double flow_gain_max{25.0};
  /** EMA blend when adapting flow_gain from delayed observations. */
  double flow_gain_alpha{0.15};
  /** Extra early-stop margin on predicted weight (grams). */
  double early_stop_margin_g{0.5};

private:
  struct CmdSample {
    double u{0.0};
    double dt{0.0};
    double age{0.0};
  };

  double min_{0.0};
  double max_{1.0};
  double integ_{0.0};
  double prev_err_{0.0};
  double last_u_{0.0};
  double last_weight_{0.0};
  bool have_weight_{false};
  std::deque<CmdSample> cmd_hist_;
  /** Integral of u*dt that has aged past inflight_s since last adapt. */
  double aged_u_dt_{0.0};
  double aged_weight_ref_{0.0};
  bool aged_weight_ref_valid_{false};
};

}  // namespace pouring_controller
