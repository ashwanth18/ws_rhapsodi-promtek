#pragma once

#include "pouring_controller/control_law.hpp"

namespace pouring_controller {

class PidVibration : public ControlLaw {
public:
  void configure(double min_cmd, double max_cmd) override { min_=min_cmd; max_=max_cmd; }
  void reset() override { integ_=0.0; prev_err_=0.0; }
  ControlCommand update(const ControlContext & ctx) override;

  // Gains apply to error normalized by error_norm_g (see update()).
  // With kp=0.7 and error_norm_g=100, P≈0.7 at 100 g remaining — not at 100% of
  // this goal's target (target-relative would re-max on every rescoop top-up).
  double kp{0.7}, ki{0.05}, kd{0.0};
  double ff_bias{0.0};
  double incline_fixed_deg{0.0};
  double integ_limit{5.0};
  /** Grams of error that map to a unit normalized error (default 100 g). */
  double error_norm_g{100.0};

private:
  double min_{0.0}, max_{1.0};
  double integ_{0.0};
  double prev_err_{0.0};
};

// Continuous PID. pour_server does not apply settle-to-zero, phase caps, or
// trickle pulsing. Duty is sent as computed. slew_per_s defaults to 0 (no
// software ramp); set it only if the drive should be rate-limited here.
class PidSmooth : public PidVibration {
public:
  void configure(double min_cmd, double max_cmd) override;
  void reset() override;
  ControlCommand update(const ControlContext & ctx) override;

  /** Max |du/dt| in duty per second. 0 disables the software ramp. */
  double slew_per_s{0.0};
  /** Floor applied to the slew target while error remains, not to each tick. */
  double min_pour{0.40};

private:
  double min_cmd_{0.0};
  double max_cmd_{1.0};
  double last_u_{0.0};
};

} // namespace pouring_controller













