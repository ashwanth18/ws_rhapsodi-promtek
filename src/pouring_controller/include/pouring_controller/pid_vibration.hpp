#pragma once

#include "pouring_controller/control_law.hpp"

namespace pouring_controller {

class PidVibration : public ControlLaw {
public:
  void configure(double min_cmd, double max_cmd) override { min_=min_cmd; max_=max_cmd; }
  void reset() override { integ_=0.0; prev_err_=0.0; }
  ControlCommand update(const ControlContext & ctx) override;

  // Gains (public for quick param wiring)
  double kp{0.5}, ki{0.0}, kd{0.0};
  double valve_bias{0.0};
  double incline_fixed_deg{0.0};
  double integ_limit{1.0};

private:
  double min_{0.0}, max_{1.0};
  double integ_{0.0};
  double prev_err_{0.0};
};

} // namespace pouring_controller













