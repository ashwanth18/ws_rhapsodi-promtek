#pragma once

#include "pouring_controller/control_law.hpp"

namespace pouring_controller {

class BangBangTrickle : public ControlLaw {
public:
  void configure(double min_cmd, double max_cmd) override { (void)min_cmd; (void)max_cmd; }
  void reset() override {}
  ControlCommand update(const ControlContext & ctx) override;

  double coarse_open{0.9};
  double fine_open{0.5};
  double trickle_open{0.2};
  double coarse_threshold{0.2};
  double fine_threshold{0.05};
  double incline_deg{0.0};
};

} // namespace pouring_controller













