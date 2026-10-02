#pragma once

#include "pouring_controller/pid_vibration.hpp"

#include <deque>

namespace pouring_controller {

/**
 * Cascade on flow. endgame_below_g == 0 (law name pid_flow) runs the cascade
 * for the whole pour. endgame_below_g == 80 (law name pid_flow_80) runs
 * PidSmooth until that many grams remain, then the cascade.
 *
 * Vibration-to-flow has a deadband: below u* nothing moves, and u* drifts as
 * the bed bridges or empties. A mass PID commanded into that band stalls.
 * This law estimates flow from a mass window, runs PI on flow error around
 * u*_hat, and when flow stays at zero ramps duty up until powder moves. If the
 * ramp hits the seek cap and flow is still zero, scoop_empty is set so
 * pour_server can rescoop without waiting out the no-progress timer.
 * The seek cap shrinks with remaining mass so the last few grams are not
 * chased at full duty.
 */
class PidFlow : public PidSmooth {
public:
  void configure(double min_cmd, double max_cmd) override;
  void reset() override;
  ControlCommand update(const ControlContext & ctx) override;

  double threshold_hat() const { return u_hat_; }
  double gain_hat() const { return gain_hat_; }

  /** If > 0, PidSmooth runs while remaining grams exceed this. 0 = cascade always. */
  double endgame_below_g{0.0};
  /** Sliding window for the flow estimate (seconds). */
  double window_s{0.6};
  /** Outer loop: flow_ref = (err_g - stop_margin_g) / land_time_s. */
  double land_time_s{2.0};
  double flow_max_g_s{8.0};
  double stop_margin_g{0.5};
  /** Inner PI on flow error, in duty per (g/s) and duty per (g/s·s). */
  double kp_flow{0.05};
  double ki_flow{0.02};
  /** Starting vibration threshold and g/s per unit duty above it. */
  double u_thresh_init{0.20};
  double gain_init{8.0};
  /** EMA blend for u*_hat on stall recovery and for gain_hat while flowing. */
  double gain_alpha{0.15};
  double stall_g_s{0.3};
  double stall_time_s{1.0};
  /** Stall-escape ramp (duty per second) and its ceiling. */
  double seek_rate{0.06};
  double seek_max_duty{0.70};
  /**
   * Full seek_max_duty while err_g is above this. Below it the cap falls
   * linearly to u*_hat so a small remainder is not chased at full duty.
   */
  double seek_taper_g{15.0};
  /** Time at the seek cap with no flow before scoop_empty. */
  double exhausted_time_s{1.5};
  /** Max duty increase per second while tracking the cascade. 0 disables it. */
  double ramp_up_rate{0.08};
  /** Max duty decrease per second while tracking the cascade. 0 disables it. */
  double ramp_down_rate{0.40};
  /** Added to u*_hat on the tick flow resumes, instead of staying at the seek peak. */
  double resume_margin{0.05};
  /** Square dither around the cascade command. 0 disables it. */
  double dither_amp{0.0};

private:
  struct MassSample {
    double t{0.0};
    double mass{0.0};
  };

  double seekCap(double err_g) const;
  double estimateFlow(bool & valid) const;
  void noteMass(double mass);
  double slewToward(double target, double dt) const;

  double min_cmd_flow_{0.0};
  double max_cmd_flow_{1.0};
  double t_{0.0};
  double cmd_u_{0.0};
  double u_hat_{0.20};
  double gain_hat_{8.0};
  double stall_acc_{0.0};
  double at_cap_acc_{0.0};
  double flow_integ_{0.0};
  double flow_integ_limit_{5.0};
  double dither_phase_{0.0};
  double dither_sign_{1.0};
  bool seeking_{false};
  std::deque<MassSample> hist_;
};

}  // namespace pouring_controller
