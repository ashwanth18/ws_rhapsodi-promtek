#include "pouring_controller/flow_rate_vibration.hpp"
#include "pouring_controller/pid_vibration.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace {

struct DelayedPlant {
  double threshold{0.30};
  double gain{10.0};
  double delay_s{0.80};
  double noise_amp{0.0};
  double mass{0.0};
  double t{0.0};
  unsigned noise_state{1};

  struct Packet {
    double arrive{0.0};
    double dm{0.0};
  };
  std::deque<Packet> packets;

  double noise()
  {
    noise_state = noise_state * 1103515245u + 12345u;
    const double unit = static_cast<double>((noise_state >> 16) & 0x7fff) / 32767.0;
    return (unit - 0.5) * 2.0 * noise_amp;
  }

  double step(double u, double dt)
  {
    const double flow = (u > threshold) ? gain * (u - threshold) : 0.0;
    packets.push_back(Packet{t + delay_s, flow * dt});
    double add = 0.0;
    while (!packets.empty() && packets.front().arrive <= t) {
      add += packets.front().dm;
      packets.pop_front();
    }
    if (noise_amp > 0.0) {
      add += noise();
    }
    mass = std::max(0.0, mass + add);
    t += dt;
    return mass;
  }
};

pouring_controller::ControlContext ctx_at(double target, double tol, double mass, double dt)
{
  pouring_controller::ControlContext ctx;
  ctx.target_weight = target;
  ctx.tolerance = tol;
  ctx.filtered_weight = mass;
  ctx.raw_weight = mass;
  ctx.phase = "flow";
  ctx.dt_s = dt;
  return ctx;
}

void match_smooth_gains(pouring_controller::PidSmooth & law)
{
  law.configure(0.0, 0.70);
  law.kp = 0.7;
  law.ki = 0.05;
  law.kd = 0.0;
  law.ff_bias = 0.0;
  law.integ_limit = 5.0;
  law.error_norm_g = 250.0;
  law.slew_per_s = 0.0;
  law.min_pour = 0.20;
  law.reset();
}

}  // namespace

TEST(PidFlow, BulkMatchesPidSmooth)
{
  pouring_controller::PidSmooth smooth;
  pouring_controller::PidFlow flow;
  match_smooth_gains(smooth);
  match_smooth_gains(flow);
  flow.endgame_below_g = 80.0;
  flow.reset();

  const double dt = 1.0 / 12.0;
  for (int i = 0; i < 240; ++i) {
    const double mass = i * 0.4;  // err stays above 80 g on a 500 g target
    auto ctx = ctx_at(500.0, 3.0, mass, dt);
    const auto a = smooth.update(ctx);
    const auto b = flow.update(ctx);
    EXPECT_DOUBLE_EQ(a.vibration_duty, b.vibration_duty) << "tick " << i;
    EXPECT_FALSE(b.scoop_empty);
  }
}

TEST(PidFlow, StallSeekRestoresFlowAndLearnsThreshold)
{
  pouring_controller::PidFlow law;
  law.configure(0.0, 0.70);
  law.endgame_below_g = 80.0;
  law.u_thresh_init = 0.20;
  law.gain_init = 30.0;
  law.gain_alpha = 0.5;
  law.kp_flow = 0.02;
  law.ki_flow = 0.0;
  law.window_s = 0.40;
  law.land_time_s = 8.0;
  law.flow_max_g_s = 3.0;
  law.stop_margin_g = 0.5;
  law.stall_g_s = 0.30;
  law.stall_time_s = 0.40;
  law.seek_rate = 0.20;
  law.seek_max_duty = 0.70;
  law.seek_taper_g = 15.0;
  law.exhausted_time_s = 2.0;
  law.ramp_up_rate = 0.50;
  law.ramp_down_rate = 0.50;
  law.resume_margin = 0.05;
  law.dither_amp = 0.0;
  law.reset();

  DelayedPlant plant;
  plant.threshold = 0.50;
  plant.gain = 12.0;
  plant.delay_s = 0.80;
  plant.noise_amp = 0.0;

  const double target = 40.0;
  const double tol = 3.0;
  const double dt = 1.0 / 12.0;
  double peak_duty = 0.0;
  double min_duty_after_flow = 1.0;
  double t_flow = -1.0;
  for (int i = 0; i < static_cast<int>(12.0 * 25.0); ++i) {
    auto ctx = ctx_at(target, tol, plant.mass, dt);
    const auto cmd = law.update(ctx);
    peak_duty = std::max(peak_duty, cmd.vibration_duty);
    plant.step(cmd.vibration_duty, dt);
    EXPECT_FALSE(cmd.scoop_empty) << "tick " << i << " mass " << plant.mass;
    if (cmd.scoop_empty) {
      break;
    }
    if (t_flow < 0.0 && plant.mass > 2.0) {
      t_flow = plant.t;
    }
    // Flow estimate lags the scale. Duty should leave the seek peak once that
    // estimate sees powder moving.
    if (t_flow >= 0.0 && plant.t > t_flow + law.window_s + 0.2) {
      min_duty_after_flow = std::min(min_duty_after_flow, cmd.vibration_duty);
    }
    if (plant.mass > target - tol) {
      break;
    }
  }

  EXPECT_GE(t_flow, 0.0);
  EXPECT_GT(plant.mass, 5.0);
  EXPECT_GT(peak_duty, plant.threshold);
  EXPECT_NEAR(law.threshold_hat(), plant.threshold, 0.20);
  EXPECT_LT(min_duty_after_flow, peak_duty - 0.02);
}

TEST(PidFlow, EmptyScoopSignalsInsideWatchdog)
{
  pouring_controller::PidFlow law;
  law.configure(0.0, 0.70);
  law.endgame_below_g = 80.0;
  law.u_thresh_init = 0.20;
  law.gain_init = 8.0;
  law.kp_flow = 0.05;
  law.ki_flow = 0.0;
  law.window_s = 0.30;
  law.land_time_s = 2.0;
  law.flow_max_g_s = 8.0;
  law.stall_g_s = 0.30;
  law.stall_time_s = 0.50;
  law.seek_rate = 0.40;
  law.seek_max_duty = 0.70;
  law.seek_taper_g = 15.0;
  law.exhausted_time_s = 0.80;
  law.ramp_up_rate = 2.0;
  law.ramp_down_rate = 2.0;
  law.reset();

  DelayedPlant plant;
  plant.threshold = 1.5;  // unreachable: scoop is empty
  plant.gain = 10.0;
  plant.delay_s = 0.80;
  plant.noise_amp = 0.02;

  const double dt = 1.0 / 12.0;
  bool signaled = false;
  double duty_at_signal = 0.0;
  double t_signal = 0.0;
  for (int i = 0; i < static_cast<int>(12.0 * 10.0); ++i) {
    auto ctx = ctx_at(40.0, 3.0, plant.mass, dt);
    const auto cmd = law.update(ctx);
    plant.step(cmd.vibration_duty, dt);
    if (cmd.scoop_empty) {
      signaled = true;
      duty_at_signal = cmd.vibration_duty;
      t_signal = plant.t;
      break;
    }
  }

  EXPECT_TRUE(signaled);
  EXPECT_GT(t_signal, law.stall_time_s);
  EXPECT_LT(t_signal, 8.0);
  EXPECT_NEAR(duty_at_signal, law.seek_max_duty, 0.05);
  EXPECT_LT(plant.mass, 1.0);
}

TEST(PidFlow, LandsInsideTolerance)
{
  pouring_controller::PidFlow law;
  law.configure(0.0, 0.70);
  law.endgame_below_g = 80.0;
  law.u_thresh_init = 0.18;
  law.gain_init = 12.0;
  law.gain_alpha = 0.10;
  law.kp_flow = 0.04;
  law.ki_flow = 0.0;
  law.window_s = 0.40;
  law.land_time_s = 3.0;
  law.flow_max_g_s = 4.0;
  law.stop_margin_g = 1.0;
  law.stall_g_s = 0.30;
  law.stall_time_s = 5.0;
  law.seek_rate = 0.10;
  law.seek_max_duty = 0.70;
  law.seek_taper_g = 15.0;
  law.exhausted_time_s = 1.5;
  law.ramp_up_rate = 0.80;
  law.ramp_down_rate = 0.80;
  law.dither_amp = 0.0;
  law.reset();

  DelayedPlant plant;
  plant.threshold = 0.18;
  plant.gain = 12.0;
  plant.delay_s = 0.25;
  plant.noise_amp = 0.0;

  const double target = 30.0;
  const double tol = 3.0;
  const double dt = 1.0 / 12.0;
  double peak_mass = 0.0;
  double last_duty = 1.0;
  for (int i = 0; i < static_cast<int>(12.0 * 40.0); ++i) {
    auto ctx = ctx_at(target, tol, plant.mass, dt);
    const auto cmd = law.update(ctx);
    last_duty = cmd.vibration_duty;
    plant.step(last_duty, dt);
    peak_mass = std::max(peak_mass, plant.mass);
    EXPECT_FALSE(cmd.scoop_empty);
    if (plant.mass >= target - tol && last_duty == 0.0) {
      break;
    }
  }

  EXPECT_GE(plant.mass, target - tol);
  EXPECT_LE(peak_mass, target + tol);
  EXPECT_DOUBLE_EQ(last_duty, 0.0);
}
