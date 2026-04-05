#include "pouring_controller/pour_server.hpp"
#include "pouring_controller/pid_vibration.hpp"
#include "pouring_controller/bangbang_trickle.hpp"

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

namespace pouring_controller {

PourServer::PourServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("pour_server", options)
{
  // parameters
  this->declare_parameter<std::string>("weight_topic", "/weight");
  this->declare_parameter<std::string>("vibration_topic", "/vibration/intensity");
  this->declare_parameter<std::string>("valve_topic", "/valve_control");
  this->declare_parameter<std::string>("incline_topic", "/incline_control");
  this->declare_parameter<double>("ema_alpha", 0.2);
  this->declare_parameter<double>("sample_rate_hz", 12.0);
  this->declare_parameter<double>("stale_ms", 500.0);
  this->declare_parameter<double>("coarse_threshold", 0.10);
  this->declare_parameter<double>("fine_threshold", 0.02);
  this->declare_parameter<double>("start_in_fine_below_g", 40.0);
  this->declare_parameter<double>("start_in_trickle_below_g", 10.0);
  this->declare_parameter<double>("settle_time_s", 2.0);
  this->declare_parameter<int>("hold_within_tol_count", 5);
  this->declare_parameter<double>("final_settle_time_s", 2.0);
  this->declare_parameter<double>("min_progress_g", 0.5);
  this->declare_parameter<double>("no_progress_timeout_s", 2.0);
  this->declare_parameter<std::string>("tilt_joint_name", "");
  this->declare_parameter<std::string>("traj_action_server", "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
  this->declare_parameter<double>("coarse_tilt_deg", 0.0);
  this->declare_parameter<double>("fine_tilt_deg", 0.0);
  this->declare_parameter<double>("trickle_tilt_deg", 0.0);
  this->declare_parameter<double>("joint_move_time_s", 0.5);
  this->declare_parameter<std::string>("control_law_type", "bangbang"); // pid|bangbang
  this->declare_parameter<double>("coarse_vibration_intensity", 0.75);
  this->declare_parameter<double>("settle_vibration_intensity", 0.0);
  this->declare_parameter<double>("fine_vibration_intensity", 0.40);
  this->declare_parameter<double>("trickle_vibration_intensity", 0.15);
  this->declare_parameter<double>("trickle_pulse_ms", 120.0);
  this->declare_parameter<double>("trickle_pause_ms", 180.0);
  this->declare_parameter<double>("pid_kp", 0.02);
  this->declare_parameter<double>("pid_ki", 0.001);
  this->declare_parameter<double>("pid_kd", 0.0);
  this->declare_parameter<double>("pid_feedforward_intensity", 0.0);
  this->declare_parameter<double>("pid_integral_limit", 200.0);
  this->declare_parameter<std::string>("joint_state_topic", "/joint_states");

  ema_alpha_ = this->get_parameter("ema_alpha").as_double();
  sample_rate_hz_ = this->get_parameter("sample_rate_hz").as_double();
  stale_ms_ = this->get_parameter("stale_ms").as_double();
  coarse_thresh_ = this->get_parameter("coarse_threshold").as_double();
  fine_thresh_ = this->get_parameter("fine_threshold").as_double();
  start_in_fine_below_g_ = this->get_parameter("start_in_fine_below_g").as_double();
  start_in_trickle_below_g_ = this->get_parameter("start_in_trickle_below_g").as_double();
  settle_time_s_ = this->get_parameter("settle_time_s").as_double();
  hold_within_tol_count_ = this->get_parameter("hold_within_tol_count").as_int();
  final_settle_time_s_ = this->get_parameter("final_settle_time_s").as_double();
  min_delta_g_ = this->get_parameter("min_progress_g").as_double();
  no_progress_timeout_s_ = this->get_parameter("no_progress_timeout_s").as_double();
  tilt_joint_name_ = this->get_parameter("tilt_joint_name").as_string();
  traj_action_server_ = this->get_parameter("traj_action_server").as_string();
  coarse_tilt_deg_ = this->get_parameter("coarse_tilt_deg").as_double();
  fine_tilt_deg_ = this->get_parameter("fine_tilt_deg").as_double();
  trickle_tilt_deg_ = this->get_parameter("trickle_tilt_deg").as_double();
  joint_move_time_s_ = this->get_parameter("joint_move_time_s").as_double();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  coarse_vibration_intensity_ = this->get_parameter("coarse_vibration_intensity").as_double();
  settle_vibration_intensity_ = this->get_parameter("settle_vibration_intensity").as_double();
  fine_vibration_intensity_ = this->get_parameter("fine_vibration_intensity").as_double();
  trickle_vibration_intensity_ = this->get_parameter("trickle_vibration_intensity").as_double();
  trickle_pulse_ms_ = this->get_parameter("trickle_pulse_ms").as_double();
  trickle_pause_ms_ = this->get_parameter("trickle_pause_ms").as_double();

  auto wt = this->get_parameter("weight_topic").as_string();
  auto vt = this->get_parameter("vibration_topic").as_string();
  auto valvet = this->get_parameter("valve_topic").as_string();
  auto it = this->get_parameter("incline_topic").as_string();

  weight_sub_ = this->create_subscription<std_msgs::msg::Float64>(wt, 10, std::bind(&PourServer::weightCb, this, std::placeholders::_1));
  vibration_pub_ = this->create_publisher<std_msgs::msg::Float64>(vt, 10);
  valve_pub_ = this->create_publisher<std_msgs::msg::Float64>(valvet, 10);
  incline_pub_ = this->create_publisher<std_msgs::msg::Float64>(it, 10);
  pour_status_pub_ = this->create_publisher<robot_common_msgs::msg::PourStatus>("/pour_status", 10);

  // control plugin selection
  const auto law = this->get_parameter("control_law_type").as_string();
  if (law == "pid") {
    control_ = std::make_shared<PidVibration>();
  } else {
    control_ = std::make_shared<BangBangTrickle>();
  }
  control_->configure(0.0, 1.0);
  if (auto* bangbang = dynamic_cast<BangBangTrickle*>(control_.get())) {
    bangbang->coarse_open = std::clamp(coarse_vibration_intensity_, 0.0, 1.0);
    bangbang->fine_open = std::clamp(fine_vibration_intensity_, 0.0, 1.0);
    bangbang->trickle_open = std::clamp(trickle_vibration_intensity_, 0.0, 1.0);
  }
  if (auto* pid = dynamic_cast<PidVibration*>(control_.get())) {
    pid->kp = this->get_parameter("pid_kp").as_double();
    pid->ki = this->get_parameter("pid_ki").as_double();
    pid->kd = this->get_parameter("pid_kd").as_double();
    pid->ff_bias = this->get_parameter("pid_feedforward_intensity").as_double();
    pid->integ_limit = this->get_parameter("pid_integral_limit").as_double();
  }

  // Create action server without shared_from_this() (avoid bad_weak_ptr in constructor)
  action_server_ = rclcpp_action::create_server<PourToTarget>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "pour_to_target",
    std::bind(&PourServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PourServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&PourServer::handle_accepted, this, std::placeholders::_1));

  // Subscribe to joint states to build full joint vector
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 100,
      std::bind(&PourServer::onJointState, this, std::placeholders::_1));

  // Disable trajectory action client: pouring_controller will not command joints
  // (tilt disabled to avoid conflicting with MoveIt / Niryo driver)
}

// Tilt commands removed: this controller no longer sends joint trajectories

void PourServer::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(joint_mutex_);
  last_joint_names_ = msg->name;
  last_joint_positions_ = msg->position;
}

void PourServer::weightCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  raw_weight_ = msg->data;
  last_weight_stamp_ = now();
}

rclcpp_action::GoalResponse PourServer::handle_goal(const rclcpp_action::GoalUUID &,
                                                    std::shared_ptr<const PourToTarget::Goal> goal)
{
  if (goal->target_weight <= 0.0f || goal->tolerance <= 0.0f) {
    RCLCPP_WARN(get_logger(), "Rejecting pour goal: invalid target/tolerance");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PourServer::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  // Ensure actuators are stopped on cancel
  std_msgs::msg::Float64 mi; mi.data = 0.0;
  vibration_pub_->publish(mi);
  // Motion outputs disabled (valve/incline)
  RCLCPP_INFO(get_logger(), "Cancel request received");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PourServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&PourServer::execute, this, goal_handle)}.detach();
}

void PourServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PourToTarget::Feedback>();
  auto result = std::make_shared<PourToTarget::Result>();
  auto phase_str = [](int p){
    switch (p) {
      case 0: return "COARSE";
      case 1: return "SETTLE";
      case 2: return "FINE";
      case 3: return "TRICKLE";
    }
    return "?";
  };

  rclcpp::Rate rate( sample_rate_hz_ );
  const auto start = now();
  filtered_weight_ = raw_weight_;
  // Capture baseline at goal start (RAW absolute scale reading)
  const double baseline_g = raw_weight_;
  RCLCPP_INFO(get_logger(), "Pour start: target=%.3f tol=%.3f baseline=%.3f",
              goal->target_weight, goal->tolerance, baseline_g);
  int within_tol_count = 0;
  double progress_reference_net_g = 0.0;
  auto progress_reference_time = now();
  auto phase_enter_time = now();
  // Compute percentage-based error bands relative to target
  const double coarse_band = std::abs(goal->target_weight) * coarse_thresh_;
  const double fine_band = std::abs(goal->target_weight) * fine_thresh_;
  enum Phase { COARSE, SETTLE, FINE, TRICKLE } phase = COARSE;
  const double abs_target_g = std::abs(goal->target_weight);
  if (abs_target_g <= std::max(0.0, start_in_trickle_below_g_)) {
    phase = TRICKLE;
  } else if (abs_target_g <= std::max(0.0, start_in_fine_below_g_)) {
    phase = FINE;
  }
  RCLCPP_INFO(
    get_logger(),
    "Pour phase start: target=%.3fg coarse_band=%.3fg fine_band=%.3fg initial_phase=%s thresholds(fine<=%.3fg trickle<=%.3fg)",
    goal->target_weight,
    coarse_band,
    fine_band,
    phase_str(static_cast<int>(phase)),
    start_in_fine_below_g_,
    start_in_trickle_below_g_);
  control_->reset();
  auto send_cmd = [&](double vibration_intensity, double valve, double incline){
    std_msgs::msg::Float64 ms;
    ms.data = std::clamp(vibration_intensity, 0.0, 1.0);
    vibration_pub_->publish(ms);
    std_msgs::msg::Float64 m;
    m.data = valve; valve_pub_->publish(m);
    m.data = incline; incline_pub_->publish(m);
  };
  auto stop_cmd = [&]{ send_cmd(0.0, 0.0, 0.0); };
  auto publish_status = [&](bool active, const std::string& phase_str_name, double target_g, double band_g, double abs_err_val){
    robot_common_msgs::msg::PourStatus ps;
    ps.active = active;
    ps.phase = phase_str_name;
    const double rem_g = (band_g > 0.0) ? std::max(0.0, abs_err_val - band_g) : -1.0;
    ps.target_g = static_cast<float>(target_g);
    ps.band_threshold_g = static_cast<float>(band_g);
    ps.remaining_to_band_g = static_cast<float>(rem_g);
    pour_status_pub_->publish(ps);
  };

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_cmd();
      publish_status(false, "", 0.0, 0.0, 0.0);
      result->achieved = false;
      result->timeout = false;
      result->overshoot = false;
      result->final_weight = 0.0f;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    // staleness check
    if ((now() - last_weight_stamp_).nanoseconds() / 1e6 > stale_ms_) {
      stop_cmd();
      publish_status(false, "", 0.0, 0.0, 0.0);
      result->achieved = false;
      result->timeout = false;
      result->overshoot = false;
      result->final_weight = filtered_weight_;
      result->message = "Stale weight";
      goal_handle->abort(result);
      return;
    }

    // No filtering: use RAW weight directly
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      filtered_weight_ = raw_weight_;
    }
    // Net poured relative to baseline (RAW)
    const double net_g = std::max(0.0, raw_weight_ - baseline_g);
    const double err = goal->target_weight - net_g;
    const double abs_err = std::abs(err);
    static bool first_iter_logged = false;
    if (!first_iter_logged) {
      RCLCPP_INFO(get_logger(), "First iter: raw=%.3f net=%.3f err=%.3f abs_err=%.3f",
                  raw_weight_, net_g, err, abs_err);
      first_iter_logged = true;
    }

    // phase transitions
    Phase old_phase = phase;
    static rclcpp::Time settle_start; // track settle start
    switch (phase) {
      case COARSE:
        if (abs_err <= coarse_band) { phase = SETTLE; settle_start = now(); }
        break;
      case SETTLE:
        // wait settle_time
        if ((now() - settle_start).seconds() > settle_time_s_) { phase = FINE; }
        break;
      case FINE:
        if (abs_err <= fine_band) { phase = TRICKLE; }
        break;
      case TRICKLE:
        break;
    }
    std::string phase_name = "coarse";
    if (phase == SETTLE) {
      phase_name = "settle";
    } else if (phase == FINE) {
      phase_name = "fine";
    } else if (phase == TRICKLE) {
      phase_name = "trickle";
    }

    if (phase != old_phase) {
      RCLCPP_INFO(get_logger(), "Phase %s -> %s (abs_err=%.4f, coarse_band=%.4f, fine_band=%.4f)",
                  phase_str(static_cast<int>(old_phase)), phase_str(static_cast<int>(phase)), abs_err, coarse_band, fine_band);
      progress_reference_net_g = net_g;
      progress_reference_time = now();
      phase_enter_time = now();
    }

    // Joint tilt disabled

    // control law
    ControlContext ctx; ctx.target_weight = goal->target_weight; ctx.tolerance = goal->tolerance;
    ctx.filtered_weight = net_g; ctx.raw_weight = net_g; ctx.phase = phase_name;
    ctx.dt_s = 1.0 / std::max(1.0, sample_rate_hz_);
    ControlCommand cmd = control_->update(ctx);
    double vibration_intensity = std::clamp(cmd.vibration_duty, 0.0, 1.0);
    if (phase == COARSE) {
      vibration_intensity = std::max(vibration_intensity, std::clamp(coarse_vibration_intensity_, 0.0, 1.0));
    } else if (phase == SETTLE) {
      vibration_intensity = std::clamp(settle_vibration_intensity_, 0.0, 1.0);
    } else if (phase == FINE) {
      vibration_intensity = std::min(vibration_intensity, std::clamp(fine_vibration_intensity_, 0.0, 1.0));
    } else if (phase == TRICKLE) {
      vibration_intensity = std::min(vibration_intensity, std::clamp(trickle_vibration_intensity_, 0.0, 1.0));
      const double cycle_ms = std::max(0.0, trickle_pulse_ms_) + std::max(0.0, trickle_pause_ms_);
      if (cycle_ms > 0.0) {
        const double phase_elapsed_ms = (now() - phase_enter_time).seconds() * 1000.0;
        const double cycle_offset_ms = std::fmod(std::max(0.0, phase_elapsed_ms), cycle_ms);
        if (cycle_offset_ms >= std::max(0.0, trickle_pulse_ms_)) {
          vibration_intensity = 0.0;
        }
      }
    }
    send_cmd(vibration_intensity, 0.0, 0.0);

    // feedback
    feedback->current_weight = static_cast<float>(raw_weight_);
    feedback->phase = phase_name;
    // Error to next phase band and band threshold are reported in grams.
    float err_to_band = -1.0f;
    float band_thresh = 0.0f;
    if (phase == COARSE) {
      err_to_band = static_cast<float>(std::max(0.0, abs_err - coarse_band));
      band_thresh = static_cast<float>(coarse_band);
    } else if (phase == FINE) {
      err_to_band = static_cast<float>(std::max(0.0, abs_err - fine_band));
      band_thresh = static_cast<float>(fine_band);
    } else {
      err_to_band = -1.0f;
      band_thresh = 0.0f;
    }
    feedback->error_to_next_band = err_to_band;
    feedback->band_threshold = band_thresh;
    // hold time remaining when within tolerance
    if (abs_err <= goal->tolerance) {
      const int remaining_counts = std::max(0, hold_within_tol_count_ - within_tol_count);
      feedback->hold_time_remaining = static_cast<float>(remaining_counts / std::max(1.0, sample_rate_hz_));
    } else {
      feedback->hold_time_remaining = -1.0f;
    }
    goal_handle->publish_feedback(feedback);

    // UI status publish (active)
    double band_for_phase = 0.0;
    if (phase == COARSE) band_for_phase = coarse_band; else if (phase == FINE) band_for_phase = fine_band;
    publish_status(true, phase_name, goal->target_weight, band_for_phase, abs_err);

    // termination checks (net-based)
    if (net_g > goal->target_weight + goal->tolerance) {
      stop_cmd();
      // Post-stop settle window for overshoot as well
      const auto stop_t = now();
      while ((now() - stop_t).seconds() < final_settle_time_s_) {
        if ((now() - last_weight_stamp_).nanoseconds() / 1e6 > stale_ms_) {
          break; // don't block forever on stale sensor
        }
        {
          std::lock_guard<std::mutex> lk(data_mutex_);
          filtered_weight_ = ema_alpha_ * raw_weight_ + (1.0 - ema_alpha_) * filtered_weight_;
        }
        rate.sleep();
      }
      publish_status(false, "", 0.0, 0.0, 0.0);
      const double final_net = std::max(0.0, raw_weight_ - baseline_g);
      result->achieved = false;
      result->timeout = false;
      result->overshoot = true;
      result->final_weight = static_cast<float>(raw_weight_);
      result->final_net_g = static_cast<float>(final_net);
      result->need_rescoop = false;
      result->proceed_next = true;
      result->message = "Overshoot";
      // No tilt reset (disabled)
      RCLCPP_INFO(get_logger(), "Pour overshoot: final_abs=%.3fg final_net=%.3fg (baseline=%.3f)",
                  result->final_weight, result->final_net_g, baseline_g);
      goal_handle->succeed(result);
      return;
    }

    const bool progress_watchdog_active =
      no_progress_timeout_s_ > 0.0 && (phase == COARSE || phase == FINE);
    if (progress_watchdog_active) {
      if ((net_g - progress_reference_net_g) >= min_delta_g_) {
        progress_reference_net_g = net_g;
        progress_reference_time = now();
      } else if ((now() - progress_reference_time).seconds() > no_progress_timeout_s_) {
        stop_cmd();
        publish_status(false, "", 0.0, 0.0, 0.0);
        result->achieved = false;
        result->timeout = true;
        result->overshoot = false;
        result->final_weight = static_cast<float>(raw_weight_);
        result->final_net_g = static_cast<float>(net_g);
        result->need_rescoop = true;
        result->proceed_next = false;
        result->message = "No progress timeout";
        RCLCPP_WARN(
          get_logger(),
          "Pour no-progress timeout: phase=%s net=%.3fg progress_window=%.3fg dt=%.2fs (baseline=%.3f)",
          phase_name.c_str(),
          net_g,
          net_g - progress_reference_net_g,
          (now() - progress_reference_time).seconds(),
          baseline_g);
        goal_handle->succeed(result);
        return;
      }
    }

    if (abs_err <= goal->tolerance) {
      within_tol_count++;
      if (within_tol_count >= hold_within_tol_count_) {
        stop_cmd();
        // Post-stop settle window
        const auto stop_t = now();
        while ((now() - stop_t).seconds() < final_settle_time_s_) {
          // keep filtering; ensure weight not stale
          if ((now() - last_weight_stamp_).nanoseconds() / 1e6 > stale_ms_) {
            result->achieved = false;
            result->timeout = false;
            result->overshoot = false;
            result->final_weight = static_cast<float>(raw_weight_);
            result->final_net_g = static_cast<float>(std::max(0.0, raw_weight_ - baseline_g));
            result->need_rescoop = true;
            result->proceed_next = false;
            result->message = "Stale weight during final settle";
            goal_handle->abort(result);
            return;
          }
          // No filtering: rely on raw updates
          rate.sleep();
        }
        result->achieved = true;
        result->timeout = false;
        result->overshoot = false;
        result->final_weight = static_cast<float>(raw_weight_);
        result->final_net_g = static_cast<float>(std::max(0.0, raw_weight_ - baseline_g));
        result->need_rescoop = false;
        result->proceed_next = true;
        result->message = "Success";
        // No tilt reset (disabled)
        RCLCPP_INFO(get_logger(), "Pour result: final_abs=%.3fg final_net=%.3fg (baseline=%.3f)",
                    result->final_weight, result->final_net_g, baseline_g);
        goal_handle->succeed(result);
        publish_status(false, "", 0.0, 0.0, 0.0);
        return;
      }
    } else {
      within_tol_count = 0;
    }

    if ((now() - start).seconds() > goal->max_time_s) {
      stop_cmd();
      publish_status(false, "", 0.0, 0.0, 0.0);
      result->achieved = false;
      result->timeout = true;
      result->overshoot = false;
      result->final_weight = static_cast<float>(raw_weight_);
      result->final_net_g = static_cast<float>(std::max(0.0, raw_weight_ - baseline_g));
      result->need_rescoop = true;
      result->proceed_next = false;
      result->message = "Timeout";
      // No tilt reset (disabled)
      RCLCPP_INFO(get_logger(), "Pour timeout: final_abs=%.3fg final_net=%.3fg (baseline=%.3f)",
                  result->final_weight, result->final_net_g, baseline_g);
      goal_handle->succeed(result);
      return;
    }

    rate.sleep();
  }
}

} // namespace pouring_controller


