#include "pouring_controller/pour_server.hpp"
#include "pouring_controller/pid_vibration.hpp"
#include "pouring_controller/flow_rate_vibration.hpp"
#include "pouring_controller/pid_inflight_vibration.hpp"
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
  this->declare_parameter<double>("coarse_threshold", 0.40);
  this->declare_parameter<double>("fine_threshold", 0.05);
  this->declare_parameter<double>("start_in_fine_below_g", 80.0);
  this->declare_parameter<double>("start_in_trickle_below_g", 10.0);
  this->declare_parameter<double>("settle_time_s", 2.0);
  this->declare_parameter<int>("hold_within_tol_count", 5);
  this->declare_parameter<double>("final_settle_time_s", 2.0);
  this->declare_parameter<double>("min_progress_g", 0.5);
  this->declare_parameter<double>("no_progress_timeout_s", 3.0);
  this->declare_parameter<double>("no_progress_incline_step_deg", 5.0);
  this->declare_parameter<double>("max_incline_deg", 20.0);
  this->declare_parameter<std::string>("tilt_joint_name", "");
  this->declare_parameter<std::string>("traj_action_server", "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
  this->declare_parameter<double>("coarse_tilt_deg", 15.0);
  this->declare_parameter<double>("fine_tilt_deg", 0.0);
  this->declare_parameter<double>("trickle_tilt_deg", 0.0);
  this->declare_parameter<double>("joint_move_time_s", 0.5);
  this->declare_parameter<std::string>("control_law_type", "bangbang"); // bangbang|pid|pid_smooth|pid_flow|pid_flow_80|pid_inflight
  this->declare_parameter<double>("coarse_vibration_intensity", 0.9);
  this->declare_parameter<double>("settle_vibration_intensity", 0.0);
  this->declare_parameter<double>("fine_vibration_intensity", 0.70);
  this->declare_parameter<double>("trickle_vibration_intensity", 0.5);
  // Global PID/inflight command ceiling (smooth start; was hard-coded 1.0).
  this->declare_parameter<double>("vibration_cmd_max", 0.7);
  // Below this intensity powder often does not move — bump nonzero cmds up to it.
  this->declare_parameter<double>("min_pour_vibration", 0.40);
  this->declare_parameter<double>("trickle_pulse_ms", 180.0);
  this->declare_parameter<double>("trickle_pause_ms", 160.0);
  this->declare_parameter<double>("pid_kp", 0.7);
  this->declare_parameter<double>("pid_ki", 0.05);
  this->declare_parameter<double>("pid_kd", 0.0);
  this->declare_parameter<double>("pid_feedforward_intensity", 0.0);
  this->declare_parameter<double>("pid_integral_limit", 5.0);
  // Absolute grams of error that map to normalized error = 1.0 for PID.
  // Fixed scale (not target-relative) so a 56 g rescoop top-up is gentler than
  // a 500 g first pour — target-relative would re-saturate every new goal.
  this->declare_parameter<double>("pid_error_norm_g", 100.0);
  // pid_smooth only: wider norm so duty eases with remaining error.
  // pid_slew_per_s defaults to 0 (no software ramp).
  this->declare_parameter<double>("pid_smooth_error_norm_g", 250.0);
  this->declare_parameter<double>("pid_slew_per_s", 0.0);
  this->declare_parameter<double>("pid_smooth_min_pour", 0.20);
  // Unused by the named laws. pid_flow is the whole-pour cascade (0 g).
  // pid_flow_80 is PidSmooth until 80 g remain, then the cascade.
  this->declare_parameter<double>("pid_flow_endgame_below_g", 0.0);
  this->declare_parameter<double>("pid_flow_window_s", 0.6);
  this->declare_parameter<double>("pid_flow_land_time_s", 2.0);
  this->declare_parameter<double>("pid_flow_flow_max_g_s", 8.0);
  this->declare_parameter<double>("pid_flow_stop_margin_g", 0.5);
  this->declare_parameter<double>("pid_flow_kp", 0.05);
  this->declare_parameter<double>("pid_flow_ki", 0.02);
  this->declare_parameter<double>("pid_flow_u_thresh_init", 0.20);
  this->declare_parameter<double>("pid_flow_gain_init_g_s", 8.0);
  this->declare_parameter<double>("pid_flow_gain_alpha", 0.15);
  this->declare_parameter<double>("pid_flow_stall_g_s", 0.3);
  this->declare_parameter<double>("pid_flow_stall_time_s", 1.0);
  this->declare_parameter<double>("pid_flow_seek_rate", 0.06);
  this->declare_parameter<double>("pid_flow_seek_max_duty", 0.70);
  this->declare_parameter<double>("pid_flow_seek_taper_g", 15.0);
  this->declare_parameter<double>("pid_flow_exhausted_time_s", 1.5);
  this->declare_parameter<double>("pid_flow_ramp_up_rate", 0.08);
  this->declare_parameter<double>("pid_flow_ramp_down_rate", 0.40);
  this->declare_parameter<double>("pid_flow_dither_amp", 0.0);
  this->declare_parameter<double>("inflight_s", 0.80);
  this->declare_parameter<double>("inflight_flow_gain", 8.0);
  this->declare_parameter<double>("inflight_flow_gain_alpha", 0.15);
  this->declare_parameter<double>("inflight_early_stop_margin_g", 0.5);
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
  no_progress_incline_step_deg_ = this->get_parameter("no_progress_incline_step_deg").as_double();
  max_incline_deg_ = this->get_parameter("max_incline_deg").as_double();
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
  vibration_cmd_max_ = std::clamp(
    this->get_parameter("vibration_cmd_max").as_double(), 0.0, 1.0);
  min_pour_vibration_ = std::clamp(
    this->get_parameter("min_pour_vibration").as_double(), 0.0, 1.0);
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
  health_ = std::make_unique<rhapsodi_common_cpp::HealthEventPublisher>(this, "pour_server");

  // control plugin selection
  const auto law = this->get_parameter("control_law_type").as_string();
  // pid_flow: cascade for the whole pour. pid_flow_80: PidSmooth until 80 g left.
  const bool flow_whole = (law == "pid_flow");
  const bool flow_80 = (law == "pid_flow_80");
  const bool flow_law = flow_whole || flow_80;
  smooth_pour_ = (law == "pid_smooth" || flow_law);
  flow_pour_ = flow_law;
  if (law == "pid_inflight") {
    control_ = std::make_shared<PidInflightVibration>();
  } else if (flow_law) {
    control_ = std::make_shared<PidFlow>();
  } else if (law == "pid_smooth") {
    control_ = std::make_shared<PidSmooth>();
  } else if (law == "pid") {
    control_ = std::make_shared<PidVibration>();
  } else {
    control_ = std::make_shared<BangBangTrickle>();
  }
  control_->configure(0.0, vibration_cmd_max_);
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
    pid->error_norm_g = this->get_parameter("pid_error_norm_g").as_double();
  }
  if (auto* smooth = dynamic_cast<PidSmooth*>(control_.get())) {
    smooth->error_norm_g = this->get_parameter("pid_smooth_error_norm_g").as_double();
    smooth->slew_per_s = this->get_parameter("pid_slew_per_s").as_double();
    smooth->min_pour = std::clamp(
      this->get_parameter("pid_smooth_min_pour").as_double(), 0.0, 1.0);
  }
  if (auto* flow = dynamic_cast<PidFlow*>(control_.get())) {
    // The law name is the version recorded on the run. Do not let the
    // endgame param relabel one version as the other.
    flow->endgame_below_g = flow_80 ? 80.0 : 0.0;
    flow->window_s = this->get_parameter("pid_flow_window_s").as_double();
    flow->land_time_s = this->get_parameter("pid_flow_land_time_s").as_double();
    flow->flow_max_g_s = this->get_parameter("pid_flow_flow_max_g_s").as_double();
    flow->stop_margin_g = this->get_parameter("pid_flow_stop_margin_g").as_double();
    flow->kp_flow = this->get_parameter("pid_flow_kp").as_double();
    flow->ki_flow = this->get_parameter("pid_flow_ki").as_double();
    flow->u_thresh_init = std::clamp(
      this->get_parameter("pid_flow_u_thresh_init").as_double(), 0.0, 1.0);
    flow->gain_init = this->get_parameter("pid_flow_gain_init_g_s").as_double();
    flow->gain_alpha = this->get_parameter("pid_flow_gain_alpha").as_double();
    flow->stall_g_s = this->get_parameter("pid_flow_stall_g_s").as_double();
    flow->stall_time_s = this->get_parameter("pid_flow_stall_time_s").as_double();
    flow->seek_rate = this->get_parameter("pid_flow_seek_rate").as_double();
    flow->seek_max_duty = std::clamp(
      this->get_parameter("pid_flow_seek_max_duty").as_double(), 0.0, 1.0);
    flow->seek_taper_g = this->get_parameter("pid_flow_seek_taper_g").as_double();
    flow->exhausted_time_s = this->get_parameter("pid_flow_exhausted_time_s").as_double();
    flow->ramp_up_rate = this->get_parameter("pid_flow_ramp_up_rate").as_double();
    flow->ramp_down_rate = this->get_parameter("pid_flow_ramp_down_rate").as_double();
    flow->dither_amp = std::max(0.0, this->get_parameter("pid_flow_dither_amp").as_double());
  }
  if (auto* pid_if = dynamic_cast<PidInflightVibration*>(control_.get())) {
    pid_if->kp = this->get_parameter("pid_kp").as_double();
    pid_if->ki = this->get_parameter("pid_ki").as_double();
    pid_if->kd = this->get_parameter("pid_kd").as_double();
    pid_if->ff_bias = this->get_parameter("pid_feedforward_intensity").as_double();
    pid_if->integ_limit = this->get_parameter("pid_integral_limit").as_double();
    pid_if->error_norm_g = this->get_parameter("pid_error_norm_g").as_double();
    pid_if->inflight_s = this->get_parameter("inflight_s").as_double();
    pid_if->flow_gain = this->get_parameter("inflight_flow_gain").as_double();
    pid_if->flow_gain_alpha = this->get_parameter("inflight_flow_gain_alpha").as_double();
    pid_if->early_stop_margin_g =
      this->get_parameter("inflight_early_stop_margin_g").as_double();
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Pour control_law_type=%s vibration_cmd_max=%.2f min_pour_vibration=%.2f "
    "pid_smooth_min_pour=%.2f pid_flow_endgame_below_g=%.1f pid_flow_seek_max_duty=%.2f "
    "pid_flow_ramp_up_rate=%.2f pid_flow_ramp_down_rate=%.2f",
    law.c_str(),
    vibration_cmd_max_,
    min_pour_vibration_,
    std::clamp(this->get_parameter("pid_smooth_min_pour").as_double(), 0.0, 1.0),
    flow_80 ? 80.0 : 0.0,
    std::clamp(this->get_parameter("pid_flow_seek_max_duty").as_double(), 0.0, 1.0),
    this->get_parameter("pid_flow_ramp_up_rate").as_double(),
    this->get_parameter("pid_flow_ramp_down_rate").as_double());

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
  // 0 disables incline-before-rescoop. Do not rewrite 0 to the 5° default:
  // the cell sets NO_PROGRESS_INCLINE_STEP_DEG=0 so a stall goes straight to rescoop.
  const double configured_incline_step = std::max(0.0, no_progress_incline_step_deg_);
  const double configured_max_incline =
    max_incline_deg_ > 0.0 ? max_incline_deg_ : 20.0;
  RCLCPP_INFO(
    get_logger(),
    "Pour incline recovery: step=%.3fdeg max=%.3fdeg coarse_base=%.3fdeg fine_base=%.3fdeg trickle_base=%.3fdeg",
    configured_incline_step,
    configured_max_incline,
    coarse_tilt_deg_,
    fine_tilt_deg_,
    trickle_tilt_deg_);
  int within_tol_count = 0;
  double progress_reference_net_g = 0.0;
  auto progress_reference_time = now();
  auto phase_enter_time = now();
  int no_progress_incline_boosts = 0;
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
      health_->error(
        "pour_stale_weight_abort",
        "Pour aborted: no fresh weight reading (stale_ms=" + std::to_string(stale_ms_) + ")",
        "{\"stale_ms\":" + std::to_string(stale_ms_) + ",\"target_weight\":" +
          std::to_string(goal->target_weight) + "}");
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

    // phase transitions. pid_smooth and pid_flow stay in one regime so vibration
    // is not stopped or recapped on a coarse/settle/fine/trickle boundary.
    Phase old_phase = phase;
    static rclcpp::Time settle_start; // track settle start
    if (!smooth_pour_) {
    switch (phase) {
      case COARSE:
        if (abs_err <= coarse_band) {
          phase = SETTLE;
          settle_start = now();
        }
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
    }
    std::string phase_name = "coarse";
    if (flow_pour_) {
      phase_name = "flow";
    } else if (smooth_pour_) {
      phase_name = "smooth";
    } else if (phase == SETTLE) {
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

    auto base_incline_for_phase = [&]() {
      switch (phase) {
        case COARSE:
          return coarse_tilt_deg_;
        case SETTLE:
          return fine_tilt_deg_;
        case FINE:
          return fine_tilt_deg_;
        case TRICKLE:
          return trickle_tilt_deg_;
      }
      return 0.0;
    };

    const double max_incline = configured_max_incline;
    const double incline_step = configured_incline_step;
    const double base_incline_deg = base_incline_for_phase();
    const double incline_deg = std::clamp(
      base_incline_deg + (no_progress_incline_boosts * incline_step),
      0.0,
      max_incline);

    // control law
    ControlContext ctx; ctx.target_weight = goal->target_weight; ctx.tolerance = goal->tolerance;
    ctx.filtered_weight = net_g; ctx.raw_weight = net_g; ctx.phase = phase_name;
    ctx.dt_s = 1.0 / std::max(1.0, sample_rate_hz_);
    ControlCommand cmd = control_->update(ctx);
    double vibration_intensity = std::clamp(cmd.vibration_duty, 0.0, vibration_cmd_max_);
    if (!smooth_pour_) {
    if (phase == COARSE) {
      // Cap (same as FINE/TRICKLE) — previously max() floored at coarse and
      // forced a hard 0.8–1.0 blast at pour start for PID laws.
      vibration_intensity = std::min(
        vibration_intensity, std::clamp(coarse_vibration_intensity_, 0.0, 1.0));
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
    // Escape the no-flow deadband: nonzero but sub-threshold cmds rarely move powder.
    constexpr double kCmdEps = 0.02;
    if (vibration_intensity > kCmdEps &&
        vibration_intensity < min_pour_vibration_ &&
        phase != SETTLE) {
      vibration_intensity = min_pour_vibration_;
    }
    }  // phase caps / settle stop / trickle pulse — skipped for pid_smooth and pid_flow
    send_cmd(vibration_intensity, 0.0, incline_deg);

    // PidFlow already ramped to its seek cap and powder still did not move.
    // That is stronger evidence the scoop is empty than the no-progress timer,
    // which would also fire during the deliberate seek.
    if (cmd.scoop_empty) {
      stop_cmd();
      publish_status(false, "", 0.0, 0.0, 0.0);
      result->achieved = false;
      result->timeout = true;
      result->overshoot = false;
      result->final_weight = static_cast<float>(raw_weight_);
      result->final_net_g = static_cast<float>(net_g);
      result->need_rescoop = true;
      result->proceed_next = false;
      result->message = "Scoop empty";
      RCLCPP_WARN(
        get_logger(),
        "Pour scoop empty: phase=%s net=%.3fg duty=%.2f (baseline=%.3f)",
        phase_name.c_str(),
        net_g,
        vibration_intensity,
        baseline_g);
      health_->warn(
        "pour_scoop_empty",
        "Pour seek reached cap with no flow, needs rescoop",
        "{\"phase\":\"" + phase_name + "\",\"net_g\":" + std::to_string(net_g) +
          ",\"duty\":" + std::to_string(vibration_intensity) + "}");
      goal_handle->succeed(result);
      return;
    }

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
      health_->warn(
        "pour_overshoot",
        "Pour overshot target by " + std::to_string(result->final_net_g - goal->target_weight) + "g",
        "{\"target_weight\":" + std::to_string(goal->target_weight) +
          ",\"final_net_g\":" + std::to_string(result->final_net_g) +
          ",\"baseline_g\":" + std::to_string(baseline_g) + "}");
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
        const double next_incline_deg = std::clamp(
          base_incline_deg + ((no_progress_incline_boosts + 1) * incline_step),
          0.0,
          max_incline);
        if (incline_step > 0.0 && next_incline_deg > incline_deg + 1e-6) {
          ++no_progress_incline_boosts;
          progress_reference_net_g = net_g;
          progress_reference_time = now();
          RCLCPP_WARN(
            get_logger(),
            "Pour no-progress: boosting incline to %.3fdeg (boost %d, max %.3fdeg) before rescoop",
            next_incline_deg,
            no_progress_incline_boosts,
            max_incline);
          continue;
        }
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
          "Pour no-progress timeout: phase=%s net=%.3fg progress_window=%.3fg dt=%.2fs "
          "incline=%.3fdeg step=%.3fdeg max=%.3fdeg (baseline=%.3f)",
          phase_name.c_str(),
          net_g,
          net_g - progress_reference_net_g,
          (now() - progress_reference_time).seconds(),
          incline_deg,
          incline_step,
          max_incline,
          baseline_g);
        health_->warn(
          "pour_no_progress_timeout",
          "Pour stalled: no progress for " + std::to_string(no_progress_timeout_s_) +
            "s, needs rescoop",
          "{\"phase\":\"" + phase_name + "\",\"net_g\":" + std::to_string(net_g) +
            ",\"incline_deg\":" + std::to_string(incline_deg) + "}");
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
            health_->error(
              "pour_stale_weight_during_settle",
              "Pour aborted during final settle: weight went stale",
              "{\"target_weight\":" + std::to_string(goal->target_weight) + "}");
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
      health_->warn(
        "pour_max_time_timeout",
        "Pour exceeded max_time_s=" + std::to_string(goal->max_time_s) + "s",
        "{\"target_weight\":" + std::to_string(goal->target_weight) +
          ",\"final_net_g\":" + std::to_string(result->final_net_g) + "}");
      goal_handle->succeed(result);
      return;
    }

    rate.sleep();
  }
}

} // namespace pouring_controller


