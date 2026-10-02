#include "scooping_controller/scoop_vision_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSignalBlocker>

#include <cmath>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace scooping_controller
{
namespace
{
constexpr const char* kOk = "#86efac";
constexpr const char* kBusy = "#93c5fd";
constexpr const char* kError = "#fca5a5";
constexpr float kMoveScaling = 0.2F;

QPushButton* makeButton(const QString& text, const QString& tip)
{
  auto* button = new QPushButton(text);
  button->setToolTip(tip);
  return button;
}
}  // namespace

ScoopVisionPanel::ScoopVisionPanel(QWidget* parent)
: rviz_common::Panel(parent), ros_timer_(new QTimer(this))
{
  auto* layout = new QVBoxLayout(this);

  auto* container_box = new QGroupBox("Container calibration (camera)");
  auto* container_row = new QHBoxLayout(container_box);
  check_button_ = makeButton(
    "Check", "Compare the bin rim seen by the D455 with the layout (arm out of view)");
  fit_button_ = makeButton(
    "Fit pose", "Fit the bin pose from the rim and write a layout proposal (orange bin in RViz)");
  apply_fit_button_ = makeButton(
    "Apply fit", "Write the fitted pose into the layout and re-apply it (scoop follows the bin)");
  container_row->addWidget(check_button_);
  container_row->addWidget(fit_button_);
  container_row->addWidget(apply_fit_button_);
  layout->addWidget(container_box);

  auto* scoop_box = new QGroupBox("Next scoop");
  auto* scoop_layout = new QVBoxLayout(scoop_box);
  auto* row1 = new QHBoxLayout();
  weighing_button_ = makeButton(
    "Arm to camera-clear",
    "MoveTo CameraClear: arm to the side, out of the camera's view of the bin, "
    "joints well inside their limits");
  capture_button_ = makeButton("Capture powder", "Fuse D455 frames into the powder height map");
  plan_button_ = makeButton("Plan next scoop", "Pick the best shift of the authored scoop");
  row1->addWidget(weighing_button_);
  row1->addWidget(capture_button_);
  row1->addWidget(plan_button_);
  auto* row2 = new QHBoxLayout();
  preview_button_ = makeButton(
    "Preview in MoveIt", "Plan the shifted scoop with MTC (Motion Planning Tasks display)");
  run_button_ = makeButton(
    "Run scoop", "Move to the scooping container and execute the planned scoop (moves the arm)");
  authored_button_ = makeButton(
    "Use authored scoop", "Reset pattern_offset_x/y/z to 0 on scooping_mtc_node");
  row2->addWidget(preview_button_);
  row2->addWidget(run_button_);
  row2->addWidget(authored_button_);
  auto* row3 = new QHBoxLayout();
  depth_correction_ = new QSpinBox();
  depth_correction_->setRange(0, 80);
  depth_correction_->setSingleStep(5);
  depth_correction_->setSuffix(" mm deeper");
  depth_correction_->setEnabled(false);
  depth_correction_->setToolTip(
    "Temporary calibration compensation (planner.surface_correction_m): the camera reads "
    "the powder this much too high, so plans go this much deeper. Set to 0 after "
    "recalibrating. Re-plan after changing it.");
  row3->addWidget(new QLabel("Depth correction:"));
  row3->addWidget(depth_correction_);
  row3->addStretch();
  scoop_layout->addLayout(row1);
  scoop_layout->addLayout(row2);
  scoop_layout->addLayout(row3);
  plan_label_ = new QLabel("No plan yet.");
  plan_label_->setWordWrap(true);
  scoop_layout->addWidget(plan_label_);
  layout->addWidget(scoop_box);

  status_label_ = new QLabel("Waiting for RViz panel initialization...");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  layout->addStretch();

  connect(check_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onCheckContainer);
  connect(fit_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onFitContainer);
  connect(apply_fit_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onApplyFit);
  connect(weighing_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onMoveToWeighing);
  connect(capture_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onCapture);
  connect(plan_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onPlan);
  connect(preview_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onPreview);
  connect(run_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onRun);
  connect(authored_button_, &QPushButton::clicked, this, &ScoopVisionPanel::onUseAuthored);
  connect(depth_correction_, &QSpinBox::editingFinished, this,
    &ScoopVisionPanel::onDepthCorrectionChanged);
  connect(ros_timer_, &QTimer::timeout, this, &ScoopVisionPanel::onRosTimer);
  refreshButtons();
}

void ScoopVisionPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("scoop_vision_rviz_panel");
  check_client_ = node_->create_client<Trigger>("/scoop_vision/check_container_alignment");
  fit_client_ = node_->create_client<Trigger>("/scoop_vision/fit_container_pose");
  apply_fit_client_ = node_->create_client<Trigger>("/scoop_vision/apply_container_fit");
  capture_client_ = node_->create_client<Trigger>("/scoop_vision/capture");
  plan_client_ = node_->create_client<PlanScoop>("/scoop_vision/plan");
  mtc_plan_client_ = node_->create_client<Trigger>("/plan_scoop");
  mtc_execute_client_ = node_->create_client<Trigger>("/execute_scoop_continuous");
  move_to_client_ = rclcpp_action::create_client<MoveTo>(node_, "/move_to");
  mtc_params_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/scooping_mtc_node");
  vision_params_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/scoop_vision");
  ros_timer_->start(50);
  setStatus("Ready. Arm to camera-clear, then Check / Capture.");
}

void ScoopVisionPanel::onRosTimer()
{
  if (!node_) {
    return;
  }
  rclcpp::spin_some(node_);
  // Show the node's current correction once it is up (it may start after RViz).
  if (!depth_correction_loaded_ && vision_params_->service_is_ready()) {
    depth_correction_loaded_ = true;
    vision_params_->get_parameters(
      {"planner.surface_correction_m"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        try {
          const auto params = future.get();
          if (!params.empty()) {
            const QSignalBlocker block(depth_correction_);
            depth_correction_applied_mm_ =
              static_cast<int>(std::lround(params[0].as_double() * 1000.0));
            depth_correction_->setValue(depth_correction_applied_mm_);
          }
          depth_correction_->setEnabled(true);
        } catch (const std::exception&) {
          depth_correction_loaded_ = false;
        }
      });
  }
}

void ScoopVisionPanel::onDepthCorrectionChanged()
{
  // editingFinished also fires on focus loss: only act on a real change, so
  // clicking another button does not silently drop the current plan.
  if (depth_correction_->value() == depth_correction_applied_mm_) {
    return;
  }
  if (!vision_params_->service_is_ready()) {
    setStatus("/scoop_vision parameters are not available.", kError);
    return;
  }
  const double metres = depth_correction_->value() / 1000.0;
  vision_params_->set_parameters(
    {rclcpp::Parameter("planner.surface_correction_m", metres)},
    [this, metres](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        const auto results = future.get();
        const bool ok = !results.empty() && results[0].successful;
        if (ok) {
          depth_correction_applied_mm_ = static_cast<int>(std::lround(metres * 1000.0));
          have_plan_ = false;
          plan_label_->setText("Depth correction changed: plan the next scoop again.");
          refreshButtons();
        }
        setStatus(
          ok ? QString("Depth correction set to %1 mm; re-plan.").arg(metres * 1000.0, 0, 'f', 0)
             : QString("Could not set depth correction."),
          ok ? kOk : kError);
      } catch (const std::exception& ex) {
        setStatus(QString("Could not set depth correction: %1").arg(ex.what()), kError);
      }
    });
}

void ScoopVisionPanel::setStatus(const QString& text, const QString& color)
{
  status_label_->setStyleSheet(QString("color: %1;").arg(color));
  status_label_->setText(text);
}

void ScoopVisionPanel::setBusy(bool busy)
{
  busy_ = busy;
  refreshButtons();
}

void ScoopVisionPanel::refreshButtons()
{
  for (auto* button : {check_button_, fit_button_, weighing_button_, capture_button_,
                       plan_button_, authored_button_})
  {
    button->setEnabled(!busy_);
  }
  apply_fit_button_->setEnabled(!busy_ && fit_pending_);
  preview_button_->setEnabled(!busy_ && have_plan_);
  run_button_->setEnabled(!busy_ && have_plan_);
}

void ScoopVisionPanel::callTrigger(
  const rclcpp::Client<Trigger>::SharedPtr& client,
  const QString& busy_text,
  std::function<void(bool, const std::string&)> done)
{
  if (!client || !client->service_is_ready()) {
    const std::string name = client ? client->get_service_name() : "service";
    setStatus(QString("%1 is not available.").arg(QString::fromStdString(name)), kError);
    if (done) {
      done(false, name + " is not available");
    }
    return;
  }
  setBusy(true);
  setStatus(busy_text, kBusy);
  client->async_send_request(
    std::make_shared<Trigger::Request>(),
    [this, done](rclcpp::Client<Trigger>::SharedFuture future) {
      bool ok = false;
      std::string message;
      try {
        const auto response = future.get();
        ok = response->success;
        message = response->message;
      } catch (const std::exception& ex) {
        message = std::string("Service call failed: ") + ex.what();
      }
      setBusy(false);
      setStatus(QString::fromStdString(message), ok ? kOk : kError);
      if (done) {
        done(ok, message);
      }
    });
}

void ScoopVisionPanel::onCheckContainer()
{
  callTrigger(check_client_, "Checking the bin rim against the layout...");
}

void ScoopVisionPanel::onFitContainer()
{
  callTrigger(fit_client_, "Fitting the bin pose from the camera...",
    [this](bool ok, const std::string& message) {
      // Only offer Apply when a proposal was written (not "No layout change needed").
      fit_pending_ = ok && message.find("Proposed") != std::string::npos;
      refreshButtons();
    });
}

void ScoopVisionPanel::onApplyFit()
{
  const auto answer = QMessageBox::question(
    this, "Apply container fit",
    "Write the camera-fitted bin pose into the layout and re-apply it?\n\n"
    "The authored scoop poses move with the bin (they are stored in the bin's frame). "
    "A backup of the old layout is kept next to the proposal.");
  if (answer != QMessageBox::Yes) {
    return;
  }
  callTrigger(apply_fit_client_, "Applying the fitted container pose...",
    [this](bool ok, const std::string&) {
      if (ok) {
        fit_pending_ = false;
        have_plan_ = false;
        plan_label_->setText("Layout changed: capture again before planning.");
      }
      refreshButtons();
    });
}

void ScoopVisionPanel::moveTo(
  const std::string& target, std::function<void(bool, const std::string&)> done)
{
  if (!move_to_client_->action_server_is_ready()) {
    done(false, "/move_to action server is not available");
    return;
  }
  MoveTo::Goal goal;
  goal.target_name = target;
  goal.velocity_scaling = kMoveScaling;
  goal.acceleration_scaling = kMoveScaling;
  goal.use_cartesian = false;
  rclcpp_action::Client<MoveTo>::SendGoalOptions options;
  options.goal_response_callback =
    [done, target](const rclcpp_action::ClientGoalHandle<MoveTo>::SharedPtr& handle) {
      if (!handle) {
        done(false, "MoveTo " + target + " rejected");
      }
    };
  options.result_callback =
    [done, target](const rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult& result) {
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->success;
      done(ok, "MoveTo " + target + ": " + (result.result ? result.result->message : ""));
    };
  move_to_client_->async_send_goal(goal, options);
}

void ScoopVisionPanel::onMoveToWeighing()
{
  setBusy(true);
  setStatus("Moving to CameraClear...", kBusy);
  moveTo("CameraClear", [this](bool ok, const std::string& message) {
    setBusy(false);
    setStatus(QString::fromStdString(message), ok ? kOk : kError);
  });
}

void ScoopVisionPanel::onCapture()
{
  callTrigger(capture_client_, "Capturing the powder surface...",
    [this](bool ok, const std::string&) {
      if (ok) {
        have_plan_ = false;
        plan_label_->setText("Height map captured. Plan the next scoop.");
        refreshButtons();
      }
    });
}

void ScoopVisionPanel::onPlan()
{
  if (!plan_client_->service_is_ready()) {
    setStatus("/scoop_vision/plan is not available.", kError);
    return;
  }
  setBusy(true);
  setStatus("Planning the next scoop...", kBusy);
  plan_client_->async_send_request(
    std::make_shared<PlanScoop::Request>(),
    [this](rclcpp::Client<PlanScoop>::SharedFuture future) {
      setBusy(false);
      try {
        const auto r = future.get();
        have_plan_ = r->success;
        if (r->success) {
          plan_x_ = r->pattern_offset_x;
          plan_y_ = r->pattern_offset_y;
          plan_z_ = r->pattern_offset_z;
          plan_label_->setText(
            QString("Shift (%1, %2, %3) mm · fill %4% · depth %5 mm · clearance %6 mm · "
                    "map %7 s old")
              .arg(plan_x_ * 1000.0, 0, 'f', 0)
              .arg(plan_y_ * 1000.0, 0, 'f', 0)
              .arg(plan_z_ * 1000.0, 0, 'f', 0)
              .arg(r->predicted_fill_ratio * 100.0, 0, 'f', 0)
              .arg(r->max_penetration_m * 1000.0, 0, 'f', 0)
              .arg(r->min_clearance_m * 1000.0, 0, 'f', 0)
              .arg(r->heightmap_age_s, 0, 'f', 0));
        } else {
          plan_label_->setText(r->container_empty ? "Container looks empty." : "No plan.");
        }
        setStatus(QString::fromStdString(r->message), r->success ? kOk : kError);
      } catch (const std::exception& ex) {
        have_plan_ = false;
        setStatus(QString("Plan call failed: %1").arg(ex.what()), kError);
      }
      refreshButtons();
    });
}

void ScoopVisionPanel::setOffsets(
  double x, double y, double z, std::function<void(bool, const std::string&)> done)
{
  if (!mtc_params_->service_is_ready()) {
    done(false, "/scooping_mtc_node parameters are not available");
    return;
  }
  mtc_params_->set_parameters(
    {rclcpp::Parameter("pattern_offset_x", x), rclcpp::Parameter("pattern_offset_y", y),
     rclcpp::Parameter("pattern_offset_z", z)},
    [done](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        for (const auto& result : future.get()) {
          if (!result.successful) {
            done(false, "set pattern_offset failed: " + result.reason);
            return;
          }
        }
        done(true, "");
      } catch (const std::exception& ex) {
        done(false, std::string("set pattern_offset failed: ") + ex.what());
      }
    });
}

void ScoopVisionPanel::onPreview()
{
  setBusy(true);
  setStatus("Setting the scoop offsets and planning with MTC...", kBusy);
  setOffsets(plan_x_, plan_y_, plan_z_, [this](bool ok, const std::string& message) {
    if (!ok) {
      setBusy(false);
      setStatus(QString::fromStdString(message), kError);
      return;
    }
    setBusy(false);
    callTrigger(mtc_plan_client_, "MTC planning the shifted scoop (see Motion Planning Tasks)...");
  });
}

void ScoopVisionPanel::onRun()
{
  const auto answer = QMessageBox::question(
    this, "Run planned scoop",
    QString("This MOVES THE ARM:\n\n1. MoveTo MoveToScoopingContainer (20% speed)\n"
            "2. Execute the authored scoop shifted by (%1, %2, %3) mm\n\nContinue?")
      .arg(plan_x_ * 1000.0, 0, 'f', 0)
      .arg(plan_y_ * 1000.0, 0, 'f', 0)
      .arg(plan_z_ * 1000.0, 0, 'f', 0));
  if (answer != QMessageBox::Yes) {
    return;
  }
  setBusy(true);
  setStatus("Setting the scoop offsets...", kBusy);
  setOffsets(plan_x_, plan_y_, plan_z_, [this](bool ok, const std::string& message) {
    if (!ok) {
      setBusy(false);
      setStatus(QString::fromStdString(message), kError);
      return;
    }
    setStatus("Moving to MoveToScoopingContainer...", kBusy);
    moveTo("MoveToScoopingContainer", [this](bool moved, const std::string& move_message) {
      if (!moved) {
        setBusy(false);
        setStatus(QString::fromStdString(move_message), kError);
        return;
      }
      setBusy(false);
      // The height map goes stale as soon as the TCP enters the bin.
      have_plan_ = false;
      plan_label_->setText("Scoop running; capture again for the next one.");
      callTrigger(mtc_execute_client_, "Executing the planned scoop...");
    });
  });
}

void ScoopVisionPanel::onUseAuthored()
{
  setBusy(true);
  setOffsets(0.0, 0.0, 0.0, [this](bool ok, const std::string& message) {
    setBusy(false);
    setStatus(
      ok ? "pattern_offset_x/y/z reset to 0: authored scoop." : QString::fromStdString(message),
      ok ? kOk : kError);
  });
}

}  // namespace scooping_controller

PLUGINLIB_EXPORT_CLASS(scooping_controller::ScoopVisionPanel, rviz_common::Panel)
