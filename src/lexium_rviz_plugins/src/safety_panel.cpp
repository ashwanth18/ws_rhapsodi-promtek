#include "lexium_rviz_plugins/safety_panel.hpp"
#include "lexium_rviz_plugins/error_code_utils.hpp"

#include <sstream>

#include <QFont>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace lexium_rviz_plugins
{

SafetyPanel::SafetyPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout();
  root->setContentsMargins(8, 8, 8, 8);

  auto * title = new QLabel("Lexium Safety");
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 1);
  title->setFont(title_font);
  root->addWidget(title);

  status_label_ = new QLabel("Waiting for /lexium/status...");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  hint_label_ = new QLabel("");
  hint_label_->setWordWrap(true);
  hint_label_->setStyleSheet("color: #666;");
  root->addWidget(hint_label_);

  message_label_ = new QLabel("");
  message_label_->setWordWrap(true);
  root->addWidget(message_label_);

  bring_up_btn_ = new QPushButton("Bring Up");
  bring_up_btn_->setStyleSheet(
    "QPushButton { background-color: #27ae60; color: white; font-weight: bold; "
    "padding: 10px; }"
    "QPushButton:disabled { background-color: #888; color: #ddd; }");
  root->addWidget(bring_up_btn_);

  shut_down_btn_ = new QPushButton("Shut Down");
  shut_down_btn_->setStyleSheet(
    "QPushButton { background-color: #555; color: white; font-weight: bold; "
    "padding: 10px; }"
    "QPushButton:disabled { background-color: #888; color: #ddd; }");
  root->addWidget(shut_down_btn_);

  clear_error_btn_ = new QPushButton("Clear Error");
  root->addWidget(clear_error_btn_);

  stop_btn_ = new QPushButton("STOP");
  stop_btn_->setStyleSheet(
    "QPushButton { background-color: #c0392b; color: white; font-weight: bold; "
    "padding: 8px; }"
    "QPushButton:disabled { background-color: #888; color: #ddd; }");
  root->addWidget(stop_btn_);

  root->addStretch();
  setLayout(root);

  connect(bring_up_btn_, &QPushButton::clicked, this, &SafetyPanel::onBringUp);
  connect(shut_down_btn_, &QPushButton::clicked, this, &SafetyPanel::onShutDown);
  connect(clear_error_btn_, &QPushButton::clicked, this, &SafetyPanel::onClearError);
  connect(stop_btn_, &QPushButton::clicked, this, &SafetyPanel::onStop);

  bring_up_btn_->setEnabled(false);
  shut_down_btn_->setEnabled(false);
  clear_error_btn_->setEnabled(false);
  stop_btn_->setEnabled(false);
}

void SafetyPanel::onInitialize()
{
  auto node_iface = getDisplayContext()->getRosNodeAbstraction().lock();
  node_ = node_iface->get_raw_node();

  node_->declare_parameter("driver_ns", driver_ns_);
  driver_ns_ = node_->get_parameter("driver_ns").as_string();

  const auto svc = [this](const std::string & name) {
      return node_->create_client<std_srvs::srv::Trigger>(driver_ns_ + "/" + name);
    };

  bring_up_client_ = svc("bring_up");
  shut_down_client_ = svc("shut_down");
  clear_error_client_ = svc("clear_error");
  stop_client_ = svc("stop");

  status_sub_ = node_->create_subscription<lexium_msgs::msg::LexiumStatus>(
    "/lexium/status", rclcpp::QoS(10),
    std::bind(&SafetyPanel::statusCallback, this, std::placeholders::_1));
}

void SafetyPanel::statusCallback(const lexium_msgs::msg::LexiumStatus::SharedPtr msg)
{
  last_status_ = *msg;
  QTimer::singleShot(0, this, [this]() { updateUi(last_status_); });
}

std::string SafetyPanel::controlSourceLabel(int32_t source) const
{
  switch (source) {
    case 1: return "Stick";
    case 2: return "App";
    case 3: return "Remote";
    default: return "Unknown(" + std::to_string(source) + ")";
  }
}

bool SafetyPanel::hasFaults(const lexium_msgs::msg::LexiumStatus & status) const
{
  return blocksBringUp(status) ||
         (!status.error_msg.empty() && !isNoErrorCode(status.error_code));
}

bool SafetyPanel::blocksBringUp(const lexium_msgs::msg::LexiumStatus & status) const
{
  if (status.protective_stop || status.emergency_stop ||
    status.collision_stop || status.on_soft_limit)
  {
    return true;
  }
  return !isNoErrorCode(status.error_code);
}

std::string SafetyPanel::statusHint(const lexium_msgs::msg::LexiumStatus & status) const
{
  if (busy_) {
    return "Sequence in progress…";
  }
  if (!status.connected) {
    return "Not connected to controller.";
  }
  if (status.enabled && status.powered_on) {
    if (status.control_source != 3) {
      return "Ready for motion once control is Remote (3).";
    }
    return "Arm is up. Use Shut Down when finished.";
  }
  if (blocksBringUp(status)) {
    return "Clear faults before Bring Up.";
  }
  if (!status.powered_on || !status.enabled) {
    return "Bring Up runs power on → settle → enable automatically.";
  }
  return "";
}

void SafetyPanel::updateUi(const lexium_msgs::msg::LexiumStatus & status)
{
  std::ostringstream ss;
  ss << "Connected: " << (status.connected ? "yes" : "no")
     << "  |  Feedback: " << (status.feedback_fresh ? "fresh" : "stale")
     << "\nControl: " << controlSourceLabel(status.control_source)
     << "  |  Powered: " << (status.powered_on ? "on" : "off")
     << "  |  Enabled: " << (status.enabled ? "yes" : "no");

  if (hasFaults(status)) {
    ss << "\nFAULT";
    if (status.protective_stop) { ss << " protective_stop"; }
    if (status.emergency_stop) { ss << " e-stop"; }
    if (status.collision_stop) { ss << " collision"; }
    if (status.on_soft_limit) { ss << " soft_limit"; }
    if (!isNoErrorCode(status.error_code)) {
      ss << " [" << status.error_code << "]";
    }
    if (!status.error_msg.empty()) {
      ss << " " << status.error_msg;
    }
  }

  status_label_->setText(QString::fromStdString(ss.str()));

  const bool faults = hasFaults(status);
  const bool ready = status.enabled && status.powered_on;

  bring_up_btn_->setEnabled(
    !busy_ && status.connected && !ready && !blocksBringUp(status));
  shut_down_btn_->setEnabled(
    !busy_ && status.connected && (status.enabled || status.powered_on));
  clear_error_btn_->setEnabled(!busy_ && status.connected && faults);
  stop_btn_->setEnabled(status.connected);

  hint_label_->setText(QString::fromStdString(statusHint(status)));
}

void SafetyPanel::callTrigger(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & label)
{
  if (!client->service_is_ready()) {
    message_label_->setText(
      QString("Service %1 is not ready.").arg(QString::fromStdString(label)));
    return;
  }

  busy_ = true;
  updateUi(last_status_);
  message_label_->setText(
    QString("Running %1… (may take up to a minute)").arg(QString::fromStdString(label)));

  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, label](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString text = response->success ?
        QString("%1: %2")
          .arg(QString::fromStdString(label))
          .arg(QString::fromStdString(response->message)) :
        QString("%1 failed: %2")
          .arg(QString::fromStdString(label))
          .arg(QString::fromStdString(response->message));
      QTimer::singleShot(0, this, [this, text]() {
          busy_ = false;
          message_label_->setText(text);
          updateUi(last_status_);
        });
    });
}

void SafetyPanel::onBringUp()
{
  callTrigger(bring_up_client_, "bring_up");
}

void SafetyPanel::onShutDown()
{
  callTrigger(shut_down_client_, "shut_down");
}

void SafetyPanel::onClearError()
{
  callTrigger(clear_error_client_, "clear_error");
}

void SafetyPanel::onStop()
{
  callTrigger(stop_client_, "stop");
}

}  // namespace lexium_rviz_plugins

PLUGINLIB_EXPORT_CLASS(lexium_rviz_plugins::SafetyPanel, rviz_common::Panel)
