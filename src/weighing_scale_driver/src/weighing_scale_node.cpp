#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <memory>
#include <optional>
#include <string>
#include <cerrno>
#include <cstring>

#include "rhapsodi_common_cpp/health_event_publisher.hpp"

namespace
{

int open_serial(const std::string & port, int baud)
{
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  termios tty;
  if (tcgetattr(fd, &tty) != 0) { ::close(fd); return -1; }
  cfmakeraw(&tty);
  speed_t b = B115200;
  switch (baud) { case 9600: b = B9600; break; case 19200: b = B19200; break; case 38400: b = B38400; break; case 57600: b = B57600; break; case 115200: b = B115200; break; }
  cfsetispeed(&tty, b); cfsetospeed(&tty, b);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
  tty.c_cc[VTIME] = 1; tty.c_cc[VMIN] = 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return -1; }
  return fd;
}

std::optional<double> parse_weight(const std::string & line)
{
  std::string s = line;
  // trim CR/LF and spaces
  while (!s.empty() && (s.back()=='\r' || s.back()=='\n' || s.back()==' ')) s.pop_back();
  size_t i = 0; std::string num; bool seen_digit=false, seen_dot=false;
  if (i < s.size() && (s[i]=='+'||s[i]=='-')) { num.push_back(s[i++]); }
  for (; i < s.size(); ++i) {
    char c = s[i];
    if (c>='0'&&c<='9') { num.push_back(c); seen_digit=true; }
    else if (c=='.' && !seen_dot) { num.push_back(c); seen_dot=true; }
    else if (seen_digit) break;
  }
  if (!seen_digit) return std::nullopt;
  try { return std::stod(num); } catch (...) { return std::nullopt; }
}

} // namespace

class WeighingScaleNode : public rclcpp::Node
{
public:
  WeighingScaleNode() : rclcpp::Node("weighing_scale_node")
  {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 9600);
    this->declare_parameter<std::string>("topic", "/weight");
    this->declare_parameter<double>("stale_after_seconds", 5.0);
    topic_ = this->get_parameter("topic").as_string();
    stale_after_s_ = this->get_parameter("stale_after_seconds").as_double();
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(topic_, 10);
    health_ = std::make_unique<rhapsodi_common_cpp::HealthEventPublisher>(
      this, "weighing_scale_driver");
    int baud = this->get_parameter("baud").as_int();
    std::string port = this->get_parameter("port").as_string();
    fd_ = open_serial(port, baud);
    if (fd_ < 0) {
      const std::string err = std::strerror(errno);
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s (baud %d): %s",
                   port.c_str(), baud, err.c_str());
      health_->critical(
        "serial_open_failed",
        "Failed to open weighing scale serial port " + port,
        "{\"port\":\"" + port + "\",\"baud\":" + std::to_string(baud) +
          ",\"errno\":\"" + err + "\"}");
      throw std::runtime_error("open serial failed");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to weighing scale on %s at %d baud",
                port.c_str(), baud);
    RCLCPP_INFO(this->get_logger(), "Publishing scale readings on %s", topic_.c_str());
    last_reading_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&WeighingScaleNode::tick, this));
    stale_check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&WeighingScaleNode::checkStale, this));
  }

  ~WeighingScaleNode() override
  {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void tick()
  {
    char buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) return;
    buffer_.append(buf, static_cast<size_t>(n));
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
      std::string line = buffer_.substr(0, pos);
      buffer_.erase(0, pos + 1);
      auto w = parse_weight(line);
      if (w.has_value()) {
        std_msgs::msg::Float64 msg; msg.data = *w; publisher_->publish(msg);
        last_reading_time_ = this->now();
        if (is_stale_) {
          is_stale_ = false;
          health_->info("scale_readings_resumed", "Weighing scale readings resumed");
        }
      }
    }
  }

  void checkStale()
  {
    const double age_s = (this->now() - last_reading_time_).seconds();
    if (age_s > stale_after_s_ && !is_stale_) {
      is_stale_ = true;
      RCLCPP_WARN(this->get_logger(), "No weight reading for %.1fs", age_s);
      health_->warn(
        "scale_readings_stale",
        "No weighing scale reading for " + std::to_string(age_s) + "s",
        "{\"age_seconds\":" + std::to_string(age_s) + "}");
    }
  }

  int fd_ { -1 };
  std::string buffer_;
  std::string topic_;
  double stale_after_s_ { 5.0 };
  bool is_stale_ { false };
  rclcpp::Time last_reading_time_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stale_check_timer_;
  std::unique_ptr<rhapsodi_common_cpp::HealthEventPublisher> health_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeighingScaleNode>());
  rclcpp::shutdown();
  return 0;
}
