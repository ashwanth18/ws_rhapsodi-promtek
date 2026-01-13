#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/scan_qr.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

// forward declaration to avoid pulling OpenCV headers into the public header
namespace cv { class Mat; }

namespace qr_scanner {

class ScanQrServer : public rclcpp::Node {
public:
  using ScanQr = robot_common_msgs::action::ScanQr;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ScanQr>;
  explicit ScanQrServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ScanQrServer();

private:
  rclcpp_action::Server<ScanQr>::SharedPtr action_server_;
  std::mutex frame_mutex_;
  std::shared_ptr<cv::Mat> latest_frame_bgr_;
  rclcpp::Time last_frame_stamp_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  bool publish_debug_image_{true};
  bool always_scan_{false};
  // Preprocessing and decoder params
  bool enable_preproc_{false};
  std::string preproc_mode_{"clahe_adapt"};
  double clahe_clip_{2.0};
  int clahe_grid_{8};
  int thr_block_size_{21};
  int thr_C_{5};
  int morph_close_{3};
  bool try_opencv_{true};
  double min_confidence_{0.0};
  rclcpp::TimerBase::SharedPtr init_timer_;
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void setup_sub();
  std::condition_variable frame_cv_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::thread worker_;
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> active_{false};
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ScanQr::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

} // namespace qr_scanner


