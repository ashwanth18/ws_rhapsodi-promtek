#include "qr_scanner/scan_qr_server.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <zbar.h>

using namespace std::chrono_literals;

namespace qr_scanner {

ScanQrServer::ScanQrServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_qr_server", options)
{
  this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
  this->declare_parameter<bool>("publish_debug_image", true);
  this->declare_parameter<bool>("always_scan", false);
  // Preprocessing parameters
  this->declare_parameter<bool>("enable_preproc", false);
  this->declare_parameter<std::string>("preproc_mode", "clahe_adapt");
  this->declare_parameter<double>("clahe_clip", 2.0);
  this->declare_parameter<int>("clahe_grid", 8);
  this->declare_parameter<int>("thr_block_size", 21);
  this->declare_parameter<int>("thr_C", 5);
  this->declare_parameter<int>("morph_close", 3);
  this->declare_parameter<bool>("try_opencv", true);
  this->declare_parameter<double>("min_confidence", 0.0);
  publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
  always_scan_ = this->get_parameter("always_scan").as_bool();
  enable_preproc_ = this->get_parameter("enable_preproc").as_bool();
  preproc_mode_ = this->get_parameter("preproc_mode").as_string();
  clahe_clip_ = this->get_parameter("clahe_clip").as_double();
  clahe_grid_ = this->get_parameter("clahe_grid").as_int();
  {
    int v = static_cast<int>(this->get_parameter("thr_block_size").as_int());
    v |= 1; // ensure odd
    thr_block_size_ = std::max(3, v);
  }
  thr_C_ = static_cast<int>(this->get_parameter("thr_C").as_int());
  {
    int v = static_cast<int>(this->get_parameter("morph_close").as_int());
    morph_close_ = std::max(0, v);
  }
  try_opencv_ = this->get_parameter("try_opencv").as_bool();
  min_confidence_ = this->get_parameter("min_confidence").as_double();
  auto topic = this->get_parameter("image_topic").as_string();
  image_sub_ = image_transport::create_subscription(
    this, topic, std::bind(&ScanQrServer::imageCb, this, std::placeholders::_1), "raw");
  // Advertise debug image topic upfront so it's visible even before first detection
  if (publish_debug_image_) {
    debug_pub_ = image_transport::create_publisher(this, "scan_qr/debug_image");
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("scan_qr/camera_info", 10);
  }

  action_server_ = rclcpp_action::create_server<ScanQr>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "scan_qr",
    std::bind(&ScanQrServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ScanQrServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ScanQrServer::handle_accepted, this, std::placeholders::_1));
}
void ScanQrServer::setup_sub()
{
  init_timer_->cancel();
  auto topic = this->get_parameter("image_topic").as_string();
  image_transport::ImageTransport it(shared_from_this());
  image_sub_ = it.subscribe(topic, 1, std::bind(&ScanQrServer::imageCb, this, std::placeholders::_1));
}

ScanQrServer::~ScanQrServer()
{
  cancel_requested_ = true;
  frame_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void ScanQrServer::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    std::lock_guard<std::mutex> lk(frame_mutex_);
    latest_frame_bgr_ = std::make_shared<cv::Mat>(cv_ptr->image.clone());
    last_frame_stamp_ = msg->header.stamp;
    frame_cv_.notify_all();
    if (publish_debug_image_) {
      cv::Mat dbg = cv_ptr->image.clone();
      if (always_scan_) {
        // Run a lightweight detection pass and overlay boxes
        cv::Mat gray; cv::cvtColor(dbg, gray, cv::COLOR_BGR2GRAY);
        if (enable_preproc_) {
          if (preproc_mode_ == std::string("clahe_adapt")) {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_clip_, cv::Size(clahe_grid_, clahe_grid_));
            clahe->apply(gray, gray);
            cv::adaptiveThreshold(gray, gray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, thr_block_size_, thr_C_);
            if (morph_close_ > 1) {
              cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_close_, morph_close_));
              cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, k);
            }
          }
        }
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
        int n = scanner.scan(zbar_img);
        if (n > 0) {
          for (auto sym = zbar_img.symbol_begin(); sym != zbar_img.symbol_end(); ++sym) {
            const int pts = sym->get_location_size();
            for (int i = 0; i < pts; ++i) {
              cv::Point p1(sym->get_location_x(i), sym->get_location_y(i));
              cv::Point p2(sym->get_location_x((i+1)%pts), sym->get_location_y((i+1)%pts));
              cv::line(dbg, p1, p2, cv::Scalar(0,255,0), 2);
            }
          }
        }
      }
      if (!debug_pub_) {
        debug_pub_ = image_transport::create_publisher(this, "scan_qr/debug_image");
      }
      auto out = cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg();
      debug_pub_.publish(out);
      if (camera_info_pub_) {
        sensor_msgs::msg::CameraInfo ci;
        ci.header = msg->header;
        ci.width = out->width;
        ci.height = out->height;
        camera_info_pub_->publish(ci);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "cv_bridge error: %s", e.what());
  }
}

rclcpp_action::GoalResponse ScanQrServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ScanQr::Goal>)
{
  if (active_) {
    RCLCPP_WARN(this->get_logger(), "Scan goal rejected: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ScanQrServer::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  cancel_requested_ = true;
  frame_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ScanQrServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  active_goal_ = goal_handle;
  cancel_requested_ = false;
  active_ = true;
  if (worker_.joinable()) {
    // Join any previous worker (shouldn't happen with rejection logic)
    worker_.join();
  }
  worker_ = std::thread{std::bind(&ScanQrServer::execute, this, goal_handle)};
}

void ScanQrServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ScanQr::Feedback>();
  auto result = std::make_shared<ScanQr::Result>();

  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
  while (rclcpp::ok() && this->now() < deadline && !cancel_requested_) {
    cv::Mat frame;
    {
      std::unique_lock<std::mutex> lk(frame_mutex_);
      frame_cv_.wait_for(lk, 100ms, [&]{ return cancel_requested_ || (latest_frame_bgr_ && !latest_frame_bgr_->empty()); });
      if (cancel_requested_) break;
      if (latest_frame_bgr_ && !latest_frame_bgr_->empty()) {
        frame = latest_frame_bgr_->clone();
      }
    }
    if (frame.empty()) { continue; }
    cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    if (enable_preproc_) {
      if (preproc_mode_ == std::string("clahe_adapt")) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_clip_, cv::Size(clahe_grid_, clahe_grid_));
        clahe->apply(gray, gray);
        cv::adaptiveThreshold(gray, gray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, thr_block_size_, thr_C_);
        if (morph_close_ > 1) {
          cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_close_, morph_close_));
          cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, k);
        }
      }
    }
    zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
    int n = scanner.scan(zbar_img);
    if (n < 0) {
      RCLCPP_WARN(this->get_logger(), "zbar scan returned %d", n);
    }
    if (publish_debug_image_) {
      cv::Mat dbg = frame.clone();
      if (n > 0) {
        for (auto sym = zbar_img.symbol_begin(); sym != zbar_img.symbol_end(); ++sym) {
          const int pts = sym->get_location_size();
          for (int i = 0; i < pts; ++i) {
            cv::Point p1(sym->get_location_x(i), sym->get_location_y(i));
            cv::Point p2(sym->get_location_x((i+1)%pts), sym->get_location_y((i+1)%pts));
            cv::line(dbg, p1, p2, cv::Scalar(0,255,0), 2);
          }
        }
      }
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dbg).toImageMsg();
      msg->header.stamp = this->now();
      msg->header.frame_id = "camera";
      if (!debug_pub_) {
        debug_pub_ = image_transport::create_publisher(this, "scan_qr/debug_image");
      }
      debug_pub_.publish(msg);
      if (camera_info_pub_) {
        sensor_msgs::msg::CameraInfo ci;
        ci.header = msg->header;
        ci.width = msg->width;
        ci.height = msg->height;
        camera_info_pub_->publish(ci);
      }
    }
    if (n > 0) {
      for (auto sym = zbar_img.symbol_begin(); sym != zbar_img.symbol_end(); ++sym) {
        std::string data = sym->get_data();
        // confidence from symbol quality (heuristic 0..1)
        int quality = sym->get_quality();
        float conf = std::max(0, std::min(quality, 100)) / 100.0f;
        if (conf < min_confidence_) continue;
        feedback->detected_lot = data;
        feedback->confidence = conf;
        goal_handle->publish_feedback(feedback);
        // Log bounding box
        const int pts = sym->get_location_size();
        std::string bbox_str;
        for (int i = 0; i < pts; ++i) {
          int x = sym->get_location_x(i);
          int y = sym->get_location_y(i);
          bbox_str += "(" + std::to_string(x) + "," + std::to_string(y) + ") ";
        }
        RCLCPP_INFO(this->get_logger(), "QR bbox: %s", bbox_str.c_str());
        result->lot = data;
        result->match = (goal->expected_lot.empty() || data == goal->expected_lot);
        result->message = result->match ? "Matched" : "Mismatched";
        goal_handle->succeed(result);
        active_ = false;
        return;
      }
    }
    // Optional fallback: OpenCV QRCodeDetector if zbar fails
    if (n == 0 && try_opencv_) {
      cv::QRCodeDetector qrd;
      std::string dec = qrd.detectAndDecode(gray);
      if (!dec.empty()) {
        feedback->detected_lot = dec;
        feedback->confidence = 0.5f; // unknown; assign mid confidence
        goal_handle->publish_feedback(feedback);
        result->lot = dec;
        result->match = (goal->expected_lot.empty() || dec == goal->expected_lot);
        result->message = result->match ? "Matched" : "Mismatched";
        goal_handle->succeed(result);
        active_ = false;
        return;
      }
    }
  }

  if (cancel_requested_) {
    result->match = false;
    result->lot = "";
    result->message = "Canceled";
    goal_handle->canceled(result);
  } else {
    result->match = false;
    result->lot = "";
    result->message = "Timeout";
    goal_handle->abort(result);
  }
  active_ = false;
}

} // namespace qr_scanner


