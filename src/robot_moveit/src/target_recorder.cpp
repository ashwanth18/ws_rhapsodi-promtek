#include "robot_moveit/target_recorder.hpp"

#include <fstream>

namespace robot_moveit {

TargetRecorder::TargetRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("target_recorder", options)
{
  this->declare_parameter<std::string>("planning_group", "arm");
  this->declare_parameter<std::string>("targets_yaml", "");
  this->declare_parameter<std::string>("eef_link", "");
  this->declare_parameter<std::string>("pose_source", "auto");
  init_timer_ = this->create_wall_timer(std::chrono::milliseconds(0), std::bind(&TargetRecorder::deferred_init, this));
}

void TargetRecorder::deferred_init()
{
  init_timer_->cancel();
  auto group = this->get_parameter("planning_group").as_string();
  mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group);
  end_effector_link_ = this->get_parameter("eef_link").as_string();
  if (!end_effector_link_.empty()) {
    // Force the tip link if provided
    mgi_->setEndEffectorLink(end_effector_link_);
  } else {
    end_effector_link_ = mgi_->getEndEffectorLink();
  }

  // Set up TF buffer/listener to transform the recorded pose into base_link
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  pose_source_ = this->get_parameter("pose_source").as_string();
  if (pose_source_ != "tf" && pose_source_ != "moveit" && pose_source_ != "auto") {
    RCLCPP_WARN(this->get_logger(), "Invalid pose_source '%s'. Using 'auto' (tf then moveit).", pose_source_.c_str());
    pose_source_ = "auto";
  }

  srv_ = this->create_service<robot_common_msgs::srv::RecordTarget>(
    "record_target",
    std::bind(&TargetRecorder::handle_request, this, std::placeholders::_1, std::placeholders::_2));
}

void TargetRecorder::handle_request(const std::shared_ptr<robot_common_msgs::srv::RecordTarget::Request> request,
                                    std::shared_ptr<robot_common_msgs::srv::RecordTarget::Response> response)
{
  const std::string yaml_path = this->get_parameter("targets_yaml").as_string();
  if (yaml_path.empty()) {
    response->success = false;
    response->message = "Parameter targets_yaml is empty";
    return;
  }

  YAML::Node root;
  try {
    if (std::ifstream(yaml_path).good()) {
      root = YAML::LoadFile(yaml_path);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to load YAML, will create new: %s", e.what());
  }

  if (!root["targets"]) {
    root["targets"] = YAML::Node(YAML::NodeType::Map);
  }

  // Append/overwrite into the existing YAML under the 'targets' map.
  // If 'joints' is true, store a joint map; otherwise, store a Cartesian pose
  // using the same schema consumed by MoveToServer (frame_id/position/orientation).
  if (request->joints) {
    const std::vector<std::string> names = mgi_->getJointNames();
    const std::vector<double> values = mgi_->getCurrentJointValues();
    YAML::Node entry;
    write_joints(entry, names, values);
    root["targets"][request->name] = entry;
  } else {
    // Record Cartesian pose using requested source
    const std::string target_frame = "base_link";
    bool wrote = false;
    if (pose_source_ == "tf" || pose_source_ == "auto") {
      try {
        const auto tf = tf_buffer_->lookupTransform(target_frame, end_effector_link_, tf2::TimePointZero, tf2::durationFromSec(0.2));
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = target_frame;
        ps.pose.position.x = tf.transform.translation.x;
        ps.pose.position.y = tf.transform.translation.y;
        ps.pose.position.z = tf.transform.translation.z;
        ps.pose.orientation = tf.transform.rotation;
        YAML::Node entry;
        write_pose(entry, ps);
        root["targets"][request->name] = entry;
        wrote = true;
      } catch (const std::exception & e) {
        if (pose_source_ == "tf") {
          RCLCPP_ERROR(this->get_logger(), "TF lookup failed and pose_source=tf. Error: %s", e.what());
          response->success = false;
          response->message = std::string("TF lookup failed: ") + e.what();
          return;
        }
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Falling back to MoveIt pose.", e.what());
      }
    }

    if (!wrote && (pose_source_ == "moveit" || pose_source_ == "auto")) {
      // Fallback to MoveIt current pose, then try to transform to base_link
      geometry_msgs::msg::PoseStamped ps_raw = mgi_->getCurrentPose(end_effector_link_);
      geometry_msgs::msg::PoseStamped ps;
      try {
        ps = tf_buffer_->transform(ps_raw, target_frame, tf2::durationFromSec(0.2));
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Transform of MoveIt pose failed: %s. Storing raw pose.", e.what());
        ps = ps_raw;
      }
      YAML::Node entry;
      write_pose(entry, ps);
      root["targets"][request->name] = entry;
    }
  }

  std::string err;
  // Persist the updated YAML file to disk.
  if (!save_yaml(yaml_path, root, err)) {
    response->success = false;
    response->message = err;
    return;
  }

  response->success = true;
  response->message = "Recorded '" + request->name + "'";
}

void TargetRecorder::write_pose(YAML::Node & node, const geometry_msgs::msg::PoseStamped & ps)
{
  node["frame_id"] = ps.header.frame_id;
  YAML::Node pos;
  pos["x"] = ps.pose.position.x;
  pos["y"] = ps.pose.position.y;
  pos["z"] = ps.pose.position.z;
  node["position"] = pos;
  YAML::Node ori;
  ori["x"] = ps.pose.orientation.x;
  ori["y"] = ps.pose.orientation.y;
  ori["z"] = ps.pose.orientation.z;
  ori["w"] = ps.pose.orientation.w;
  node["orientation"] = ori;
}

void TargetRecorder::write_joints(YAML::Node & node,
                                  const std::vector<std::string> & names,
                                  const std::vector<double> & values)
{
  YAML::Node joints(YAML::NodeType::Map);
  const std::size_t n = std::min(names.size(), values.size());
  for (std::size_t i = 0; i < n; ++i) {
    joints[names[i]] = values[i];
  }
  node["joints"] = joints;
}

bool TargetRecorder::save_yaml(const std::string & path, const YAML::Node & root, std::string & err)
{
  try {
    std::ofstream out(path);
    out << root;
    out.close();
    return true;
  } catch (const std::exception & e) {
    err = std::string("Failed to write YAML: ") + e.what();
    return false;
  }
}

} // namespace robot_moveit


