#include <chrono>
#include <memory>
#include <string>

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tsid_controller_msgs/msg/ee_pos.hpp"
#include "geometry_msgs/msg/pose.hpp"


using std::placeholders::_1;

namespace tsid_interactive_markers
{

class InteractiveMarkerTsidNode : public rclcpp::Node
{
public:
  explicit InteractiveMarkerTsidNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~InteractiveMarkerTsidNode() = default;

  inline void
  applyChanges()
  {
    server_->applyChanges();
  }

  visualization_msgs::msg::Marker
  makeBox(const visualization_msgs::msg::InteractiveMarker & msg);

  visualization_msgs::msg::InteractiveMarkerControl &
  makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg);

  void
  make6DofMarker(
    bool fixed, std::string frame_name, unsigned int interaction_mode,
    const tf2::Vector3 & position, bool show_6dof);

  void
  makeButtonMarker(const tf2::Vector3 & position);

private:
  void
  frameCallback();

  void
  processFeedbackLeft(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void
  processFeedbackRight(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void
  processFeedbackHead(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void
  processButtonFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void
  alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void on_timer();
  void publish_cmd_();

  rclcpp::Publisher<tsid_controller_msgs::msg::EePos>::SharedPtr publisher_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::vector<double> desired_position_left_;
  std::vector<double> desired_position_right_;
  std::vector<double> desired_position_head_;

  std::vector<double> current_position_left_;
  std::vector<double> current_position_right_;
  std::vector<double> current_position_head_;

  std::vector<std::string> ee_to_update_;
  std::vector<geometry_msgs::msg::Pose> desired_pose_;

  bool first_pos = false;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;

};  // class InteractiveMarkerTsidNode

InteractiveMarkerTsidNode::InteractiveMarkerTsidNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("interactive_marker_tsid", options),
  menu_handler_()
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "interactive_marker_tsid",
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_topics_interface(),
    get_node_services_interface());

  publisher_ = this->create_publisher<tsid_controller_msgs::msg::EePos>(
    "dynamic_tsid_controller/pose_cmd",
    10);

  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("Origin", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  desired_position_left_.resize(3);
  desired_position_right_.resize(3);
  desired_position_head_.resize(3);

  current_position_left_.resize(3);
  current_position_right_.resize(3);
  current_position_head_.resize(3);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&InteractiveMarkerTsidNode::on_timer, this));
  // create a timer to update the published transforms
  frame_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&InteractiveMarkerTsidNode::frameCallback, this));

  timer_cmd_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&InteractiveMarkerTsidNode::publish_cmd_, this));
}

visualization_msgs::msg::Marker
InteractiveMarkerTsidNode::makeBox(const visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

void InteractiveMarkerTsidNode::on_timer()
{
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped = tf_buffer_->lookupTransform(
    "base_link", "arm_left_7_link",
    tf2::TimePointZero, tf2::durationFromSec(5.0));

  current_position_left_[0] = transformStamped.transform.translation.x;
  current_position_left_[1] = transformStamped.transform.translation.y;
  current_position_left_[2] = transformStamped.transform.translation.z;

  transformStamped = tf_buffer_->lookupTransform(
    "base_link", "arm_right_7_link",
    tf2::TimePointZero, tf2::durationFromSec(5.0));

  current_position_right_[0] = transformStamped.transform.translation.x;
  current_position_right_[1] = transformStamped.transform.translation.y;
  current_position_right_[2] = transformStamped.transform.translation.z;

  transformStamped = tf_buffer_->lookupTransform(
    "base_link", "arm_head_7_link",
    tf2::TimePointZero, tf2::durationFromSec(5.0));

  current_position_head_[0] = transformStamped.transform.translation.x;
  current_position_head_[1] = transformStamped.transform.translation.y;
  current_position_head_[2] = transformStamped.transform.translation.z;

  first_pos = true;
}


visualization_msgs::msg::InteractiveMarkerControl &
InteractiveMarkerTsidNode::makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void
InteractiveMarkerTsidNode::frameCallback()
{
  static uint32_t counter = 0;

  if (!tf_broadcaster_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

  tf2::TimePoint tf_time_point(std::chrono::nanoseconds(this->get_clock()->now().nanoseconds()));

  tf2::Stamped<tf2::Transform> transform;
  transform.stamp_ = tf_time_point;
  transform.frame_id_ = "gripper_thumb_finger_tip_link";
  transform.setOrigin(tf2::Vector3(0.0, 0.0, sin(static_cast<double>(counter) / 140.0) * 2.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "moving_frame";
  tf_broadcaster_->sendTransform(transform_msg);

  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setRPY(0.0, static_cast<double>(counter) / 140.0, 0.0);
  transform.setRotation(quat);
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "rotating_frame";
  tf_broadcaster_->sendTransform(transform_msg);

  counter++;
}
void
InteractiveMarkerTsidNode::processButtonFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    auto message = tsid_controller_msgs::msg::EePos();

    message.ee_name.resize(ee_to_update_.size());
    message.desired_pose.resize(ee_to_update_.size());

    for (int i = 0; i < ee_to_update_.size(); i++) {
      message.ee_name[i] = ee_to_update_[i];
      message.desired_pose[i].position.x = desired_pose_[i].position.x;
      message.desired_pose[i].position.y = desired_pose_[i].position.y;
      message.desired_pose[i].position.z = desired_pose_[i].position.z;
      message.desired_pose[i].orientation.x = 0.0;
      message.desired_pose[i].orientation.y = 0.0;
      message.desired_pose[i].orientation.z = 0.0;
      message.desired_pose[i].orientation.w = 1.0;
    }


    publisher_->publish(message);
  }

  server_->applyChanges();
}

void
InteractiveMarkerTsidNode::processFeedbackLeft(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {

    if (find(
        ee_to_update_.begin(), ee_to_update_.end(),
        "arm_left_7_link") == ee_to_update_.end())
    {
      ee_to_update_.push_back("arm_left_7_link");
      desired_pose_.push_back(feedback->pose);
    } else {
      for (int i = 0; i < ee_to_update_.size(); i++) {
        if (ee_to_update_[i] == "arm_left_7_link") {
          desired_pose_[i] = feedback->pose;
        }
      }
    }
  }

  server_->applyChanges();
}


void
InteractiveMarkerTsidNode::processFeedbackRight(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {

    if (find(
        ee_to_update_.begin(), ee_to_update_.end(),
        "arm_right_7_link") == ee_to_update_.end())
    {
      ee_to_update_.push_back("arm_right_7_link");
      desired_pose_.push_back(feedback->pose);
    } else {
      for (int i = 0; i < ee_to_update_.size(); i++) {
        if (ee_to_update_[i] == "arm_right_7_link") {
          desired_pose_[i] = feedback->pose;
        }
      }
    }
  }

  server_->applyChanges();
}


void
InteractiveMarkerTsidNode::processFeedbackHead(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {

    if (find(
        ee_to_update_.begin(), ee_to_update_.end(),
        "arm_head_7_link") == ee_to_update_.end())
    {
      ee_to_update_.push_back("arm_head_7_link");
      desired_pose_.push_back(feedback->pose);
    } else {
      for (int i = 0; i < ee_to_update_.size(); i++) {
        if (ee_to_update_[i] == "arm_head_7_link") {
          desired_pose_[i] = feedback->pose;
        }
      }
    }

  }

  server_->applyChanges();
}

void
InteractiveMarkerTsidNode::alignMarker(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  geometry_msgs::msg::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x - 0.5) + 0.5;
  pose.position.y = round(pose.position.y - 0.5) + 0.5;

  std::ostringstream oss;
  oss << feedback->marker_name << ":" <<
    " aligning position = " <<
    feedback->pose.position.x <<
    ", " << feedback->pose.position.y <<
    ", " << feedback->pose.position.z <<
    " to " <<
    pose.position.x <<
    ", " << pose.position.y <<
    ", " << pose.position.z;
  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

  server_->setPose(feedback->marker_name, pose);
  server_->applyChanges();
}

void
InteractiveMarkerTsidNode::make6DofMarker(
  bool fixed, std::string frame_name, unsigned int interaction_mode,
  const tf2::Vector3 & position,
  bool show_6dof)
{
  while (!first_pos) {
    rclcpp::spin_some(this->get_node_base_interface());
  }

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";

  if (frame_name == "arm_left_7_link") {
    int_marker.pose.position.x = current_position_left_[0];
    int_marker.pose.position.y = current_position_left_[1];
    int_marker.pose.position.z = current_position_left_[2];
  } else if (frame_name == "arm_right_7_link") {
    int_marker.pose.position.x = current_position_right_[0];
    int_marker.pose.position.y = current_position_right_[1];
    int_marker.pose.position.z = current_position_right_[2];
  } else if (frame_name == "arm_head_7_link") {
    int_marker.pose.position.x = current_position_head_[0];
    int_marker.pose.position.y = current_position_head_[1];
    int_marker.pose.position.z = current_position_head_[2];
  }

  int_marker.scale = 0.1;

  int_marker.name = frame_name;
  int_marker.description = frame_name;

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::msg::InteractiveMarkerControl control;


  if (show_6dof) {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server_->insert(int_marker);
  if (frame_name == "arm_left_7_link") {
    server_->setCallback(
      int_marker.name,
      std::bind(&InteractiveMarkerTsidNode::processFeedbackLeft, this, _1));
  } else if (frame_name == "arm_right_7_link") {
    server_->setCallback(
      int_marker.name,
      std::bind(&InteractiveMarkerTsidNode::processFeedbackRight, this, _1));
  } else if (frame_name == "arm_head_7_link") {
    server_->setCallback(
      int_marker.name,
      std::bind(&InteractiveMarkerTsidNode::processFeedbackHead, this, _1));
  }

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE) {
    menu_handler_.apply(*server_, int_marker.name);
  }
}

void
InteractiveMarkerTsidNode::makeButtonMarker(const tf2::Vector3 & position)
{

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = 0.0;
  int_marker.pose.position.y = -1.0;
  int_marker.pose.position.z = 0.0;
  int_marker.scale = 0.1;

  int_marker.name = "button";
  int_marker.description = "Button_Send_Command";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";


  visualization_msgs::msg::Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(
    int_marker.name,
    std::bind(&InteractiveMarkerTsidNode::processButtonFeedback, this, _1));
}

void InteractiveMarkerTsidNode::publish_cmd_()
{
  auto message = tsid_controller_msgs::msg::EePos();

  message.ee_name.resize(ee_to_update_.size());
  message.desired_pose.resize(ee_to_update_.size());

  for (int i = 0; i < ee_to_update_.size(); i++) {
    message.ee_name[i] = ee_to_update_[i];
    message.desired_pose[i].position.x = desired_pose_[i].position.x;
    message.desired_pose[i].position.y = desired_pose_[i].position.y;
    message.desired_pose[i].position.z = desired_pose_[i].position.z;
    message.desired_pose[i].orientation.x = 0.0;
    message.desired_pose[i].orientation.y = 0.0;
    message.desired_pose[i].orientation.z = 0.0;
    message.desired_pose[i].orientation.w = 1.0;
  }

  publisher_->publish(message);

}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto tsid_markers = std::make_shared<tsid_interactive_markers::InteractiveMarkerTsidNode>();

  tf2::Vector3 position(0, 0, 0);
  tsid_markers->make6DofMarker(
    false, "arm_left_7_link",
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);
  position = tf2::Vector3(0, 0, 0);
  tsid_markers->make6DofMarker(
    true, "arm_right_7_link",
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);
  position = tf2::Vector3(0, 0, 0);
  tsid_markers->make6DofMarker(
    false, "arm_head_7_link",
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);
  position = tf2::Vector3(0, 0, 0);


  tsid_markers->makeButtonMarker(position);

  tsid_markers->applyChanges();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tsid_markers);
  RCLCPP_INFO(tsid_markers->get_logger(), "Ready");
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
