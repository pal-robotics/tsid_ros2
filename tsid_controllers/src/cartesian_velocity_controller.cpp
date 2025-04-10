// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tsid_controllers/cartesian_velocity_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pluginlib/class_list_macros.hpp>

using namespace controller_interface;
// using hardware_interface::HW_IF_EFFORT;
// using hardware_interface::HW_IF_POSITION;
// using hardware_interface::HW_IF_VELOCITY;

namespace tsid_controllers
{
using std::placeholders::_1;

CartesianVelocityController::CartesianVelocityController()
: tsid_controllers::TsidVelocityControl() {}

controller_interface::CallbackReturn CartesianVelocityController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  auto result = TsidVelocityControl::on_configure(prev_state);

  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  if (getParams().ee_names.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "The end effector name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Storing names of desired end effector
  ee_names_.resize(getParams().ee_names.size());
  H_ee_0_.resize(getParams().ee_names.size());

  for (size_t i = 0; i < getParams().ee_names.size(); i++) {
    ee_id_.insert(std::make_pair(getParams().ee_names[i], i));
    ee_names_[i] = getParams().ee_names[i];
  }


  std::string controller_name = get_node()->get_name();

  //  creating publisher for current pose ee
  publisher_curr_pos_ = get_node()->create_publisher<geometry_msgs::msg::Pose>(
    controller_name + "/current_position", 10);


  // Pose reference callback
  ee_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    controller_name + "/vel_cmd", 1,
    std::bind(&CartesianVelocityController::setVelCallback, this, _1));

  // Initializing tasks
  double transition_time = 0.0;

  for (auto ee : ee_names_) {
    task_ee_.push_back(
      new tsid::tasks::TaskCartesianVelocity(
        "task-ee" + ee, *robot_wrapper_, ee, dt_.seconds()));

    auto gain = getParams().cartesian_vel_gain.ee_names_map.at(ee);

    Eigen::VectorXd kp_gain = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd kd_gain = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ki_gain = Eigen::VectorXd::Zero(6);
    kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch,
      gain.kp_yaw;
    kd_gain << gain.kd_x, gain.kd_y, gain.kd_z, gain.kd_roll, gain.kd_pitch,
      gain.kd_yaw;
    ki_gain << gain.ki_x, gain.ki_y, gain.ki_z, gain.ki_roll, gain.ki_pitch,
      gain.ki_yaw;
    task_ee_[ee_id_[ee]]->Kp(kp_gain);
    task_ee_[ee_id_[ee]]->Kd(kd_gain);
    task_ee_[ee_id_[ee]]->Ki(ki_gain);

    Eigen::VectorXd ee_mask = Eigen::VectorXd::Zero(6);
    ee_mask << 1, 1, 1, 1, 1, 1;
    task_ee_[ee_id_[ee]]->setMask(ee_mask);
    task_ee_[ee_id_[ee]]->useLocalFrame(false);

    double ee_weight = 1;
    int ee_priority = 1;
    formulation_->addMotionTask(
      *task_ee_[ee_id_[ee]], ee_weight, ee_priority,
      transition_time);
  }


  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn CartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto result = TsidVelocityControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  // Setting initial reference for the end effector tasks
  for (auto ee : ee_names_) {
    Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(6);

    traj_ee_.push_back(
      tsid::trajectories::TrajectoryEuclidianConstant("traj_ee", vel_des));
    tsid::trajectories::TrajectorySample sample_vel_ee =
      traj_ee_[ee_id_[ee]].computeNext();
    task_ee_[ee_id_[ee]]->setReference(sample_vel_ee);
  }


  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type
CartesianVelocityController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{

  // Updating params if new ones are available
  updateParams();

  // Taking current state
  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;

  compute_problem_and_set_command(state.first, state.second);


  if (joint_limit_reached_) {
    for (auto ee : ee_names_) {

      Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(6);
      vel_des << 0, 0, 0, 0, 0, 0;

      tsid::trajectories::TrajectorySample sample_vel_ee =
        traj_ee_[ee_id_[ee]].computeNext();
      sample_vel_ee.setValue(vel_des);
      task_ee_[ee_id_[ee]]->setReference(sample_vel_ee);
    }
  }

  auto h_ee_ = TsidVelocityControl::robot_wrapper_->framePosition(
    TsidVelocityControl::formulation_->data(),
    TsidVelocityControl::model_.getFrameId(ee_names_[0]));

  Eigen::VectorXd m_p;
  m_p = Eigen::VectorXd::Zero(7);
  tsid::math::SE3ToXYZQUAT(h_ee_, m_p);

  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = h_ee_.translation()[0];
  current_pose.position.y = h_ee_.translation()[1];
  current_pose.position.z = h_ee_.translation()[2];
  current_pose.orientation.x = m_p[3];
  current_pose.orientation.y = m_p[4];
  current_pose.orientation.z = m_p[5];
  current_pose.orientation.w = m_p[6];

  publisher_curr_pos_->publish(current_pose);

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::setVelCallback(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != 6) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "The vel command should have 6 elements");
    return;
  }
  auto ee = params_.ee_names[0];
  Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(6);
  vel_des << msg->data[0], msg->data[1], msg->data[2], msg->data[3],
    msg->data[4], msg->data[5];

  tsid::trajectories::TrajectorySample sample_vel_ee =
    traj_ee_[ee_id_[ee]].computeNext();
  sample_vel_ee.setValue(vel_des);
  task_ee_[ee_id_[ee]]->setReference(sample_vel_ee);
  std::cout << "setVelCallback" << std::endl;
  RCLCPP_INFO(
    get_node()->get_logger(), "Desired velocity: %f %f %f %f %f %f",
    vel_des[0], vel_des[1], vel_des[2], vel_des[3], vel_des[4],
    vel_des[5]);
}

void CartesianVelocityController::updateParams()
{
  TsidVelocityControl::updateParams();


  for (auto ee : ee_names_) {
    auto gain = getParams().cartesian_vel_gain.ee_names_map.at(ee);

    Eigen::VectorXd kp_gain = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd kd_gain = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ki_gain = Eigen::VectorXd::Zero(6);
    kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch,
      gain.kp_yaw;
    kd_gain << gain.kd_x, gain.kd_y, gain.kd_z, gain.kd_roll, gain.kd_pitch,
      gain.kd_yaw;
    ki_gain << gain.ki_x, gain.ki_y, gain.ki_z, gain.ki_roll, gain.ki_pitch,
      gain.ki_yaw;
    task_ee_[ee_id_[ee]]->Kp(kp_gain);
    task_ee_[ee_id_[ee]]->Kd(kd_gain);
    task_ee_[ee_id_[ee]]->Ki(ki_gain);

  }

}


} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::CartesianVelocityController,
  controller_interface::ControllerInterface)
