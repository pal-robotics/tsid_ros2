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

#include "tsid_controllers/joint_space_vel_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

JointSpaceVelTsidController::JointSpaceVelTsidController()
: tsid_controllers::TsidVelocityControl() {}

controller_interface::CallbackReturn JointSpaceVelTsidController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  // Position command
  joint_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "tsid_controllers/joint_velocity_cmd", 1,
    std::bind(&JointSpaceVelTsidController::setVelocityCb, this, _1));

  // Joint Velocity Task
  task_joint_velocity_ = new tsid::tasks::TaskJointVel(
    "task-joint-velocity", *robot_wrapper_, dt_.seconds());
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);
  Eigen::VectorXd ki = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);

  for (auto joint : joint_command_names_) {
    auto gain = params_.joint_vel_gain.joint_state_names_map.at(joint);

    if (gain.kp < 0 || gain.kd < 0 || gain.ki < 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "The gains must be positive");
      return controller_interface::CallbackReturn::ERROR;
    }

    kp[jnt_command_id_[joint]] = gain.kp;
    kd[jnt_command_id_[joint]] = gain.kd;
    ki[jnt_command_id_[joint]] = gain.ki;
  }

  task_joint_velocity_->Kp(kp);
  task_joint_velocity_->Kd(kd);
  task_joint_velocity_->Ki(ki);
  int velocity_priority = 1; // 0 constraint, 1 cost function
  double transition_time = 0.0;
  double velocity_weight = 1e-3;
  formulation_->addMotionTask(
    *task_joint_velocity_, velocity_weight,
    velocity_priority, transition_time);

  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  traj_joint_velocity_ = new tsid::trajectories::TrajectoryEuclidianConstant(
    "traj_joint", v0.tail(robot_wrapper_->nv() - 6));
  task_joint_velocity_->setReference(traj_joint_velocity_->computeNext());

  return TsidVelocityControl::on_configure(prev_state);
}

controller_interface::CallbackReturn JointSpaceVelTsidController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto result = TsidVelocityControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }


  // Taking initial state from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  for (const auto & joint : params_.joint_state_names) {
    q0.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_[joint]][0].get().get_value();
  }

  formulation_->computeProblemData(0.0, q0, v0);

  traj_joint_velocity_->setReference(v0.tail(robot_wrapper_->nv() - 6));
  task_joint_velocity_->setReference(traj_joint_velocity_->computeNext());


  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type
JointSpaceVelTsidController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  TsidVelocityControl::updateParams();

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;
  compute_problem_and_set_command(state.first, state.second); // q and v

  return controller_interface::return_type::OK;
}

void JointSpaceVelTsidController::setVelocityCb(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != params_.joint_command_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received joint velocity command with incorrect size");
    return;
  }

  // Setting the reference
  Eigen::VectorXd ref(params_.joint_command_names.size());

  for (size_t i = 0; i < params_.joint_command_names.size(); i++) {
    ref[i] = msg->data[i];
  }

  tsid::trajectories::TrajectorySample sample_velocity_joint(ref.size());
  sample_velocity_joint.setValue(ref);

  task_joint_velocity_->setReference(sample_velocity_joint);


}

} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::JointSpaceVelTsidController,
  controller_interface::ControllerInterface)
