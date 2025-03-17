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

#include "tsid_controllers/joint_space_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

JointSpaceTsidController::JointSpaceTsidController()
: tsid_controllers::TsidPositionControl() {}

controller_interface::CallbackReturn JointSpaceTsidController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  position_start_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  position_end_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  position_curr_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  vel_curr_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());

  // Position command
  joint_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "tsid_controllers/joint_position_cmd", 1,
    std::bind(&JointSpaceTsidController::setPositionCb, this, _1));

  return TsidPositionControl::on_configure(prev_state);
}

controller_interface::CallbackReturn JointSpaceTsidController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto result = TsidPositionControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();

  position_start_ = state.first.tail(params_.joint_command_names.size());

  position_end_ = position_start_;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpaceTsidController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  t_curr_ = t_curr_ + dt_.seconds();

  TsidPositionControl::updateParams();

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;

  interpolate(t_curr_);

  tsid::trajectories::TrajectorySample sample_posture_joint(position_curr_.size());
  sample_posture_joint.setValue(position_curr_);
  sample_posture_joint.setDerivative(vel_curr_);

  task_joint_posture_->setReference(sample_posture_joint);


  compute_problem_and_set_command(state.first, state.second); //q and v
  return controller_interface::return_type::OK;
}

void JointSpaceTsidController::setPositionCb(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != params_.joint_command_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received joint position command with incorrect size");
    return;
  }

  auto upper_limits = model_.upperPositionLimit.tail(model_.nv - 6);
  auto lower_limits = model_.lowerPositionLimit.tail(model_.nv - 6);

  for (auto joint : joint_command_names_) {

    if (msg->data[jnt_command_id_[joint]] >
      upper_limits[model_.getJointId(joint) - 2] ||
      msg->data[jnt_command_id_[joint]] <
      lower_limits[model_.getJointId(joint) - 2])
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joint %s command out of boundaries! The motion will not be "
        "performed!",
        joint.c_str());
      return;
    }
  }

  // Setting the reference
  Eigen::VectorXd ref(params_.joint_command_names.size());

  for (size_t i = 0; i < params_.joint_command_names.size(); i++) {
    ref[i] = msg->data[i];
  }

  /*tsid::trajectories::TrajectorySample sample_posture_joint(ref.size());
  sample_posture_joint.setValue(ref);

  task_joint_posture_->setReference(sample_posture_joint);*/


  position_start_ = getActualState().first.tail(params_.joint_command_names.size());
  position_end_ = ref;

  t_curr_ = 0;
}

void JointSpaceTsidController::interpolate(double t_curr)
{
  if (position_end_ == position_start_) {
    position_curr_ = position_end_;
    return;
  }


  int maxDiffIndex = 0;
  double maxDiff = std::abs(position_end_[0] - position_start_[0]);

  for (size_t i = 1; i < position_start_.size(); ++i) {
    double currentDiff = std::abs(position_end_[i] - position_start_[i]);
    if (currentDiff > maxDiff) {
      maxDiff = currentDiff;
      maxDiffIndex = i;
    }
  }

  double a_max;
  double v_max_ = params_.velocity_scaling * v_max;
  RCLCPP_INFO(get_node()->get_logger(), "v_max %f", v_max_);
  a_max = v_max_ / ( 2 * dt_.seconds());
  t_acc_ = v_max_ / a_max;


  double s = 0;
  double s_dot = 0;

  t_flat_ =
    (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex]) - v_max_ * t_acc_) /
    v_max_;

  // Computation of a trapezoidal trajectory
  for (auto joint : params_.joint_command_names) {
    if (t_curr < t_acc_) {
      s = 0.5 * a_max * t_curr * t_curr;
      s_dot = a_max * t_curr;
    } else if (t_curr >= t_acc_ && t_curr < t_acc_ + t_flat_) {
      s = v_max_ *
        (std::abs(position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]])) /
        (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex])) *
        (t_curr - t_acc_ / 2 );
      s_dot = v_max_ * (std::abs(position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]])) /
        (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex]));
    } else if (t_curr >= t_acc_ + t_flat_ && t_curr < t_flat_ + 2 * t_acc_) {
      s = std::abs(position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]]) -
        0.5 * a_max *
        (t_flat_ + 2 * t_acc_ - t_curr) *
        (t_flat_ + 2 * t_acc_ - t_curr);
      s_dot = v_max_ * (std::abs(position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]])) /
        (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex])) - a_max *
        (t_curr - t_flat_ - t_acc_);
    } else {
      s = std::abs(position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]]);
      s_dot = 0;
    }
    position_curr_[jnt_id_[joint]] = position_start_[jnt_id_[joint]] +
      copysign(1, position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]]) * s;
    vel_curr_[jnt_id_[joint]] =
      copysign(1, position_end_[jnt_id_[joint]] - position_start_[jnt_id_[joint]]) * s_dot;
  }


}

} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::JointSpaceTsidController,
  controller_interface::ControllerInterface)
