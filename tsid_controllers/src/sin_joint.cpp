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

#include "tsid_controllers/sin_joint.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

SinJointSpaceController::SinJointSpaceController()
: tsid_controllers::TsidPositionControl()
{
}

controller_interface::CallbackReturn SinJointSpaceController::on_init()
{
  return TsidPositionControl::on_init();
}

controller_interface::CallbackReturn SinJointSpaceController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  // Position command
  initial_position_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  initial_velocity_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  interface_name_ = getParams().interface_name
  
  sin_amplitude_ = getParams().sin_amplitude; 
  sin_frequency_ = getParams().sin_frequency; 
  sin_phase_ = getParams().sin_phase * M_PI;

  return TsidPositionControl::on_configure(prev_state);
}


controller_interface::CallbackReturn SinJointSpaceController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto result = TsidPositionControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  t_curr_ = 0.0; 

  initial_position_ = state.first.tail(params_.joint_command_names.size());
  initial_velocity_ = state.second.tail(params_.joint_command_names.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SinJointSpaceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  t_curr_ = t_curr_ + dt_.seconds();
  TsidPositionControl::updateParams();

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;

  Eigen::VectorXd ref(params_.joint_command_names.size());
  Eigen::VectorXd vel_ref(params_.joint_command_names.size());

  for (size_t i = 0; i < params_.joint_command_names.size(); ++i) {
      ref[i] =  initial_position_[i] + sin_amplitude_ * sin(2 * M_PI * sin_frequency_ * t_curr_ + sin_phase_);
      vel_ref[i] = sin_amplitude_ * 2 * M_PI * sin_frequency_ * cos(2 * M_PI * sin_frequency_ * t_curr_ + sin_phase_);
  }
  // Set the reference to the joint posture task
  tsid::trajectories::TrajectorySample sample_posture_joint(ref.size());
  sample_posture_joint.setValue(ref);
  sample_posture_joint.setDerivative(vel_ref);
  task_joint_posture_->setReference(sample_posture_joint);
  
  compute_problem_and_set_command(state.first, state.second); //q and v

  return controller_interface::return_type::OK;
}

}  // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::SinJointSpaceController,
  controller_interface::ControllerInterface)
