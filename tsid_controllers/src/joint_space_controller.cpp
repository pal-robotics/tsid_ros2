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
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

JointSpaceTsidController::JointSpaceTsidController()
: tsid_controllers::TsidPositionControl(),
  dt_(0, 0)
{
}

controller_interface::CallbackReturn JointSpaceTsidController::on_init()
{
  return TsidPositionControl::on_init();
}

controller_interface::CallbackReturn JointSpaceTsidController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  // Position command
  joint_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "tsid_controllers/joint_position_cmd", 1,
    std::bind(&JointSpaceTsidController::setPositionCb, this, _1));

  return TsidPositionControl::on_configure(prev_state);
}


controller_interface::CallbackReturn JointSpaceTsidController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return TsidPositionControl::on_activate(previous_state);
}

controller_interface::return_type JointSpaceTsidController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  TsidPositionControl::updateParams(); //updateParams()??
  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;
  compute_problem_and_set_command(state.first,state.second); //q and v
  return controller_interface::return_type::OK;
}

void JointSpaceTsidController::setPositionCb(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  setDesiredRef(msg);
}


}  // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tsid_controllers::JointSpaceTsidController, controller_interface::ControllerInterface)
