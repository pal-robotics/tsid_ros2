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
  // Position command
  joint_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "tsid_controllers/joint_position_cmd", 1,
    std::bind(&JointSpaceTsidController::setPositionCb, this, _1));

  return TsidPositionControl::on_configure(prev_state);
}


controller_interface::return_type
JointSpaceTsidController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  TsidPositionControl::updateParams();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;
  compute_problem_and_set_command(state.first, state.second); // q and v
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

  tsid::trajectories::TrajectorySample sample_posture_joint(ref.size());
  sample_posture_joint.setValue(ref);

  task_joint_posture_->setReference(sample_posture_joint);

  auto get_ref = task_joint_posture_->getReference();
  auto ref_pos = get_ref.getValue();
  RCLCPP_INFO(
    get_node()->get_logger(), " Reference position joints : %f ",
    ref_pos[0]);
}

} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::JointSpaceTsidController,
  controller_interface::ControllerInterface)
