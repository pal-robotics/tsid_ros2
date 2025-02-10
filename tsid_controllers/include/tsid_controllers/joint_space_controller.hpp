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

#ifndef JOINT_SPACE_TISD_CONTROLLER_HPP_
#define JOINT_SPACE_TISD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "tsid_controllers/tsid_position_control.hpp"
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsid_controllers_params.hpp>

namespace tsid_controllers
{

class JointSpaceTsidController : public TsidPositionControl
{
public:
  JointSpaceTsidController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  void setPositionCb(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_cmd_sub_;
};
} // namespace tsid_controllers

#endif // JOINT_SPACE_TISD_CONTROLLER_HPP_
