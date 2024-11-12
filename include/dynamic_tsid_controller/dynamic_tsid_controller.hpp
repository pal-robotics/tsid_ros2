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

#ifndef DYNAMIC_TISD_CONTROLLER_HPP_
#define DYNAMIC_TISD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include "hardware_interface/component_parser.hpp"
#include "pal_utils/better_enums.hpp"
#include <dynamic_tsid_controller_params.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>


namespace dynamic_tsid_controller
{
BETTER_ENUM(
  Interfaces, int, position = 0, velocity = 1, effort = 2);


class DynamicTsidController
  : public controller_interface::ControllerInterface
{
public:
  DynamicTsidController();


  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

protected:
  dynamic_tsid_controller::Params params_;
  std::shared_ptr<dynamic_tsid_controller::ParamListener> param_listener_;
  std::vector<std::vector<std::string>> state_interface_names_;

  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_;

private:
  std::map<std::string, int> jnt_id_;
  bool first_update_ = true;
  bool tuning_mode_ = false;
  pinocchio::Model model_;
  pinocchio::Data data_;

};
}

#endif  // DYNAMIC_TISD_CONTROLLER_HPP_
