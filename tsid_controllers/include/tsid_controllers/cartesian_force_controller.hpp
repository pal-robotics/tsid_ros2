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

#ifndef CARTESIAN_SPACE_CONTROLLER_HPP_
#define CARTESIAN_SPACE_CONTROLLER_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "pal_utils/better_enums.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_components/force_torque_sensor.hpp"

// Generated include by generate_parameter_library.
#include "tsid_controllers/tsid_controllers_params.hpp"
#include "tsid_controller_msgs/msg/ee_wrench.hpp"
#include "tsid_controllers/filters/force_derivative_filter.hpp"


namespace tsid_controllers
{
BETTER_ENUM(Interfaces, int, position = 0, velocity = 1, effort = 2);


class CartesianForceController
  : public controller_interface::ControllerInterface
{
public:
  CartesianForceController();


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

  void setWrenchCallback(
    tsid_controller_msgs::msg::EeWrench::ConstSharedPtr msg);

private:
  tsid_controllers::Params params_;
  std::shared_ptr<tsid_controllers::ParamListener> param_listener_;

  std::vector<std::string> joint_names_;
  std::array<std::vector<std::string>, 3> state_interface_names_;
  std::array<std::vector<std::reference_wrapper<
      hardware_interface::LoanedStateInterface>>, 3> joint_state_interfaces_;
  std::map<std::string, int> jnt_id_;

  std::vector<std::string> joint_command_names_;
  std::vector<std::string> command_interfaces_names_;
  std::vector
  <std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  joint_command_interfaces_;
  std::map<std::string, int> jnt_command_id_;

  std::vector<std::string> ee_names_;
  std::map<std::string, int> ee_id_;
  std::map<std::string, semantic_components::ForceTorqueSensor> ft_sensors_;
  std::map<std::string, std::vector<std::string>> ft_sensor_interfaces_;
  std::map<std::string, ForceDerivativeFilter> force_filters_;

  pinocchio::Data data_;

  bool local_frame_ = false;
  bool first_update_ = true;
  bool tuning_mode_ = false;

  /// Pinocchio objects:
  pinocchio::Model model_;

  /// TSID objects:
  std::vector<pinocchio::SE3> H_ee_0_;
  /// @brief Force trajectory for each end effector.
  rclcpp::Duration dt_;
  /// @brief Reference trajectory.
  rclcpp::Subscription<tsid_controller_msgs::msg::EeWrench>::SharedPtr ee_cmd_sub_;

  /// @brief Sensor data.
  struct SensorData
  {
    Eigen::VectorXd q, v;
    std::map<std::string, pinocchio::Force> fts;
    std::map<std::string, pinocchio::Force> dfts;
  } sensor_data_;

  std::vector<Eigen::Vector3d> desired_pose_;
  double controller_current_time_ = std::numeric_limits<double>::quiet_NaN();
  double controller_time_origin_ = std::numeric_limits<double>::quiet_NaN();

  const std::array<std::string, 3> interface_types_ = {{
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT}};

  /*
   * Debug variables.
   */
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_curr_pos_;
  static const bool debug_backdoor_;

private:
  /**
   * @brief Update the parameters from ROS if there are some new ones.
   *
   */
  void updateParams();

  /**
   * @brief Read the sensor data and populate sensor_data_.
   */
  void read_sensor_data();
};

} //  namespace tsid_controllers

#endif  // CARTESIAN_SPACE_CONTROLLER_HPP_
