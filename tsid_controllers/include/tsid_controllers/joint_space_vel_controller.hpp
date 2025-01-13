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

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include "hardware_interface/component_parser.hpp"
#include "pal_utils/better_enums.hpp"
#include <tsid_controllers_params.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-rt.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-fast.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tsid_controllers/tasks/task-vel-joint.hpp"


namespace tsid_controllers
{
BETTER_ENUM(
  Interfaces, int, position = 0, velocity = 1, effort = 2);


class JointSpaceVelTsidController
  : public controller_interface::ControllerInterface
{
public:
  JointSpaceVelTsidController();


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

  void setVelocityCb(
    std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

protected:
  tsid_controllers::Params params_;
  std::shared_ptr<tsid_controllers::ParamListener> param_listener_;
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
  tsid::robots::RobotWrapper * robot_wrapper_;
  tsid::InverseDynamicsFormulationAccForce * formulation_;
  tsid::solvers::SolverHQuadProgFast * solver_;
  tsid::tasks::TaskJointVel * task_joint_velocity_;
  tsid::tasks::TaskJointBounds * task_joint_bounds_;
  tsid::trajectories::TrajectoryEuclidianConstant * traj_joint_velocity_;
  rclcpp::Duration dt_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
  std::vector<double> desired_pose_;
  double v_scaling_;


  void updateParams();


};
}

#endif  // JOINT_SPACE_TISD_CONTROLLER_HPP_
