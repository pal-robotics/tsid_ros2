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
#include <tsid/trajectories/trajectory-se3.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "tsid_controller_msgs/msg/ee_pos.hpp"


namespace tsid_controllers
{
BETTER_ENUM(
  Interfaces, int, position = 0, velocity = 1, effort = 2);


class CartesianSpaceController
  : public controller_interface::ControllerInterface
{
public:
  CartesianSpaceController();


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

  void setPoseCallback(
    tsid_controller_msgs::msg::EePos::ConstSharedPtr msg);

protected:
  tsid_controllers::Params params_;
  std::shared_ptr<tsid_controllers::ParamListener> param_listener_;
  std::vector<std::vector<std::string>> state_interface_names_;

  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_;

private:
  std::map<std::string, int> jnt_id_;
  std::map<std::string, int> jnt_command_id_;
  std::map<std::string, int> ee_id_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> joint_command_names_;
  bool local_frame = false;
  bool first_update_ = true;
  bool tuning_mode_ = false;
  pinocchio::Model model_;
  pinocchio::Data data_;
  tsid::robots::RobotWrapper * robot_wrapper_;
  tsid::InverseDynamicsFormulationAccForce * formulation_;
  tsid::solvers::SolverHQuadProgFast * solver_;
  tsid::tasks::TaskJointPosture * task_joint_posture_;
  tsid::tasks::TaskJointBounds * task_joint_bounds_;
  tsid::trajectories::TrajectoryEuclidianConstant * traj_joint_posture_;
  std::vector<tsid::tasks::TaskSE3Equality *> task_ee_;
  std::vector<pinocchio::SE3> H_ee_0_;
  std::vector<tsid::trajectories::TrajectorySE3Constant> traj_ee_;
  const tsid::trajectories::TrajectorySample sample_posture_ee_;
  rclcpp::Duration dt_;
  rclcpp::Subscription<tsid_controller_msgs::msg::EePos>::SharedPtr ee_cmd_sub_;
  std::vector<Eigen::Vector3d> desired_pose_;

  std::vector<std::string> ee_names_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_curr_pos;


  void updateParams();


};
}

#endif  // CARTESIAN_SPACE_CONTROLLER_HPP_
