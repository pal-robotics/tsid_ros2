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

#ifndef CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "tsid_controllers/tsid_velocity_control.hpp"
#include "hardware_interface/component_parser.hpp"
#include "pal_utils/better_enums.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tsid_controller_msgs/msg/ee_pos.hpp"
#include "tsid_controllers/tasks/task-cartesian-velocity.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-fast.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-rt.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>

namespace tsid_controllers
{

class CartesianVelocityController
  : public TsidVelocityControl
{
public:
  CartesianVelocityController();

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  void setVelCallback(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

  void updateParams() override;

protected:
  const auto & getParams() const {return TsidVelocityControl::params_;}

private:
  std::map<std::string, int> ee_id_;
  tsid::tasks::TaskJointPosture * task_joint_posture_;
  tsid::trajectories::TrajectoryEuclidianConstant * traj_joint_posture_;
  std::vector<tsid::tasks::TaskCartesianVelocity *> task_ee_;
  std::vector<pinocchio::SE3> H_ee_0_;
  std::vector<tsid::trajectories::TrajectoryEuclidianConstant> traj_ee_;
  const tsid::trajectories::TrajectorySample sample_posture_ee_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_cmd_sub_;

  std::vector<std::string> ee_names_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_curr_pos_;

};
} // namespace tsid_controllers

#endif // CARTESIAN_VELOCITY_CONTROLLER_HPP_
