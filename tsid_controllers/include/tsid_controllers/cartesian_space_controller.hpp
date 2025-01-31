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

#include "geometry_msgs/msg/pose.hpp"
#include "tsid_controller_msgs/msg/ee_pos.hpp"
#include "tsid_controllers/tsid_position_control.hpp"
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid_controllers_params.hpp>

namespace tsid_controllers {
class CartesianSpaceController : public TsidPositionControl {

public:
  CartesianSpaceController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  void setPoseCallback(tsid_controller_msgs::msg::EePos::ConstSharedPtr msg);

protected:
  const auto &getParams() const { return TsidPositionControl::params_; }

private:
  std::map<std::string, int> ee_id_;
  bool local_frame_ = false;
  std::vector<tsid::tasks::TaskSE3Equality *> task_ee_;
  std::vector<pinocchio::SE3> H_ee_0_;
  std::vector<tsid::trajectories::TrajectorySE3Constant> traj_ee_;
  const tsid::trajectories::TrajectorySample sample_posture_ee_;
  rclcpp::Duration dt_;
  rclcpp::Subscription<tsid_controller_msgs::msg::EePos>::SharedPtr ee_cmd_sub_;
  std::vector<Eigen::Vector3d> desired_pose_;

  std::vector<std::string> ee_names_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_curr_pos;
};
} // namespace tsid_controllers

#endif // CARTESIAN_SPACE_CONTROLLER_HPP_
