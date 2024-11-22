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

#include "dynamic_tsid_controller/dynamic_tsid_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace controller_interface;
// using hardware_interface::HW_IF_EFFORT;
// using hardware_interface::HW_IF_POSITION;
// using hardware_interface::HW_IF_VELOCITY;

namespace dynamic_tsid_controller
{
DynamicTsidController::DynamicTsidController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn DynamicTsidController::on_init()
{
  try {
    param_listener_ = std::make_shared<dynamic_tsid_controller::ParamListener>(get_node());

    if (!param_listener_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize ParamListener.");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicTsidController::on_configure(
  const rclcpp_lifecycle::State & /*prev_state*/)
{
  // Check if parameters were taken correctly
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Check if the actuator names are not empty
  if (params_.joint_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The joints name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create the state and command interfaces
  state_interfaces_.reserve(3 * params_.joint_names.size());
  command_interfaces_.reserve(params_.joint_names.size());
  joint_state_interfaces_.resize(params_.joint_names.size());
  state_interface_names_.resize(params_.joint_names.size());

  // Create the joint handles
  //Creating a map between index and joint
  int idx = 0;


  Interfaces pos_iface = Interfaces::position;
  Interfaces vel_iface = Interfaces::velocity;
  Interfaces eff_iface = Interfaces::effort;

  for (const auto & joint : params_.joint_names) {


    jnt_id_.insert(std::make_pair(joint, idx));

    joint_state_interfaces_[idx].reserve(3);

    // Create the state interfaces

    state_interface_names_[idx].resize(3);
    state_interface_names_[idx][pos_iface._value] = joint + "/" + pos_iface._to_string();
    state_interface_names_[idx][vel_iface._value] = joint + "/" + vel_iface._to_string();
    state_interface_names_[idx][eff_iface._value] = joint + "/" + eff_iface._to_string();


    idx++;
  }
  /* ADDING INITIALIZATION OF PINOCCHIO */

  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(), model_);

  RCLCPP_INFO(get_node()->get_logger(), "Model has been built, it has %d joints", model_.njoints);


  std::vector<pinocchio::JointIndex> joints_to_lock;
  /*VMO: In this point we make a list of joints to remove from the model (in tiago case wheel joints and end effector joints)
   These are actually the joints that we don't passs to the controller*/
  for (auto & name : model_.names) {
    if (name != "universe" && name != "root_joint" &&
      std::find(
        params_.joint_names.begin(), params_.joint_names.end(),
        name) == params_.joint_names.end())
    {
      joints_to_lock.push_back(model_.getJointId(name));
      RCLCPP_INFO(get_node()->get_logger(), "Lock joint %s: ", name.c_str());
    }
  }

  /* Removing the unused joints from the model*/
  model_ = buildReducedModel(model_, joints_to_lock, pinocchio::neutral(model_));

  for (auto joint : model_.names) {
    RCLCPP_INFO(get_node()->get_logger(), "Joint name: %s", joint.c_str());
  }

  /* VMO: if we need to check the mass of the model to verify that it is more or less realistic
  IPE: 18.21225 arms+torso ~= what we expected*/
  RCLCPP_INFO(
    get_node()->get_logger(), "Total mass according to the model %f",
    pinocchio::computeTotalMass(model_));

  // Initialization of the TSID
  robot_wrapper_ = new tsid::robots::RobotWrapper(
    model_,
    tsid::robots::RobotWrapper::FLOATING_BASE_SYSTEM);

  formulation_ = new tsid::InverseDynamicsFormulationAccForce("tsid", *robot_wrapper_, true);

  // Joint Posture Task

  auto posture_task = std::make_shared<tsid::tasks::TaskJointPosture>(
    "task-posture",
    *robot_wrapper_);
  Eigen::VectorXd kp = 100 * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6);
  Eigen::VectorXd kd = 2.0 * kp.cwiseSqrt();
  posture_task->Kp(kp);
  posture_task->Kd(kd);
  int posture_priority = 1;  // 0 constraint, 1 cost function
  double transition_time = 0.0;
  double posture_weight = 1e-4;

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  auto traj_posture = std::make_shared<tsid::trajectories::TrajectoryEuclidianConstant>(
    "traj_joint", q0.tail(robot_wrapper_->nv() - 6));
  tsid::trajectories::TrajectorySample sample_posture = traj_posture->computeNext();
  posture_task->setReference(sample_posture);
  formulation_->addMotionTask(*posture_task, posture_weight, posture_priority, transition_time);

  // Joint Bounds Task
  auto bounds_task = std::make_shared<tsid::tasks::TaskJointPosVelAccBounds>(
    "task-joint-bounds",
    *robot_wrapper_, this->get_update_rate());
  Eigen::VectorXd q_min = -M_PI * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6);
  Eigen::VectorXd q_max = M_PI * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6);
  bounds_task->setPositionBounds(q_min, q_max);
  int bounds_priority = 0;  // 0 constraint, 1 cost function
  double bounds_weight = 1e-4;
  formulation_->addMotionTask(*bounds_task, bounds_weight, bounds_priority, transition_time);

  // Joint velocity bounds
  double v_scaling = 1.0;
  Eigen::VectorXd v_max = v_scaling * model_.velocityLimit.tail(model_.nv - 6);
  bounds_task->setVelocityBounds(v_max);
  formulation_->addMotionTask(*bounds_task, bounds_weight, bounds_priority, transition_time);

  // End effector task
  auto ee_task = std::make_shared<tsid::tasks::TaskSE3Equality>(
    "task-ee",
    *robot_wrapper_,
    "arm_left_7_link");

  ee_task->Kp(300 * Eigen::VectorXd::Ones(6));
  ee_task->Kd(2.0 * ee_task->Kp().cwiseSqrt());
  Eigen::VectorXd ee_mask = Eigen::VectorXd::Zero(6);
  ee_mask << 1, 1, 1, 0, 0, 0;
  ee_task->setMask(ee_mask);
  ee_task->useLocalFrame(false);

  double ee_weight = 1;
  int ee_priority = 1;
  formulation_->addMotionTask(*ee_task, ee_weight, ee_priority, transition_time);


  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration DynamicTsidController::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;

  for (const auto & joint: params_.joint_names) {
    for (int i = 0; i < 3; i++) {
      state_interfaces_config_names.push_back(state_interface_names_[jnt_id_.at(joint)][i]);
    }
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};


}


controller_interface::InterfaceConfiguration
DynamicTsidController::command_interface_configuration() const
{

  std::vector<std::string> command_interfaces_config_names;
  for (const auto & joint : params_.joint_names) {
    const auto full_name = joint + "/effort";
    command_interfaces_config_names.push_back(full_name);
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};


}


controller_interface::CallbackReturn DynamicTsidController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Check if all the command interfaces are in the actuator interface
  for (const auto & joint : params_.joint_names) {
    if (
      !controller_interface::get_ordered_interfaces(
        state_interfaces_, state_interface_names_[jnt_id_[joint]],
        std::string(""),
        joint_state_interfaces_[jnt_id_[joint]]))
    {
      RCLCPP_ERROR(
        this->get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
        state_interface_names_[jnt_id_[joint]].size(),
        joint_state_interfaces_[jnt_id_[joint]].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicTsidController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  // TODO: set 0.0 torque to stop it
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type DynamicTsidController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // If the controller is not in tuning mode the gains are set only once
  RCLCPP_INFO(get_node()->get_logger(), "Updating controller");
  return controller_interface::return_type::OK;
}

}  // namespace dynamic_tsid_controller
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamic_tsid_controller::DynamicTsidController,
  controller_interface::ControllerInterface)
