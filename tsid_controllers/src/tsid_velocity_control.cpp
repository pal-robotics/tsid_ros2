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

#include "tsid_controllers/tsid_velocity_control.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pluginlib/class_list_macros.hpp>

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

TsidVelocityControl::TsidVelocityControl()
: controller_interface::ControllerInterface(), dt_(0, 0) {}

controller_interface::CallbackReturn TsidVelocityControl::on_init()
{
  try {
    param_listener_ =
      std::make_shared<tsid_controllers::ParamListener>(get_node());

    if (!param_listener_) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to initialize ParamListener.");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during controller's init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TsidVelocityControl::on_configure(
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
  if (params_.joint_state_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The joints name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  } else {
    joint_names_ = params_.joint_state_names;
  }

  // Check if command joint names are not empty
  if (params_.joint_command_names.empty()) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "The joint command names is empty. Joint state will be used");
    joint_command_names_ = params_.joint_state_names;
  } else {
    joint_command_names_.resize(params_.joint_command_names.size());
    for (size_t i = 0; i < params_.joint_command_names.size(); i++) {
      size_t start = params_.joint_command_names[i].find("/");
      if (start != std::string::npos) {
        auto joint = params_.joint_command_names[i].substr(start + 1);
        joint_command_names_[i] = joint;

      } else {
        joint_command_names_[i] = params_.joint_command_names[i];
      }
    }
  }


  // Create the state and command interfaces
  state_interfaces_.reserve(3 * joint_names_.size());
  command_interfaces_.reserve(joint_command_names_.size());
  joint_state_interfaces_.resize(joint_names_.size());
  state_interface_names_.resize(joint_names_.size());

  q_prev_ = Eigen::VectorXd::Zero(joint_names_.size());

  // Creating a map between index and joint
  int idx = 0;

  Interfaces pos_iface = Interfaces::position;
  Interfaces vel_iface = Interfaces::velocity;
  Interfaces eff_iface = Interfaces::effort;

  for (const auto & joint : joint_names_) {

    jnt_id_.insert(std::make_pair(joint, idx));

    joint_state_interfaces_[idx].reserve(3);

    // Create the state interfaces
    state_interface_names_[idx].resize(3);
    state_interface_names_[idx][pos_iface._value] =
      joint + "/" + pos_iface._to_string();
    state_interface_names_[idx][vel_iface._value] =
      joint + "/" + vel_iface._to_string();
    state_interface_names_[idx][eff_iface._value] =
      joint + "/" + eff_iface._to_string();

    idx++;
  }

  idx = 0;

  // Creating a map for command joint
  for (const auto & joint : joint_command_names_) {
    jnt_command_id_.insert(std::make_pair(joint, idx));
    idx++;
  }

  // Creating model in pinocchio
  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(),
    pinocchio::JointModelFreeFlyer(), model_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Model has been built, it has %d joints", model_.njoints);

  std::vector<pinocchio::JointIndex> joints_to_lock;

  for (auto & name : model_.names) {
    if (name != "universe" && name != "root_joint" &&
      std::find(joint_names_.begin(), joint_names_.end(), name) ==
      joint_names_.end())
    {
      joints_to_lock.push_back(model_.getJointId(name));
      RCLCPP_INFO(get_node()->get_logger(), "Lock joint %s: ", name.c_str());
    }
  }

  /* Removing the unused joints from the model*/
  model_ =
    buildReducedModel(model_, joints_to_lock, pinocchio::neutral(model_));

  for (auto joint : model_.names) {
    RCLCPP_INFO(get_node()->get_logger(), "Joint name: %s", joint.c_str());
  }

  // Getting control period
  dt_ = rclcpp::Duration(
    std::chrono::duration<double, std::milli>(1e3 / this->get_update_rate()));
  RCLCPP_INFO(get_node()->get_logger(), "Control period: %f", dt_.seconds());

  // Initialization of the TSID
  robot_wrapper_ = new tsid::robots::RobotWrapper(
    model_, tsid::robots::RobotWrapper::RootJointType::FLOATING_BASE_SYSTEM,
    true);

  formulation_ = new tsid::InverseDynamicsFormulationAccForce(
    "tsid", *robot_wrapper_, true);

  first_tsid_iter_ = true;

  DefaultVelocityTasks();

  // Initializing solver
  solver_ = new tsid::solvers::SolverHQuadProgFast("qp solver");

  solver_->resize(
    formulation_->nVar(), formulation_->nEq(),
    formulation_->nIn());

  publisher_curr_vel_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("tsid_cmd_vel", 10);

  publisher_curr_pos_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("tsid_cmd_pos", 10);


  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration
TsidVelocityControl::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names;

  for (const auto & joint : joint_names_) {
    for (int i = 0; i < 3; i++) {
      state_interfaces_config_names.push_back(
        state_interface_names_[jnt_id_.at(joint)][i]);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    state_interfaces_config_names};
}

controller_interface::InterfaceConfiguration
TsidVelocityControl::command_interface_configuration() const
{

  std::vector<std::string> command_interfaces_config_names;
  for (const auto & joint : params_.joint_command_names) {
    const auto full_name = joint + "/velocity";
    command_interfaces_config_names.push_back(full_name);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}
controller_interface::CallbackReturn TsidVelocityControl::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Check if all the command interfaces are in the actuator interface
  for (const auto & joint : joint_names_) {
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, state_interface_names_[jnt_id_[joint]],
        std::string(""), joint_state_interfaces_[jnt_id_[joint]]))
    {
      RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Expected %zu state interfaces, got %zu",
        state_interface_names_[jnt_id_[joint]].size(),
        joint_state_interfaces_[jnt_id_[joint]].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  for (const auto & joint : joint_names_) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Joint %s position: %f",
      joint.c_str(),
      joint_state_interfaces_[jnt_id_[joint]][0].get().get_optional().value());
  }

  // Taking initial position from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();

  q0 = state.first;
  v0 = state.second;

  q_prev_ = q0.tail(robot_wrapper_->nq() - 7);

  formulation_->computeProblemData(0.0, q0, v0);

  // Setting posture task reference as initial position
  /* traj_joint_posture_->setReference(q0.tail(robot_wrapper_->nq() - 7));
   task_joint_posture_->setReference(traj_joint_posture_->computeNext());
 */
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TsidVelocityControl::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  release_interfaces();
  for (auto joint : params_.joint_state_names) {
    joint_state_interfaces_[jnt_id_[joint]].clear();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
TsidVelocityControl::getActualState() const
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  for (const auto & joint : joint_names_) {
    q.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_.at(joint)][Interfaces::position]
      .get()
      .get_optional().value();
    v.tail(robot_wrapper_->nv() - 6)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_.at(joint)][Interfaces::velocity]
      .get()
      .get_optional().value();
  }

  return std::make_pair(q, v);
}

void TsidVelocityControl::updateParams()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Updating parameters");

    v_scaling_ = params_.velocity_scaling;
    v_max_ =
      v_scaling_ * model_.velocityLimit.tail(model_.nv - 6);
    task_joint_bounds_->setVelocityBounds(v_max_);
    /*  task_joint_posture_->Kp(
        params_.posture_gain *
        Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6));
      task_joint_posture_->Kd(2.0 * task_joint_posture_->Kp().cwiseSqrt());*/
  }
}

void TsidVelocityControl::DefaultVelocityTasks()
{

  // Joint Posture Task
  /*task_joint_posture_ =
    new tsid::tasks::TaskJointPosture("task-joint-posture", *robot_wrapper_);
  Eigen::VectorXd kp =
    params_.posture_gain * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6);
  Eigen::VectorXd kd = 2.0 * kp.cwiseSqrt();
  task_joint_posture_->Kp(kp);
  task_joint_posture_->Kd(kd);
  int posture_priority = 1; // 0 constraint, 1 cost function
  double transition_time = 0.0;
  double posture_weight = 1e-3;

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  traj_joint_posture_ = new tsid::trajectories::TrajectoryEuclidianConstant(
    "traj_joint", q0.tail(robot_wrapper_->nv() - 6));
  tsid::trajectories::TrajectorySample sample_posture =
    traj_joint_posture_->computeNext();
  task_joint_posture_->setReference(sample_posture);
  formulation_->addMotionTask(
    *task_joint_posture_, posture_weight,
    posture_priority, transition_time);*/

  // Joint Bounds Task
  task_joint_bounds_ = new tsid::tasks::TaskJointPosVelAccBounds(
    "task-joint-bounds", *robot_wrapper_, dt_.seconds(), false);
  q_min_ = model_.lowerPositionLimit.tail(model_.nv - 6);
  q_max_ = model_.upperPositionLimit.tail(model_.nv - 6);

  for (Eigen::Index i = 0; i < q_max_.size(); i++) {
    std::cout << "q_max" << q_max_[i] << std::endl;
    std::cout << "q_min" << q_min_[i] << std::endl;
  }

  task_joint_bounds_->setPositionBounds(q_min_, q_max_);

  double transition_time = 0.0;
  int bounds_priority = 0; // 0 constraint, 1 cost function
  double bounds_weight = 1;
  // Joint velocity bounds
  v_scaling_ = params_.velocity_scaling;
  v_max_ = v_scaling_ * model_.velocityLimit.tail(model_.nv - 6);
  Eigen::VectorXd v_min = -v_max_;
  task_joint_bounds_->setVelocityBounds(v_max_);
  formulation_->addMotionTask(
    *task_joint_bounds_, bounds_weight,
    bounds_priority, transition_time);
}

void TsidVelocityControl::compute_problem_and_set_command(
  const Eigen::VectorXd q, const Eigen::VectorXd v)
{
  Eigen::VectorXd v_ = Eigen::VectorXd::Zero(model_.nv);
  v_.tail(model_.nq - 7) = (q.tail(model_.nq - 7) - q_prev_) / dt_.seconds();

  // Computing the problem data
  const tsid::solvers::HQPData solverData =
    formulation_->computeProblemData(0.0, q, v);

  // Solving the problem
  const auto sol = solver_->solve(solverData);

  Eigen::VectorXd a;
  Eigen::VectorXd v_cmd;
  Eigen::VectorXd q_cmd;

  a = formulation_->getAccelerations(sol);

  v_cmd = v + a * 0.5 * dt_.seconds();

  if (first_tsid_iter_ || joint_limit_reached_) {
    q_int_ = q;
    first_tsid_iter_ = false;
  }

  // Integrating velocity to get position
  q_int_ = pinocchio::integrate(model_, q_int_, v_cmd * dt_.seconds());

  q_cmd = q_int_.tail(model_.nq - 7);

  auto v_com = v_cmd.tail(model_.nv - 6);
  double threshold = 0.2;

  int indx = 0;
  joint_limit_reached_ = false;
  while (!joint_limit_reached_ && indx < static_cast<int>(joint_command_names_.size())) {

    auto joint = joint_command_names_[indx];

    if (model_.joints[model_.getJointId(joint)].shortname().find("P") != std::string::npos) {
      threshold = 0.02;
    } else if (model_.joints[model_.getJointId(joint)].shortname().find("R") != std::string::npos) {
      threshold = -v.tail(model_.nv - 6)[model_.getJointId(joint) - 2] / (2 * (-20)) + 0.02;
    }

    if (std::abs(
        q.tail(model_.nq - 7)[model_.getJointId(joint) - 2] -
        q_min_[model_.getJointId(joint) - 2]) < threshold)
    {

      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Joint %s reached the lower limit: q = %f, q_min = %f",
        joint.c_str(), q.tail(model_.nq - 7)[model_.getJointId(joint) - 2],
        q_min_[model_.getJointId(joint) - 2]);

      q_cmd[model_.getJointId(joint) - 2] = q.tail(model_.nq - 7)[model_.getJointId(joint) - 2];

      if (v_com[model_.getJointId(joint) - 2] < 0) {
        joint_limit_reached_ = true;
        v_com[model_.getJointId(joint) - 2] = 0;
      } else {
        joint_limit_reached_ = false;
      }
    } else if (std::abs(
        q.tail(model_.nq - 7)[model_.getJointId(joint) - 2] -
        q_max_[model_.getJointId(joint) - 2]) < threshold)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Joint %s reached the upper limit: q = %f, q_max = %f",
        joint.c_str(), q.tail(
          model_.nq - 7)[model_.getJointId(joint) - 2], q_max_[model_.getJointId(joint) - 2]);

      q_cmd[model_.getJointId(joint) - 2] = q.tail(model_.nq - 7)[model_.getJointId(joint) - 2];

      if (v_com[model_.getJointId(joint) - 2] > 0) {
        joint_limit_reached_ = true;

        v_com[model_.getJointId(joint) - 2] = 0;
      } else {
        joint_limit_reached_ = false;
      }
    }
    indx++;
  }

  for (auto joint : joint_command_names_) {
    if (joint_limit_reached_) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Joint limit reached");

      if (!command_interfaces_[jnt_command_id_[joint]].set_value(
          0.0))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to set command for joint %s", joint.c_str());
      }
    } else {
      if (!command_interfaces_[jnt_command_id_[joint]].set_value(
          v_com[model_.getJointId(joint) - 2]))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to set command for joint %s", joint.c_str());
      }
    }
  }

  q_prev_ = q.tail(model_.nq - 7);
  std_msgs::msg::Float64MultiArray pub;

  for (Eigen::Index i = 0; i < v_com.size(); i++) {
    pub.data.push_back(v_com[i]);
  }
  publisher_curr_vel_->publish(pub);

  std_msgs::msg::Float64MultiArray pub_pos;
  for (Eigen::Index i = 0; i < q_cmd.size(); i++) {
    pub_pos.data.push_back(q_cmd[i]);
  }
  publisher_curr_pos_->publish(pub_pos);

}

} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::TsidVelocityControl,
  controller_interface::ControllerInterface)
