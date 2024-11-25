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
using std::placeholders::_1;

DynamicTsidController::DynamicTsidController()
: controller_interface::ControllerInterface(),
  dt_(0, 0)
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
  // Pose reference callback
  ee_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "dynamic_tsid_controller/pose_cmd", 1,
    std::bind(&DynamicTsidController::setPoseCallback, this, _1));

  /* ADDING INITIALIZATION OF PINOCCHIO */

  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(), pinocchio::JointModelFreeFlyer(), model_);

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
  model_ =
    buildReducedModel(model_, joints_to_lock, pinocchio::neutral(model_));

  for (auto joint : model_.names) {
    RCLCPP_INFO(get_node()->get_logger(), "Joint name: %s", joint.c_str());
  }

  /* VMO: if we need to check the mass of the model to verify that it is more or less realistic
  IPE: 18.21225 arms+torso ~= what we expected*/
  RCLCPP_INFO(
    get_node()->get_logger(), "Total mass according to the model %f",
    pinocchio::computeTotalMass(model_));

  // Getting control period
  dt_ = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / this->get_update_rate()));
  RCLCPP_INFO(get_node()->get_logger(), "Control period: %f", dt_.seconds());
  // Initialization of the TSID
  robot_wrapper_ = new tsid::robots::RobotWrapper(
    model_,
    tsid::robots::RobotWrapper::RootJointType::FLOATING_BASE_SYSTEM, true);

  formulation_ = new tsid::InverseDynamicsFormulationAccForce("tsid", *robot_wrapper_, true);

  // Joint Posture Task

  task_joint_posture_ = new tsid::tasks::TaskJointPosture(
    "task-joint-posture",
    *robot_wrapper_);
  Eigen::VectorXd kp = 100 * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6);
  Eigen::VectorXd kd = 2.0 * kp.cwiseSqrt();
  task_joint_posture_->Kp(kp);
  task_joint_posture_->Kd(kd);
  int posture_priority = 1;  // 0 constraint, 1 cost function
  double transition_time = 0.0;
  double posture_weight = 1e-4;

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  traj_joint_posture_ = new tsid::trajectories::TrajectoryEuclidianConstant(
    "traj_joint", q0.tail(robot_wrapper_->nv() - 6));
  tsid::trajectories::TrajectorySample sample_posture = traj_joint_posture_->computeNext();
  task_joint_posture_->setReference(sample_posture);
  formulation_->addMotionTask(
    *task_joint_posture_, posture_weight, posture_priority,
    transition_time);


  // Joint Bounds Task
  task_joint_bounds_ = new tsid::tasks::TaskJointBounds(
    "task-joint-bounds",
    *robot_wrapper_, dt_.seconds());
  Eigen::VectorXd q_min = model_.lowerPositionLimit.tail(model_.nv - 6);
  Eigen::VectorXd q_max = model_.upperPositionLimit.tail(model_.nv - 6);

  int bounds_priority = 0;  // 0 constraint, 1 cost function
  double bounds_weight = 1;

  // Joint velocity bounds
  double v_scaling = 1.0;
  Eigen::VectorXd v_max = v_scaling * model_.velocityLimit.tail(model_.nv - 6);
  Eigen::VectorXd v_min = -v_scaling * model_.velocityLimit.tail(model_.nv - 6);
  task_joint_bounds_->setVelocityBounds(v_min, v_max);

  for (int i = 0; i < v_min.size(); i++) {
    RCLCPP_INFO(
      get_node()->get_logger(), "position: %f", v_max[i]
    );
  }
  formulation_->addMotionTask(*task_joint_bounds_, bounds_weight, bounds_priority, transition_time);

  // End effector task
  task_ee_ = new tsid::tasks::TaskSE3Equality(
    "task-ee", *robot_wrapper_, "arm_left_7_link");
  Eigen::VectorXd kp_gain = Eigen::VectorXd::Zero(6);
  kp_gain << 1000, 1000, 1000, 0, 0, 0;
  task_ee_->Kp(kp_gain);
  task_ee_->Kd(2.0 * task_ee_->Kp().cwiseSqrt());
  Eigen::VectorXd ee_mask = Eigen::VectorXd::Zero(6);
  ee_mask << 1, 1, 1, 0, 0, 0;
  task_ee_->setMask(ee_mask);
  task_ee_->useLocalFrame(false);

  double ee_weight = 1;
  int ee_priority = 1;
  formulation_->addMotionTask(*task_ee_, ee_weight, ee_priority, transition_time);

  // Initializing solver
  solver_ = new tsid::solvers::SolverHQuadProgFast("qp solver");

  solver_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());

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
    const auto full_name = joint + "/position";
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

  for (const auto & joint : params_.joint_names) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Joint %s position: %f",
      joint.c_str(), joint_state_interfaces_[jnt_id_[joint]][0].get().get_value());
  }

  // Taking initial position from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nq());

  for (const auto & joint : params_.joint_names) {
    q0.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][0].get().get_value();
  }

  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());
  formulation_->computeProblemData(0.0, q0, v0);

  // Setting posture task reference as initial position
  traj_joint_posture_->setReference(q0.tail(robot_wrapper_->nq() - 7));
  task_joint_posture_->setReference(traj_joint_posture_->computeNext());


  // Setting initial reference for the end effector task
  H_ee_0_ =
    robot_wrapper_->framePosition(formulation_->data(), model_.getFrameId("arm_left_7_link"));
  traj_ee_ = new tsid::trajectories::TrajectorySE3Constant("traj_ee", H_ee_0_);
  tsid::trajectories::TrajectorySample sample_posture_ee = traj_ee_->computeNext();
  task_ee_->setReference(sample_posture_ee);


  RCLCPP_INFO(
    get_node()->get_logger(), " Initial position ee : %f %f %f",
    H_ee_0_.translation()[0], H_ee_0_.translation()[1], H_ee_0_.translation()[2]);

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

  // setting reference pos for ee


  // Taking current state
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  q[6] = 1.0;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  for (const auto & joint : params_.joint_names) {
    q.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][Interfaces::position].get().get_value();
    v.tail(robot_wrapper_->nv() - 6)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][Interfaces::velocity].get().get_value();

  }

  // Computing the problem data
  const tsid::solvers::HQPData solverData = formulation_->computeProblemData(0.0, q, v);

  // Solving the problem
  const auto sol = solver_->solve(solverData);

  /*qp_status_ = sol.status;
  if (sol.status == tsid::solvers::HQP_STATUS_OPTIMAL) {
    desiredAccelerationPinocchio_ = invdyn_->getAccelerations(sol);
  }*/

  // Integrating acceleration to get velocity
  Eigen::VectorXd a = formulation_->getAccelerations(sol);
  Eigen::VectorXd v_cmd = v + a * 0.5 * dt_.seconds();

  // Integrating velocity to get position
  auto q_int = pinocchio::integrate(model_, q, v_cmd * dt_.seconds());

  auto q_cmd = q_int.tail(model_.nq - 7);

  // Setting the command to the joint command interfaces
  for (const auto & joint : params_.joint_names) {
    command_interfaces_[jnt_id_[joint]].set_value(
      q_cmd[model_.getJointId(joint) - 2]);
  }

/*  auto h_ee =
    robot_wrapper_->framePosition(formulation_->data(), model_.getFrameId("arm_left_7_link"));

  RCLCPP_INFO(
    get_node()->get_logger(), " Current position ee : %f %f %f",
    h_ee.translation()[0], h_ee.translation()[1], h_ee.translation()[2]);
    */

  return controller_interface::return_type::OK;
}

void DynamicTsidController::setPoseCallback(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  desired_pose_ << msg->data[0], msg->data[1], msg->data[2];

  auto h_ee =
    robot_wrapper_->framePosition(formulation_->data(), model_.getFrameId("arm_left_7_link"));

  RCLCPP_INFO(
    get_node()->get_logger(), " Current position ee : %f %f %f",
    h_ee.translation()[0], h_ee.translation()[1], h_ee.translation()[2]);

  Eigen::Vector3d pos_x_des = h_ee.translation() + desired_pose_;
  Eigen::VectorXd ref = Eigen::VectorXd::Zero(12);
  ref.head(3) = pos_x_des;
  ref.tail(9) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0; // useless because mask not taking orientation, but required from tsid to put at least identity
  tsid::trajectories::TrajectorySample sample_posture_ee = traj_ee_->computeNext();

  sample_posture_ee.setValue(ref);

  task_ee_->setReference(sample_posture_ee);

  auto ref_ee = task_ee_->getReference();

  auto ref_pos = ref_ee.getValue();
  RCLCPP_INFO(
    get_node()->get_logger(), " Reference position ee : %f %f %f",
    ref_pos[0], ref_pos[1], ref_pos[2]);

}


}  // namespace dynamic_tsid_controller
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamic_tsid_controller::DynamicTsidController,
  controller_interface::ControllerInterface)
