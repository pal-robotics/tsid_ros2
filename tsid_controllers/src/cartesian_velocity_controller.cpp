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

#include "tsid_controllers/cartesian_velocity_controller.hpp"
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

namespace tsid_controllers
{
using std::placeholders::_1;

CartesianVelocityController::CartesianVelocityController()
: controller_interface::ControllerInterface(),
  dt_(0, 0)
{
}

controller_interface::CallbackReturn CartesianVelocityController::on_init()
{
  try {
    param_listener_ = std::make_shared<tsid_controllers::ParamListener>(get_node());

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

controller_interface::CallbackReturn CartesianVelocityController::on_configure(
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
  if (params_.joint_command_names.empty() ) {
    RCLCPP_INFO(
      get_node()->get_logger(), "The joint command names is empty. Joint state will be used");
    joint_command_names_ = params_.joint_state_names;
  } else {
    joint_command_names_ = params_.joint_command_names;
  }

  // Storing names of desired end effector
  ee_names_.resize(params_.ee_names.size());
  desired_pose_.resize(params_.ee_names.size());
  H_ee_0_.resize(params_.ee_names.size());

  for (size_t i = 0; i < params_.ee_names.size(); i++) {
    ee_id_.insert(std::make_pair(params_.ee_names[i], i));
    ee_names_[i] = params_.ee_names[i];
  }

  // Create the state and command interfaces
  state_interfaces_.reserve(3 * joint_names_.size());
  command_interfaces_.reserve(joint_command_names_.size());
  joint_state_interfaces_.resize(joint_names_.size());
  state_interface_names_.resize(joint_names_.size());

  // TO REMOVE; creating publisher for current pose ee
  publisher_curr_pos =
    get_node()->create_publisher<geometry_msgs::msg::Pose>("current_position", 10);


  // Create the joint handles
  //Creating a map between index and joint
  int idx = 0;


  Interfaces pos_iface = Interfaces::position;
  Interfaces vel_iface = Interfaces::velocity;
  Interfaces eff_iface = Interfaces::effort;

  for (const auto & joint : joint_names_) {


    jnt_id_.insert(std::make_pair(joint, idx));

    joint_state_interfaces_[idx].reserve(3);

    // Create the state interfaces

    state_interface_names_[idx].resize(3);
    state_interface_names_[idx][pos_iface._value] = joint + "/" + pos_iface._to_string();
    state_interface_names_[idx][vel_iface._value] = joint + "/" + vel_iface._to_string();
    state_interface_names_[idx][eff_iface._value] = joint + "/" + eff_iface._to_string();


    idx++;
  }

  idx = 0;
  // Creating a map for command joint
  for (const auto & joint : joint_command_names_) {
    jnt_command_id_.insert(std::make_pair(joint, idx));
    idx++;
  }
  // Pose reference callback
  ee_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "tsid_controllers/vel_cmd", 1,
    std::bind(&CartesianVelocityController::setVelCallback, this, _1));

  /* ADDING INITIALIZATION OF PINOCCHIO */

  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(), pinocchio::JointModelFreeFlyer(), model_);

  RCLCPP_INFO(get_node()->get_logger(), "Model has been built, it has %d joints", model_.njoints);


  /*for (size_t i = 0; i < model_.frames.size(); ++i) {
    const auto & frame = model_.frames[i];
    std::cout << "Frame " << i << ": " << frame.name
              << ", type: " << frame.type << std::endl;
  }*/


  std::vector<pinocchio::JointIndex> joints_to_lock;
  /*VMO: In this point we make a list of joints to remove from the model (in tiago case wheel joints and end effector joints)
   These are actually the joints that we don't passs to the controller*/
  for (auto & name : model_.names) {
    if (name != "universe" && name != "root_joint" &&
      std::find(
        joint_names_.begin(), joint_names_.end(),
        name) == joint_names_.end())
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
  Eigen::VectorXd kp = params_.posture_gain * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6);
  Eigen::VectorXd kd = 2.0 * kp.cwiseSqrt();
  task_joint_posture_->Kp(kp);
  task_joint_posture_->Kd(kd);
  int posture_priority = 1;  // 0 constraint, 1 cost function
  double transition_time = 0.0;
  double posture_weight = 1e-2;

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
  double v_scaling = params_.velocity_scaling;
  Eigen::VectorXd v_max = v_scaling * model_.velocityLimit.tail(model_.nv - 6);
  Eigen::VectorXd v_min = -v_scaling * model_.velocityLimit.tail(model_.nv - 6);
  task_joint_bounds_->setVelocityBounds(v_min, v_max);

  for (int i = 0; i < v_min.size(); i++) {
    RCLCPP_INFO(
      get_node()->get_logger(), "position: %f", v_max[i]
    );
  }

  //formulation_->addMotionTask(*task_joint_bounds_, bounds_weight, bounds_priority, transition_time);

  // End effector tasks, one for each end effector in the config
  for (auto ee : ee_names_) {

    task_ee_.push_back(
      new
      tsid::tasks::TaskCartesianVelocity(
        "task-ee" + ee, *robot_wrapper_, ee, dt_.seconds()));

    auto gain = params_.cartesian_vel_gain.ee_names_map.at(ee);
    Eigen::VectorXd kp_gain = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd kd_gain = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ki_gain = Eigen::VectorXd::Zero(6);
    kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch, gain.kp_yaw;
    kd_gain << gain.kd_x, gain.kd_y, gain.kd_z, gain.kd_roll, gain.kd_pitch, gain.kd_yaw;
    ki_gain << gain.ki_x, gain.ki_y, gain.ki_z, gain.ki_roll, gain.ki_pitch, gain.ki_yaw;
    task_ee_[ee_id_[ee]]->Kp(kp_gain);
    task_ee_[ee_id_[ee]]->Kd(kd_gain);
    task_ee_[ee_id_[ee]]->Ki(ki_gain);
    Eigen::VectorXd ee_mask = Eigen::VectorXd::Zero(6);
    ee_mask << 1, 1, 1, 1, 1, 1;
    task_ee_[ee_id_[ee]]->setMask(ee_mask);
    task_ee_[ee_id_[ee]]->useLocalFrame(false);

    double ee_weight = 1;
    int ee_priority = 1;
    formulation_->addMotionTask(*task_ee_[ee_id_[ee]], ee_weight, ee_priority, transition_time);

  }

  // Initializing solver
  solver_ = new tsid::solvers::SolverHQuadProgFast("qp solver");

  solver_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianVelocityController::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;

  for (const auto & joint: joint_names_) {
    for (int i = 0; i < 3; i++) {
      state_interfaces_config_names.push_back(state_interface_names_[jnt_id_.at(joint)][i]);
    }
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};


}


controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const
{

  std::vector<std::string> command_interfaces_config_names;
  for (const auto & joint : joint_command_names_) {
    const auto full_name = joint + "/position";
    command_interfaces_config_names.push_back(full_name);
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};


}


controller_interface::CallbackReturn CartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Check if all the command interfaces are in the actuator interface
  for (const auto & joint : joint_names_) {
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

  for (const auto & joint : joint_names_) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Joint %s position: %f",
      joint.c_str(), joint_state_interfaces_[jnt_id_[joint]][0].get().get_value());
  }

  // Taking initial position from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());


  for (const auto & joint : joint_names_) {
    q0.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][Interfaces::position].get().get_value();

    v0.tail(robot_wrapper_->nv() - 6)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][Interfaces::velocity].get().get_value();
  }

  for (int i = 0; i < v0.size(); i++) {
    RCLCPP_INFO(get_node()->get_logger(), "Initial velocity: %f", v0[i]);
  }
  formulation_->computeProblemData(0.0, q0, v0);
  RCLCPP_INFO(get_node()->get_logger(), "Initial position: %f", q0[0]);

  // Setting posture task reference as initial position
  traj_joint_posture_->setReference(q0.tail(robot_wrapper_->nq() - 7));
  RCLCPP_INFO(get_node()->get_logger(), "Initial position: %f", q0[0]);

  task_joint_posture_->setReference(traj_joint_posture_->computeNext());
  RCLCPP_INFO(get_node()->get_logger(), "Initial position: %f", q0[0]);


  // Setting initial reference for the end effector tasks
  for (auto ee : ee_names_) {
    H_ee_0_[ee_id_[ee]] =
      robot_wrapper_->framePosition(formulation_->data(), model_.getFrameId(ee));
    Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(6);

    traj_ee_.push_back(
      tsid::trajectories::TrajectoryEuclidianConstant(
        "traj_ee",
        vel_des));
    tsid::trajectories::TrajectorySample sample_vel_ee = traj_ee_[ee_id_[ee]].computeNext();
    task_ee_[ee_id_[ee]]->setReference(sample_vel_ee);


    RCLCPP_INFO(
      get_node()->get_logger(), " Initial position ee %s : %f %f %f", ee.c_str(),
      H_ee_0_[ee_id_[ee]].translation()[0], H_ee_0_[ee_id_[ee]].translation()[1],
      H_ee_0_[ee_id_[ee]].translation()[2]);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  // TODO: set 0.0 torque to stop it
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // Updating params if new ones are available
  updateParams();

  // Taking current state
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  q[6] = 1.0;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  for (const auto & joint : joint_names_) {
    q.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][Interfaces::position].get().get_value();
    v.tail(robot_wrapper_->nv() - 6)[model_.getJointId(joint) -
      2] = joint_state_interfaces_[jnt_id_[joint]][Interfaces::velocity].get().get_value();

  }

  pinocchio::SE3 oMi;
  auto m_frame_id = robot_wrapper_->model().getFrameId("arm_right_7_link");
  robot_wrapper_->framePosition(formulation_->data(), m_frame_id, oMi);

  Eigen::VectorXd m_p;
  m_p = Eigen::VectorXd::Zero(7);
  tsid::math::SE3ToXYZQUAT(oMi, m_p);
  pinocchio::Motion v_frame;

  v_frame.linear((oMi.translation() - pos_prev.translation()) / 0.01);


  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = oMi.translation()[0];
  current_pose.position.y = oMi.translation()[1];
  current_pose.position.z = oMi.translation()[2];
  current_pose.orientation.x = m_p[3];
  current_pose.orientation.y = m_p[4];
  current_pose.orientation.z = m_p[5];
  current_pose.orientation.w = m_p[6];

  publisher_curr_pos->publish(current_pose);

  pos_prev = oMi;
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
  for (const auto & joint : joint_command_names_) {
    command_interfaces_[jnt_command_id_[joint]].set_value(
      q_cmd[model_.getJointId(joint) - 2]);
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::updateParams()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Updating parameters");
    for (auto ee : ee_names_) {

      auto gain = params_.cartesian_gain.ee_names_map.at(ee);
      Eigen::VectorXd kp_gain = Eigen::VectorXd::Zero(6);
      kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch, gain.kp_yaw;
      task_ee_[ee_id_[ee]]->Kp(kp_gain);
      task_ee_[ee_id_[ee]]->Kd(2.0 * task_ee_[ee_id_[ee]]->Kp().cwiseSqrt());
    }

    task_joint_posture_->Kp(params_.posture_gain * Eigen::VectorXd::Ones(robot_wrapper_->nv() - 6));
    task_joint_posture_->Kd(2.0 * task_joint_posture_->Kp().cwiseSqrt());


  }

}

void CartesianVelocityController::setVelCallback(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "The vel command should have 6 elements");
    return;
  }
  std::cout << "setVelCallback" << std::endl;
  auto ee = params_.ee_names[0];
  Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(6);
  vel_des << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
  std::cout << "setVelCallback" << std::endl;

  tsid::trajectories::TrajectorySample sample_vel_ee = traj_ee_[ee_id_[ee]].computeNext();
  sample_vel_ee.setValue(vel_des);
  task_ee_[ee_id_[ee]]->setReference(sample_vel_ee);
  std::cout << "setVelCallback" << std::endl;
  RCLCPP_INFO(
    get_node()->get_logger(), "Desired velocity: %f %f %f %f %f %f", vel_des[0], vel_des[1],
    vel_des[2], vel_des[3], vel_des[4], vel_des[5]);
  std::cout << "setVelCallback" << std::endl;
}


}  // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::CartesianVelocityController,
  controller_interface::ControllerInterface)
