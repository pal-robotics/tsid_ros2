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

#include "tsid_controllers/tsid_position_control.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

TsidPositionControl::TsidPositionControl()
: controller_interface::ControllerInterface(),
  dt_(0, 0)
{
}

controller_interface::CallbackReturn TsidPositionControl::on_init()
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

controller_interface::CallbackReturn TsidPositionControl::on_configure(
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
    joint_command_names_.resize(params_.joint_command_names.size());
    for (int i = 0; i < params_.joint_command_names.size(); i++) {
      size_t start = params_.joint_command_names[i].find("/");
      if (start != std::string::npos) {
        auto joint = params_.joint_command_names[i].substr(start + 1);
        joint_command_names_[i] = joint;

      } else {
        joint_command_names_[i] = params_.joint_command_names[i];

      }
    }
  }

  if (params_.ee_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The end effector name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load bounding boxes from parameters
  for (const auto & ee_name : params_.ee_names) {
    BoundingBox box;

    const auto & cube = params_.manipulation_cube.ee_names_map.at(ee_name);

    box.x_min = cube.x_min;
    box.x_max = cube.x_max;
    box.y_min = cube.y_min;
    box.y_max = cube.y_max;
    box.z_min = cube.z_min;
    box.z_max = cube.z_max;

    bounding_boxes_[ee_name] = box;
  }

  // Create the state and command interfaces
  state_interfaces_.reserve(3 * joint_names_.size());
  command_interfaces_.reserve(joint_command_names_.size());
  joint_state_interfaces_.resize(joint_names_.size());
  state_interface_names_.resize(joint_names_.size());

  q_prev_ = Eigen::VectorXd::Zero(joint_names_.size());


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

  // Creating model in pinocchio
  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(), pinocchio::JointModelFreeFlyer(), model_);

  RCLCPP_INFO(get_node()->get_logger(), "Model has been built, it has %d joints", model_.njoints);

  std::vector<pinocchio::JointIndex> joints_to_lock;

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

  // Getting control period
  dt_ = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / this->get_update_rate()));
  RCLCPP_INFO(get_node()->get_logger(), "Control period: %f", dt_.seconds());

  // Initialization of the TSID
  robot_wrapper_ = new tsid::robots::RobotWrapper(
    model_,
    tsid::robots::RobotWrapper::RootJointType::FLOATING_BASE_SYSTEM, true);

  formulation_ = new tsid::InverseDynamicsFormulationAccForce("tsid", *robot_wrapper_, true);
  first_tsid_iter_ = true;

  DefaultPositionTasks();

  // Initializing solver
  solver_ = new tsid::solvers::SolverHQuadProgFast("qp solver");

  solver_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());


  publisher_curr_vel =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("tsid_cmd_vel", 10);

  publisher_curr_pos =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("tsid_cmd_pos", 10);

  publisher_curr_current =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("tsid_cmd_curr", 10);

  box_pub_ = get_node()->create_publisher<visualization_msgs::msg::Marker>("bounding_box", 10);

  pose_pub_ = get_node()->create_publisher<visualization_msgs::msg::Marker>("desired_pose", 10);

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration TsidPositionControl::state_interface_configuration()
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
TsidPositionControl::command_interface_configuration() const
{

  std::vector<std::string> command_interfaces_config_names;
  for (const auto & joint : params_.joint_command_names) {
    const auto full_name = joint + "/" + params_.interface_name;
    command_interfaces_config_names.push_back(full_name);
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}
controller_interface::CallbackReturn TsidPositionControl::on_activate(
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
      this->get_node()->get_logger(), "Joint %s position: %f",
      joint.c_str(), joint_state_interfaces_[jnt_id_[joint]][0].get().get_value());
  }

  first_tsid_iter_ = true;

  // Taking initial position from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();

  q0 = state.first;
  //v0 = state.second;

  q_prev_ = q0.tail(robot_wrapper_->nq() - 7);

  formulation_->computeProblemData(0.0, q0, v0);

  // Setting posture task reference as initial position
  traj_joint_posture_->setReference(q0.tail(robot_wrapper_->nq() - 7));
  task_joint_posture_->setReference(traj_joint_posture_->computeNext());


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TsidPositionControl::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  release_interfaces();
  for (auto joint : params_.joint_state_names) {
    joint_state_interfaces_[jnt_id_[joint]].clear();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> TsidPositionControl::getActualState() const
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_wrapper_->nq());
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  for (const auto & joint : joint_names_) {
    q.tail(robot_wrapper_->nq() - 7)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_.at(joint)][Interfaces::position].get().get_value();
    v.tail(robot_wrapper_->nv() - 6)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_.at(joint)][Interfaces::velocity].get().get_value();
  }

  return std::make_pair(q, v);
}

void TsidPositionControl::updateParams()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Updating parameters");

    v_scaling_ = params_.velocity_scaling;
    Eigen::VectorXd v_max =
      v_scaling_ * model_.velocityLimit.tail(model_.nv - 6);
    task_joint_bounds_->setVelocityBounds(v_max);

    //Taking gain for joint posture task
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);
    Eigen::VectorXd kd = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);

    for (auto joint : joint_command_names_) {
      auto gain = params_.joint_pos_gain.joint_command_names_map.at(joint);

      if (gain.kp < 0 || gain.kd < 0) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "The gains for joint position task must be positive");
        return;
      }

      kp[jnt_command_id_[joint]] = gain.kp;
      kd[jnt_command_id_[joint]] = gain.kd;
    }

    task_joint_posture_->Kp(kp);
    task_joint_posture_->Kd(kd);
    for (const auto & ee_name : params_.ee_names) {
      BoundingBox box;

      const auto & cube = params_.manipulation_cube.ee_names_map.at(ee_name);

      box.x_min = cube.x_min;
      box.x_max = cube.x_max;
      box.y_min = cube.y_min;
      box.y_max = cube.y_max;
      box.z_min = cube.z_min;
      box.z_max = cube.z_max;

      bounding_boxes_[ee_name] = box;
    }

  }
}

void TsidPositionControl::DefaultPositionTasks()
{

  // Joint Posture Task
  task_joint_posture_ =
    new tsid::tasks::TaskJointPosture("task-joint-posture", *robot_wrapper_);

  //Taking gain for joint posture task
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(robot_wrapper_->nv() - 6);

  for (auto joint : joint_command_names_) {
    auto gain = params_.joint_pos_gain.joint_command_names_map.at(joint);

    if (gain.kp < 0 || gain.kd < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The gains for joint position task must be positive");
      return;
    }

    kp[jnt_command_id_[joint]] = gain.kp;
    kd[jnt_command_id_[joint]] = gain.kd;
  }

  task_joint_posture_->Kp(kp);
  task_joint_posture_->Kd(kd);
  int posture_priority = 1;    // 0 constraint, 1 cost function
  double transition_time = 0.0;
  double posture_weight = 1e-3;

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot_wrapper_->nv());

  traj_joint_posture_ = new tsid::trajectories::TrajectoryEuclidianConstant(
    "traj_joint", q0.tail(robot_wrapper_->nv() - 6));
  tsid::trajectories::TrajectorySample sample_posture = traj_joint_posture_->computeNext();
  task_joint_posture_->setReference(sample_posture);
  formulation_->addMotionTask(
    *task_joint_posture_, posture_weight, posture_priority,
    transition_time);

  // Joint Bounds Task
  task_joint_bounds_ = new tsid::tasks::TaskJointPosVelAccBounds(
    "task-joint-bounds",
    *robot_wrapper_, dt_.seconds(), false);
    *robot_wrapper_, dt_.seconds(), false);
  task_joint_bounds_->setTimeStep(dt_.seconds());

  Eigen::VectorXd q_min = model_.lowerPositionLimit.tail(model_.nv - 6);
  Eigen::VectorXd q_max = model_.upperPositionLimit.tail(model_.nv - 6);

  task_joint_bounds_->setPositionBounds(q_min, q_max);

  int bounds_priority = 0;    // 0 constraint, 1 cost function
  double bounds_weight = 1;
  // Joint velocity bounds
  v_scaling_ = params_.velocity_scaling;
  Eigen::VectorXd v_max = v_scaling_ * model_.velocityLimit.tail(model_.nv - 6);
  Eigen::VectorXd v_min = -v_max;
  //Eigen::VectorXd a_max = params_.acc_lim * Eigen::VectorXd::Ones(model_.nv - 6);
  task_joint_bounds_->setVelocityBounds(v_max);
  //task_joint_bounds_->setAccelerationBounds(a_max);
  formulation_->addMotionTask(*task_joint_bounds_, bounds_weight, bounds_priority, transition_time);

}

void TsidPositionControl::compute_problem_and_set_command(
  const Eigen::VectorXd q,
  const Eigen::VectorXd v)
{
  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(model_.nq);
  q_[6] = 1;

  Eigen::VectorXd v_ = Eigen::VectorXd::Zero(model_.nv);
  v_.tail(model_.nq - 7) = (q.tail(model_.nq - 7) - q_prev_) / dt_.seconds();

  if (first_tsid_iter_) {
    RCLCPP_INFO(
      get_node()->get_logger(), "position_end %f",
      task_joint_posture_->getReference().getValue()[0]);
    q_int_ = q;
    v_int_ = Eigen::VectorXd::Zero(q_int_.size() - 1);
    first_tsid_iter_ = false;
  }

  q_.tail(model_.nq - 7) = q_int_.tail(model_.nq - 7);
  // Computing the problem data
  const tsid::solvers::HQPData solverData = formulation_->computeProblemData(0.0, q_, v_int_);
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd v_cmd, a;
  Eigen::VectorXd q_int;
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(model_.nv - 6);
  q_cmd = q.tail(model_.nq - 7);

  if (params_.interface_name == "position") {
    // Solving the problem
    const auto sol = solver_->solve(solverData);
    // Integrating acceleration to get velocity
    a = formulation_->getAccelerations(sol);
    v_cmd = v + a * 0.5 * dt_.seconds();


    // Integrating velocity to get position
    q_int_ = pinocchio::integrate(
      model_, q_int_, v_cmd * dt_.seconds());

    v_int_ = v_cmd;

    q_cmd = q_int_.tail(model_.nq - 7);

    // Setting the command to the joint command interfaces
    for (const auto & joint : joint_command_names_) {
      command_interfaces_[jnt_command_id_[joint]].set_value(
        q_cmd[model_.getJointId(joint) - 2]);
    }
  } else if (params_.interface_name == "velocity") {
    // Solving the problem
    const auto sol = solver_->solve(solverData);
    // Integrating acceleration to get velocity

    a = formulation_->getAccelerations(sol);
    v_cmd = (v_ + a * 0.5 * dt_.seconds()).tail(model_.nv - 6);

    // Setting the command to the joint command interfaces
    for (const auto & joint : joint_command_names_) {
      command_interfaces_[jnt_command_id_[joint]].set_value(
        v_cmd[model_.getJointId(joint) - 2]);
    }
  } else if (params_.interface_name == "effort") {
    // Solving the problem
    const auto sol = solver_->solve(solverData);

    a = formulation_->getAccelerations(sol);
    v_cmd = v + a * 0.5 * dt_.seconds();

    tau_cmd = formulation_->getActuatorForces(sol);

    // Transform tau_cmd in effort command
    for (const auto & joint : joint_command_names_) {
      const auto joint_params = params_.parameters_sin.joint_command_names_map.at(joint);

      for (int i = 0; i < tau_cmd.size(); ++i) {
        tau_cmd[i] /= (joint_params.motor_torque_constant * joint_params.reduction_ratio);
      }
      command_interfaces_[jnt_command_id_[joint]].set_value(tau_cmd[model_.getJointId(joint) - 2]);

    }
  }
  q_prev_ = q.tail(model_.nq - 7);

  std_msgs::msg::Float64MultiArray pub;

  for (int i = 0; i < v_cmd.size(); i++) {
    pub.data.push_back(v_cmd[i]);
  }

  publisher_curr_vel->publish(pub);
  std_msgs::msg::Float64MultiArray pub_pos;

  q_cmd = task_joint_posture_->getReference().getValue();
  for (int i = 0; i < q_cmd.size(); i++) {
    pub_pos.data.push_back(q_cmd[i]);
  }

  publisher_curr_pos->publish(pub_pos);

  std_msgs::msg::Float64MultiArray pub_curr;

  for (int i = 0; i < tau_cmd.size(); i++) {
    pub_curr.data.push_back(tau_cmd[i]);
  }

  publisher_curr_current->publish(pub_curr);
  q_prev_ = q.tail(model_.nq - 7);
}

void TsidPositionControl::visualizeBoundingBox(const std::string & effector_name)
{
  // check if the effector_name is a valid end-effector
  if (bounding_boxes_.find(effector_name) == bounding_boxes_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unknown end-effector: %s", effector_name.c_str());
    return;
  }

  for (const auto & bb : bounding_boxes_) {
    if (effector_name == bb.first) {  // Match the effector name with the bounding box key
      visualization_msgs::msg::Marker bounding_box_marker;
      bounding_box_marker.header.frame_id = "base_footprint";
      bounding_box_marker.ns = "bounding_box";
      bounding_box_marker.header.stamp = rclcpp::Clock().now();

      bounding_box_marker.id = std::hash<std::string>{}(bb.first);

      bounding_box_marker.type = visualization_msgs::msg::Marker::CUBE;
      bounding_box_marker.action = visualization_msgs::msg::Marker::ADD;

      bounding_box_marker.scale.x = bb.second.x_max - bb.second.x_min;  // x range
      bounding_box_marker.scale.y = bb.second.y_max - bb.second.y_min;  // y range
      bounding_box_marker.scale.z = bb.second.z_max - bb.second.z_min;  // z range

      bounding_box_marker.pose.position.x = (bb.second.x_max + bb.second.x_min) / 2.0;
      bounding_box_marker.pose.position.y = (bb.second.y_max + bb.second.y_min) / 2.0;
      bounding_box_marker.pose.position.z = (bb.second.z_max + bb.second.z_min) / 2.0;

      bounding_box_marker.pose.orientation.w = 1.0;

      bounding_box_marker.color.r = 0.0f;
      bounding_box_marker.color.g = 1.0f;
      bounding_box_marker.color.b = 0.0f;
      bounding_box_marker.color.a = 0.1f;

      box_pub_->publish(bounding_box_marker);
    }
  }
}

void TsidPositionControl::visualizePose(
  const Eigen::Vector3d & pose)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_footprint";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "desired_pose";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = pose.x();
  marker.pose.position.y = pose.y();
  marker.pose.position.z = pose.z();

  marker.scale.x = 0.1;        // Adjust size as needed
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  pose_pub_->publish(marker);
}

bool TsidPositionControl::isPoseInsideBoundingBox(
  const Eigen::Vector3d & pose,
  const std::string & effector_name)
{
  auto it = bounding_boxes_.find(effector_name);
  if (it == bounding_boxes_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unknown end-effector: %s", effector_name.c_str());
    return false;
  }

  const BoundingBox & box = it->second;

  return pose.x() >= box.x_min && pose.x() <= box.x_max &&
         pose.y() >= box.y_min && pose.y() <= box.y_max &&
         pose.z() >= box.z_min && pose.z() <= box.z_max;
}

}  // namespace dynamic_tsid_controller
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::TsidPositionControl,
  controller_interface::ControllerInterface)
