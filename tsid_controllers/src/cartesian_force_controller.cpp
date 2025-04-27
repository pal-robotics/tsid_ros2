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

#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/fwd.hpp>


#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "tsid_controllers/cartesian_force_controller.hpp"

using namespace controller_interface;

namespace tsid_controllers
{
using std::placeholders::_1;

const bool CartesianForceController::debug_backdoor_ = true;

CartesianForceController::CartesianForceController()
: controller_interface::ControllerInterface(),
  dt_(0, 0)
{
}

controller_interface::CallbackReturn CartesianForceController::on_init()
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

// TODO
controller_interface::CallbackReturn CartesianForceController::on_configure(
  const rclcpp_lifecycle::State & /*prev_state*/)
{
  // Check if parameters were taken correctly
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Update the parameters.
  params_ = param_listener_->get_params();

  // Check if the actuator names are not empty
  if (params_.joint_state_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The joint names cannot be empty");
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

  if (params_.ee_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The end effector name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  local_frame_ = params_.local_frame;
  if (local_frame_) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "The reference is considered as expressed in the end effector frame");
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(), "The reference is considered as expressed in the base frame");
  }

  // Setup the force torque sensor measurements:
  ft_sensors_.clear();
  force_filters_.clear();
  for (const auto & ee_name : params_.ee_names) {
    ft_sensors_.emplace(
      ee_name,
      semantic_components::ForceTorqueSensor(
        params_.ft_sensor.ee_names_map.at(ee_name).name));
    force_filters_.emplace(
      ee_name, ForceDerivativeFilter(
        params_.ft_sensor.ee_names_map.at(ee_name).filter_cutoff));
    ft_sensor_interfaces_.emplace(
      ee_name, ft_sensors_.at(ee_name).get_state_interface_names());
  }

  if (debug_backdoor_) {
    // creating publisher for current pose ee
    publisher_curr_pos_ =
      get_node()->create_publisher<geometry_msgs::msg::Pose>("current_position", 10);
  }

  // Creating a map between index and end-effector ordered based on pinocchio
  ee_names_ = params_.ee_names; // backup the ee_names.

  // Create the state and command interfaces
  state_interfaces_.reserve(3 * joint_names_.size());
  command_interfaces_.reserve(joint_command_names_.size());

  // Allocate the memory for the interfaces.
  joint_state_interfaces_[0].reserve(joint_names_.size());
  joint_state_interfaces_[1].reserve(joint_names_.size());
  joint_state_interfaces_[2].reserve(joint_names_.size());
  joint_command_interfaces_.reserve(joint_names_.size());
  joint_state_interfaces_[0].clear();
  joint_state_interfaces_[1].clear();
  joint_state_interfaces_[2].clear();
  joint_command_interfaces_.clear();

  // Creating a map between index and joint
  jnt_command_id_.clear();
  jnt_id_.clear();
  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    jnt_id_.emplace(joint_names_[j], j);
    jnt_command_id_.emplace(joint_command_names_[j], j);
  }

  // Defines the interface names.
  state_interface_names_[0].clear();
  state_interface_names_[1].clear();
  state_interface_names_[2].clear();
  for (std::size_t i = 0; i < interface_types_.size(); ++i) {
    for (std::size_t j = 0; j < joint_names_.size(); ++j) {
      // Create the state interfaces
      state_interface_names_[i].push_back(
        joint_names_[j] + "/" + interface_types_[i]);
    }
  }
  // Initialize the command interface names.
  command_interfaces_names_.clear();
  for (const auto & joint : joint_command_names_) {
    command_interfaces_names_.push_back(
      joint + "_inertia_shaping_controller/" + joint + "/effort");
  }

  // 6d force (wrench) reference callback
  ee_cmd_sub_ =
    get_node()->create_subscription<tsid_controller_msgs::msg::EeWrench>(
    "cartesian_force_controller/wrench_cmd", 1,
    std::bind(&CartesianForceController::setWrenchCallback, this, _1));

  // Creating model in pinocchio
  if (params_.root_joint_type == "JointModelFreeFlyer") {
    pinocchio::urdf::buildModelFromXML(
      this->get_robot_description(), pinocchio::JointModelFreeFlyer(), model_);
  } else {
    pinocchio::urdf::buildModelFromXML(
      this->get_robot_description(), model_);
  }
  /* Removing the unused joints from the model*/
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (auto & name : model_.names) {
    if (name != "universe" &&
      name != "root_joint" &&
      std::find(joint_names_.begin(), joint_names_.end(), name) ==
      joint_names_.end())
    {
      joints_to_lock.push_back(model_.getJointId(name));
      RCLCPP_INFO(get_node()->get_logger(), "Lock joint %s: ", name.c_str());
    }
  }
  model_ = buildReducedModel(
    model_, joints_to_lock, pinocchio::neutral(model_));

  sensor_data_.q = Eigen::VectorXd::Zero(model_.nq);
  sensor_data_.v = Eigen::VectorXd::Zero(model_.nv);
  sensor_data_.fts.clear();
  for (auto ee : params_.ee_names) {
    sensor_data_.fts[ee] = Eigen::VectorXd::Zero(6);
  }

  if (debug_backdoor_) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Model has been built, it has %d joints", model_.njoints);
    for (auto joint : model_.names) {
      RCLCPP_INFO(get_node()->get_logger(), "Joint name: %s", joint.c_str());
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Model: " << model_);
  }

  // taking gain from parameters
  auto gain = params_.cartesian_force_gain.ee_names_map.at(ee_names_[0]);
  kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch, gain.kp_yaw;
  kd_gain << gain.kd_x, gain.kd_y, gain.kd_z, gain.kd_roll, gain.kd_pitch, gain.kd_yaw;
  ki_gain << gain.ki_x, gain.ki_y, gain.ki_z, gain.ki_roll, gain.ki_pitch, gain.ki_yaw;

  /*
   * VMO: if we need to check the mass of the model to verify that it is more
   * or less realistic. IPE: 18.21225 arms+torso ~= what we expected
   */
  RCLCPP_INFO(
    get_node()->get_logger(), "Total mass according to the model %f",
    pinocchio::computeTotalMass(model_));

  // Getting control period
  dt_ = rclcpp::Duration(
    std::chrono::duration<double, std::milli>(
      1e3 / this->get_update_rate()));
  RCLCPP_INFO(get_node()->get_logger(), "Control period: %f", dt_.seconds());

  data_ = pinocchio::Data(model_);
  size_t count_ee = 0;

  for (const auto & frame : model_.frames) {
    for (const auto & ee_name : params_.ee_names) {
      if (frame.name == ee_name) {
        ee_id_[frame.name] = count_ee++;

        RCLCPP_INFO(
          get_node()->get_logger(), "End effector %s found in the model, with index %u",
          frame.name.c_str(), ee_id_[frame.name]);
      }
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianForceController::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;

  for (int i = 0; i < 3; i++) {
    for (const auto & joint: joint_names_) {
      state_interfaces_config_names.push_back(
        state_interface_names_[i][jnt_id_.at(joint)]);
    }
  }
  for (const auto & ee : ee_names_) {
    for (const auto & name : ft_sensor_interfaces_.at(ee)) {
      state_interfaces_config_names.push_back(name);
    }
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}


controller_interface::InterfaceConfiguration
CartesianForceController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_names_};
}


controller_interface::CallbackReturn CartesianForceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating controller");
  // Acquire the interface to the states.
  for (size_t i = 0; i < 3; ++i) {
    if (
      !controller_interface::get_ordered_interfaces(
        state_interfaces_, state_interface_names_[i],
        "", joint_state_interfaces_[i]))
    {
      RCLCPP_ERROR(
        this->get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
        state_interface_names_[i].size(),
        joint_state_interfaces_[i].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  // Acquire interfaces to the command in effort.
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interfaces_names_,
      "", joint_command_interfaces_))
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interfaces_names_.size(),
      joint_command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  // Acquire interfaces to the FT sensors
  for (const auto & ee : ee_names_) {
    if (!ft_sensors_.at(ee).assign_loaned_state_interfaces(state_interfaces_)) {
      RCLCPP_ERROR_STREAM(
        this->get_node()->get_logger(),
        "Expected the state interfaces for the " << ee << " FT sensor ["
                                                 << params_.ft_sensor.ee_names_map.at(
          ee).name << "]. Something went wrong.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (debug_backdoor_) {
    for (const auto & joint : joint_names_) {
      RCLCPP_INFO(
        get_node()->get_logger(), "Joint %s position: %f",
        joint.c_str(), joint_state_interfaces_[0][jnt_id_[joint]].get().get_value());
    }
  }

  // Read the current measurement.
  read_sensor_data();

  // Taking initial position from the joint state interfaces
  RCLCPP_INFO_STREAM(
    get_node()->get_logger(),
    "Initial position: " << sensor_data_.q.transpose() << std::endl <<
      "Initial velocity: " << sensor_data_.v.transpose() << std::endl);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianForceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Release all interfaces.
  for (auto joint : params_.joint_state_names) {
    joint_state_interfaces_[0].clear();
    joint_state_interfaces_[1].clear();
    joint_state_interfaces_[2].clear();
    joint_command_interfaces_.clear();
  }
  for (const auto & ee : ee_names_) {
    ft_sensors_.at(ee).release_interfaces();
  }
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type CartesianForceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Updating params if new ones are available
  updateParams();

  // Read the current measurement.
  read_sensor_data();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Force-Torque Sensor Data: Forces - [%f, %f, %f], Torques - [%f, %f, %f]",
    sensor_data_.fts[ee_names_[0]].toVector()[0],
    sensor_data_.fts[ee_names_[0]].toVector()[1],
    sensor_data_.fts[ee_names_[0]].toVector()[2],
    sensor_data_.fts[ee_names_[0]].toVector()[3],
    sensor_data_.fts[ee_names_[0]].toVector()[4],
    sensor_data_.fts[ee_names_[0]].toVector()[5]);

  // Set external force from ft sensor
  /*for (size_t i = 0; i < ee_names_.size(); ++i) {
    tsid::trajectories::TrajectorySample f_ext;
    f_ext.setValue(sensor_data_.fts[ee_names_[i]].toVector());
    f_ext.setDerivative(sensor_data_.dfts[ee_names_[i]].toVector());
    f_ext.setSecondDerivative(Eigen::VectorXd::Zero(6));
    task_force_ee_[i].setExternalForce(f_ext);
  }*/

  // Taking current state
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  if (params_.root_joint_type == "JointModelFreeFlyer") {
    /// @todo acquire the base state from somewhere.
  }

  for (const auto & joint : joint_names_) {
    auto jid = model_.getJointId(joint) - 2;
    q.tail(model_.nq - 7)[jid] =
      joint_state_interfaces_[0][jnt_id_[joint]].get().get_value();
    v.tail(model_.nv - 6)[jid] =
      joint_state_interfaces_[1][jnt_id_[joint]].get().get_value();
  }

  // Computing the current time, and setting the origin if it is not set
  if (std::isnan(controller_time_origin_)) {
    controller_time_origin_ = get_node()->get_clock()->now().seconds();
    controller_current_time_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Controller time origin: %f", controller_time_origin_);
  } else {
    controller_current_time_ = get_node()->get_clock()->now().seconds() - controller_time_origin_;
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    500,
    "Controller time: %f", controller_current_time_);

  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(model_.nq - 7);

  //Eigen::MatrixXd J(6, model_.nv); // 6xN for spatial velocity

  const pinocchio::FrameIndex ee_id = model_.getFrameId(ee_names_[0]);

  //pinocchio::computeFrameJacobian(model_, data_, q, ee_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // 3. Desired force at end-effector (spatial force, 6D vector)
  /* Eigen::VectorXd F_ee_des(6);
 // Fill this with desired force + torque

 // 4. Compute desired joint torque
   Eigen::VectorXd tau_des = J.transpose() * F_ee_des;

 // 5. Compute desired joint acceleration
   Eigen::VectorXd qdd_des = M.ldlt().solve(tau_des - data.nle);
 */
  // Update kinematics
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::computeJointJacobians(model_, data_, q);   //
  pinocchio::updateFramePlacements(model_, data_);
  // Find frame position
  const pinocchio::SE3 & oMf = data_.oMf[ee_id];        // Transform from base to EE


// Compute M(q)
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd M = data_.M;

// Compute C(q,v)*v + g(q)
  Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, data_, q, v);

// Compute external wrench contribution
  pinocchio::Force wrench_vec(sensor_data_.fts[ee_names_[0]].toVector());
  pinocchio::Force wrench_vec_der(sensor_data_.dfts[ee_names_[0]].toVector());
  pinocchio::Force wrench_des_force(desired_wrench_);


  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
  getFrameJacobian(model_, data_, ee_id, pinocchio::LOCAL, J);
  //J = oMf.toActionMatrix() * J;
  //pinocchio::Motion wrench_ext = se3.act(wrench_vec);

  Eigen::VectorXd wrench_vector = wrench_vec.toVector();       // size 6
  Eigen::VectorXd wrench_des_vec = wrench_des_force.toVector();       // size 6
  //Eigen::VectorXd wrench_der_vec = wrench_base_der.toVector(); // size 6

// Apply gains element-wise — both gain vectors must also be VectorXd of size 6
  Eigen::VectorXd pid_output = wrench_des_vec.array() + kp_gain.array() *
    (wrench_des_vec.array() - wrench_vector.array());

// Project into joint torques
  Eigen::VectorXd tau_ext = J.transpose() * pid_output;


  // Stream tau ext
  if (debug_backdoor_) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "External torque: %f %f %f %f %f %f %f %f %f %f %f %f %f",
      tau_ext[0], tau_ext[1], tau_ext[2], tau_ext[3], tau_ext[4], tau_ext[5],
      tau_ext[6], tau_ext[7], tau_ext[8], tau_ext[9], tau_ext[10], tau_ext[11], tau_ext[12]
    );
  }


// Final command
  Eigen::VectorXd tau = nle + tau_ext;

  tau_cmd = tau.tail(model_.nv - 6);

  // Compute the gravity compensation

  // Setting the command to the joint command interfaces (send gravity compensation)
  for (const auto & joint : joint_command_names_) {
    // -1 because the first joint is the universe joint.
    size_t offset = -1;
    if (model_.existJointName("root_joint")) {
      // another -1 for the root_joint
      offset -= 1;
    }
    bool ret =
      joint_command_interfaces_[jnt_command_id_[joint]].get().set_value(
      tau_cmd[model_.getJointId(joint) + offset]);

    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      500,
      "Torque command for joint %s: %f",
      joint.c_str(), tau_cmd[model_.getJointId(joint) + offset]);
    // gravity compensation
    //  bool ret =
    //   joint_command_interfaces_[jnt_command_id_[joint]].get().set_value(
    //   g.tail(robot_wrapper_->na())[model_.getJointId(joint) + offset]);
    if (!ret) {
      RCLCPP_ERROR_STREAM(
        get_node()->get_logger(), "Can not set the torque command at joint " << joint);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

void CartesianForceController::updateParams()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();

    // Update gain pid
    auto gain = params_.cartesian_force_gain.ee_names_map.at(ee_names_[0]);
    kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch, gain.kp_yaw;
    kd_gain << gain.kd_x, gain.kd_y, gain.kd_z, gain.kd_roll, gain.kd_pitch, gain.kd_yaw;
  }
}

void CartesianForceController::setWrenchCallback(
  tsid_controller_msgs::msg::EeWrench::ConstSharedPtr msg)
{
  if (get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    for (size_t i = 0; i < msg->ee_name.size(); ++i) {
      if (std::find(ee_names_.begin(), ee_names_.end(), msg->ee_name[i]) == ee_names_.end()) {
        RCLCPP_WARN(
          get_node()->get_logger(), "End effector %s not found in the list of end effectors, this command will be ignored",
          msg->ee_name[i].c_str());
      } else {

        const auto & ee = msg->ee_name[i];

        RCLCPP_INFO(
          get_node()->get_logger(), "Setting desired wrench for end effector %s", ee.c_str());


        // Taking desired wrench from the message
        desired_wrench_ << msg->desired_wrench[i].force.x,
          msg->desired_wrench[i].force.y,
          msg->desired_wrench[i].force.z, msg->desired_wrench[i].torque.x,
          msg->desired_wrench[i].torque.y,
          msg->desired_wrench[i].torque.z;

        RCLCPP_INFO(
          get_node()->get_logger(), "Desired wrench for end effector %s: %f %f %f %f %f %f",
          ee.c_str(), desired_wrench_[0], desired_wrench_[1],
          desired_wrench_[2], desired_wrench_[3], desired_wrench_[4],
          desired_wrench_[5]);

/* if (local_frame_) {
          // Transforming the wrench to the base frame
          // TODO: check this transformation!!!!
          RCLCPP_INFO(
            get_node()->get_logger(), "Transforming the wrench to the base frame");
          desired_wrench = h_ee_.act(desired_wrench);
          RCLCPP_INFO(
            get_node()->get_logger(), "Desired wrench for end effector in base frame %s: %f %f %f %f %f %f",
            ee.c_str(), desired_wrench.linear().x(), desired_wrench.linear().y(),
            desired_wrench.linear().z(), desired_wrench.angular().x(),
            desired_wrench.angular().y(), desired_wrench.angular().z());
        }
        */

        RCLCPP_INFO(
          get_node()->get_logger(), "Reference set for end effector %s", ee.c_str());

      }
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Controller is not active, the command will be ignored");
  }
}

void CartesianForceController::read_sensor_data()
{
  // Read the joint states
  if (params_.root_joint_type == "JointModelFreeFlyer") {
    /// @todo acquire the base state from somewhere.
  }

  for (size_t i = 0; i < joint_names_.size(); i++) {
    sensor_data_.q.tail(model_.nq - 7)(i) =
      joint_state_interfaces_[0][i].get().get_value();
    sensor_data_.v.tail(model_.nv - 6)(i) =
      joint_state_interfaces_[1][i].get().get_value();
  }

  // Read the force torque sensors
  for (const auto & ee : ee_names_) {
    auto forces = ft_sensors_.at(ee).get_forces();
    auto torques = ft_sensors_.at(ee).get_torques();
    force_filters_.at(ee).filter(forces, torques, dt_.seconds());
    sensor_data_.fts[ee].toVector() = force_filters_.at(ee).getWrench();
    sensor_data_.dfts[ee].toVector() =
      force_filters_.at(ee).getWrenchDerivative();
  }

}


}  // namespace dynamic_tsid_controller

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::CartesianForceController,
  controller_interface::ControllerInterface)
