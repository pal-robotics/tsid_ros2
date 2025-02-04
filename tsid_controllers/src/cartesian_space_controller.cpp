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

#include "tsid_controllers/cartesian_space_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

using namespace controller_interface;
namespace tsid_controllers
{
using std::placeholders::_1;

CartesianSpaceController::CartesianSpaceController()
: tsid_controllers::TsidPositionControl() {}

controller_interface::CallbackReturn CartesianSpaceController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  auto result = TsidPositionControl::on_configure(prev_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  if (getParams().ee_names.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "The end effector name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }


  local_frame_ = getParams().local_frame;

  if (local_frame_) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "The reference is considered as expressed in the end effector frame");
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "The reference is considered as expressed in the base frame");
  }

  // Storing names of desired end effector
  ee_names_.resize(getParams().ee_names.size());
  desired_pose_.resize(getParams().ee_names.size());
  H_ee_0_.resize(getParams().ee_names.size());

  for (size_t i = 0; i < getParams().ee_names.size(); i++) {
    ee_id_.insert(std::make_pair(getParams().ee_names[i], i));
    ee_names_[i] = getParams().ee_names[i];
  }
  // Creating the publisher for the current position
  publisher_curr_pos = get_node()->create_publisher<geometry_msgs::msg::Pose>(
    "current_position", 10);

  // Pose reference callback
  ee_cmd_sub_ =
    get_node()->create_subscription<tsid_controller_msgs::msg::EePos>(
    "cartesian_space_controller/pose_cmd", 1,
    std::bind(&CartesianSpaceController::setPoseCallback, this, _1));
  // print getParams().ee_names.size()

  // End effector tasks, one for each end effector in the config
  for (const auto & ee : ee_names_) {
    task_ee_.push_back(
      new tsid::tasks::TaskSE3Equality(
        "task-ee" + ee, *TsidPositionControl::robot_wrapper_, ee));

    auto gain = getParams().cartesian_gain.ee_names_map.at(ee);
    Eigen::VectorXd kp_gain = Eigen::VectorXd::Zero(6);
    kp_gain << gain.kp_x, gain.kp_y, gain.kp_z, gain.kp_roll, gain.kp_pitch,
      gain.kp_yaw;
    task_ee_[ee_id_[ee]]->Kp(kp_gain);

    task_ee_[ee_id_[ee]]->Kd(2.0 * task_ee_[ee_id_[ee]]->Kp().cwiseSqrt());

    Eigen::VectorXd ee_mask = Eigen::VectorXd::Zero(6);
    ee_mask << 1, 1, 1, 1, 1, 1;
    task_ee_[ee_id_[ee]]->setMask(ee_mask);
    task_ee_[ee_id_[ee]]->useLocalFrame(false);
    double ee_weight = 1;
    int ee_priority = 1;
    double transition_time = 0.0;

    TsidPositionControl::formulation_->addMotionTask(
      *task_ee_[ee_id_[ee]], ee_weight, ee_priority, transition_time);
  }

  v_max = params_.ee_vmax;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianSpaceController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto result = TsidPositionControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  // Setting initial reference for the end effector tasks
  for (auto ee : ee_names_) {
    H_ee_0_[ee_id_[ee]] = robot_wrapper_->framePosition(
      TsidPositionControl::formulation_->data(),
      TsidPositionControl::model_.getFrameId(ee));
    traj_ee_.push_back(
      tsid::trajectories::TrajectorySE3Constant(
        "traj_ee", H_ee_0_[ee_id_[ee]]));
    tsid::trajectories::TrajectorySample sample_posture_ee =
      traj_ee_[ee_id_[ee]].computeNext();
    task_ee_[ee_id_[ee]]->setReference(sample_posture_ee);

    position_start_ = H_ee_0_[ee_id_[ee]].translation();
    position_end_ = H_ee_0_[ee_id_[ee]].translation();
    quat_init_ = H_ee_0_[ee_id_[ee]].rotation();
    quat_des_ = H_ee_0_[ee_id_[ee]].rotation();
    rot_des_ = H_ee_0_[ee_id_[ee]].rotation();
    vel_curr_ = Eigen::Vector3d::Zero();
    compute_trajectory_params();


    RCLCPP_INFO(
      get_node()->get_logger(), " Initial position ee %s : %f %f %f",
      ee.c_str(), H_ee_0_[ee_id_[ee]].translation()[0],
      H_ee_0_[ee_id_[ee]].translation()[1],
      H_ee_0_[ee_id_[ee]].translation()[2]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
CartesianSpaceController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  t_curr_ = t_curr_ + dt_.seconds();
  TsidPositionControl::updateParams();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;

  // if (iteration % 4 == 0) {
  interpolate(t_curr_);

  Eigen::VectorXd ref = Eigen::VectorXd::Zero(12);

  pinocchio::SE3 se3(rot_des_, position_curr_);
  tsid::math::SE3ToVector(se3, ref);

  Eigen::VectorXd ref_vel = Eigen::VectorXd::Zero(6);
  ref_vel.head<3>() = vel_curr_;
  tsid::trajectories::TrajectorySample sample_posture_ee =
    traj_ee_[ee_id_[ee_names_[0]]].computeNext();
  sample_posture_ee.setValue(ref);
  sample_posture_ee.setDerivative(ref_vel);
  task_ee_[ee_id_[ee_names_[0]]]->setReference(sample_posture_ee);
  // }

  compute_problem_and_set_command(state.first, state.second);   // q and v
  auto h_ee_ = TsidPositionControl::robot_wrapper_->framePosition(
    TsidPositionControl::formulation_->data(),
    TsidPositionControl::model_.getFrameId(ee_names_[0]));

  geometry_msgs::msg::Pose current_pose;
  Eigen::Quaterniond quat_curr(h_ee_.rotation());
  current_pose.position.x = h_ee_.translation()[0];
  current_pose.position.y = h_ee_.translation()[1];
  current_pose.position.z = h_ee_.translation()[2];
  current_pose.orientation.x = quat_curr.x();
  current_pose.orientation.y = quat_curr.y();
  current_pose.orientation.z = quat_curr.z();
  current_pose.orientation.w = quat_curr.w();

  publisher_curr_pos->publish(current_pose);

  iteration++;

  return controller_interface::return_type::OK;
}

void CartesianSpaceController::setPoseCallback(
  tsid_controller_msgs::msg::EePos::ConstSharedPtr msg)
{
  if (get_node()->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    for (int i = 0; i < msg->ee_name.size(); i++) {
      if (std::find(ee_names_.begin(), ee_names_.end(), msg->ee_name[i]) ==
        ee_names_.end())
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "End effector %s not found in the list of end effectors, "
          "this command will be ignored",
          msg->ee_name[i].c_str());
      } else {
        auto ee = msg->ee_name[i];

        auto h_ee_ = TsidPositionControl::robot_wrapper_->framePosition(
          TsidPositionControl::formulation_->data(),
          TsidPositionControl::model_.getFrameId(ee_names_[i]));
        Eigen::VectorXd ref = Eigen::VectorXd::Zero(12);

        if (local_frame_) {
          // Taking desired position from the message
          pinocchio::Motion desired_pose;
          desired_pose.setZero();
          desired_pose.linear() << msg->desired_pose[i].position.x,
            msg->desired_pose[i].position.y, msg->desired_pose[i].position.z;
          desired_pose = h_ee_.toActionMatrix() * desired_pose.toVector();

          // Adding displacement to the desired pose
          desired_pose_[ee_id_[ee]]
            << h_ee_.translation()[0] + desired_pose.linear()[0],
            h_ee_.translation()[1] + desired_pose.linear()[1],
            h_ee_.translation()[2] + desired_pose.linear()[2];

          // Setting the orientation desired
          Eigen::Quaterniond quat(msg->desired_pose[i].orientation.w,
            msg->desired_pose[i].orientation.x,
            msg->desired_pose[i].orientation.y,
            msg->desired_pose[i].orientation.z);

          Eigen::Matrix3d rot_des = h_ee_.rotation() * quat.toRotationMatrix();

          pinocchio::SE3 se3(rot_des, desired_pose_[ee_id_[ee]]);
          tsid::math::SE3ToVector(se3, ref);

          rot_des_ = h_ee_.rotation() * quat.toRotationMatrix();
        } else {
          // Taking desired position from the message
          pinocchio::Motion desired_pose;
          desired_pose.setZero();
          desired_pose.linear() << msg->desired_pose[i].position.x,
            msg->desired_pose[i].position.y, msg->desired_pose[i].position.z;

          // Adding displacement to the desired pose
          desired_pose_[ee_id_[ee]]
            << h_ee_.translation()[0] + desired_pose.linear()[0],
            h_ee_.translation()[1] + desired_pose.linear()[1],
            h_ee_.translation()[2] + desired_pose.linear()[2];

          // Setting the orientation desired
          Eigen::Quaterniond quat(msg->desired_pose[i].orientation.w,
            msg->desired_pose[i].orientation.x,
            msg->desired_pose[i].orientation.y,
            msg->desired_pose[i].orientation.z);

          // pinocchio::SE3 se3(rot_des_, desired_pose_[ee_id_[ee]]);
          // tsid::math::SE3ToVector(se3, ref);
        }

        tsid::trajectories::TrajectorySample sample_posture_ee =
          traj_ee_[ee_id_[ee]].computeNext();
        sample_posture_ee.setValue(ref);
        task_ee_[ee_id_[ee]]->setReference(sample_posture_ee);

        quat_des_ = rot_des_;
        position_end_ = desired_pose_[ee_id_[ee]];

        t_curr_ = 0.0;
        position_start_ = h_ee_.translation();
        std::cout << "position_start_ " << position_start_ << std::endl;
        std::cout << "position_end_ " << position_end_ << std::endl;
        std::cout << "quat_des_ " << quat_des_.coeffs() << std::endl;
        position_end_ = desired_pose_[ee_id_[ee]];

        compute_trajectory_params();

        geometry_msgs::msg::Pose current_pose;
        current_pose.position.x = h_ee_.translation()[0];
        current_pose.position.y = h_ee_.translation()[1];
        current_pose.position.z = h_ee_.translation()[2];
        current_pose.orientation.x = 0;
        current_pose.orientation.y = 0;
        current_pose.orientation.z = 0;
        current_pose.orientation.w = 1;

        publisher_curr_pos->publish(current_pose);
      }
    }
    for (auto ee : ee_names_) {
      if (std::find(msg->ee_name.begin(), msg->ee_name.end(), ee) ==
        msg->ee_name.end())
      {
        auto h_ee_ = TsidPositionControl::robot_wrapper_->framePosition(
          TsidPositionControl::formulation_->data(),
          TsidPositionControl::model_.getFrameId(ee));

        // Setting the reference
        Eigen::Vector3d pos_ = h_ee_.translation();
        Eigen::VectorXd ref = Eigen::VectorXd::Zero(12);
        ref.head(3) = pos_;
        ref.tail(9) << h_ee_.rotation()(0, 0), h_ee_.rotation()(0, 1),
          h_ee_.rotation()(0, 2), h_ee_.rotation()(1, 0),
          h_ee_.rotation()(1, 1), h_ee_.rotation()(1, 2),
          h_ee_.rotation()(2, 0), h_ee_.rotation()(2, 1),
          h_ee_.rotation()(
          2, 2);       // useless because mask not taking orientation, but
                       // required from tsid to put at least identity

        tsid::trajectories::TrajectorySample sample_posture_ee =
          traj_ee_[ee_id_[ee]].computeNext();
        sample_posture_ee.setValue(ref);
        task_ee_[ee_id_[ee]]->setReference(sample_posture_ee);
      }
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Controller is not active, the command will be ignored");
  }
  iteration = 0;
}
void CartesianSpaceController::interpolate(double t_curr)
{
  /* waypoints.clear();
   waypoints_orientations.clear();
   current_waypoint_ = 0;
   std::vector<double> t_;
   for (double i = 0; i <= t1; i += step) {
     t_.push_back(i);
   }

   Eigen::Vector3d v = (pf - p0) / (t1 - t0);

   Eigen::Quaterniond quat(actual_rot);
   Eigen::Quaterniond quat_des(rot_des);
   for (int i = 0; i < t_.size(); i++) {
     double Ti = t0;
     Eigen::Vector3d pos = p0 + v * (t_[i] - Ti);
     waypoints.push_back(pos);
     Eigen::Quaterniond quat = quat.slerp(t_[i] / t1, quat_des);
     waypoints_orientations.push_back(quat);
   }
   interpolate_ = true;*/
  if (position_end_ == position_start_ && quat_init_ == quat_des_) {
    position_curr_ = position_end_;
    rot_des_ = quat_init_.toRotationMatrix();
    return;
  } else if (position_end_ == position_start_) {
    position_curr_ = position_end_;
    double t_ = 2.0;
    if (t_curr > t_) {
      rot_des_ = quat_des_.toRotationMatrix();
      return;
    } else {
      Eigen::Quaterniond quat = quat_init_.slerp(t_curr / (t_), quat_des_);
      rot_des_ = quat.toRotationMatrix();
      return;
    }
  }

  Eigen::Vector3d un_dir_vec = (position_end_ - position_start_) /
    (position_end_ - position_start_).norm();

  double s = 0;
  double s_dot = 0;


  double a_max;
  a_max = v_max / ( 2 * dt_.seconds());
  t_acc_ = v_max / a_max;
  t_flat_ = ((position_end_ - position_start_).norm() - v_max * t_acc_) / v_max;

  Eigen::Quaterniond quat = quat_init_.slerp(t_curr / (t_flat_ + 2 * t_acc_), quat_des_);

  rot_des_ = quat.toRotationMatrix();
  if (t_curr < t_acc_) {
    s = 0.5 * a_max * t_curr * t_curr;
    s_dot = t_curr;
  } else if (t_curr >= t_acc_ && t_curr < t_acc_ + t_flat_) {
    s = v_max * (t_curr - t_acc_ / 2);
    s_dot = v_max;
  } else if (t_curr >= t_acc_ + t_flat_ && t_curr < t_flat_ + 2 * t_acc_) {
    s = (position_end_ - position_start_).norm() - 0.5 * a_max * (t_flat_ + t_acc_ - t_curr) *
      (t_flat_ + t_acc_ - t_curr);
  } else {
    s = (position_end_ - position_start_).norm();
    s_dot = 0;
    rot_des_ = quat_des_.toRotationMatrix();
  }


  position_curr_ = position_start_ + s * un_dir_vec;
  vel_curr_ = s_dot * un_dir_vec;

}

void CartesianSpaceController::compute_trajectory_params()
{
  t_acc_ = 0.0;
  t_flat_ = 0.0;
  a_max = 0.0;
  scale_ = 1.0;

  // Computing timing parameters of position trajectory
  if (position_end_ != position_start_) {
    a_max = v_max / ( 2 * dt_.seconds());
    t_acc_ = v_max / a_max;
    t_flat_ = ((position_end_ - position_start_).norm() - v_max * t_acc_) / v_max;
    un_dir_vec = (position_end_ - position_start_) /
      (position_end_ - position_start_).norm();
  }

  // Computing timing parameters of orientation trajectory
  double t_ang = 0.0;
  if (quat_init_ != quat_des_) {
    double dot_product = quat_init_.dot(quat_des_);
    double theta = 2.0 * std::acos(std::min(1.0, std::abs(dot_product)));
    t_ang = theta / omega_max;
  }

  // Chek if the orientation trajectory is longer than the position trajectory and computing the scale factor
  if (2 * t_acc_ + t_flat_ < t_ang) {
    scale_ = (2 * t_acc_ + t_flat_) / t_ang;
    t_flat_ = t_ang - 2 * t_acc_;
    a_max = a_max * scale_;
  }


}

void CartesianSpaceController::interpolate(double t_curr)
{

  // Check if the trajectory is already reached
  if (position_end_ == position_start_ && quat_init_ == quat_des_) {
    position_curr_ = position_end_;
    rot_des_ = quat_init_.toRotationMatrix();
    return;
  } else if (position_end_ == position_start_) {
    position_curr_ = position_end_;
    double t_ = 2.0;
    if (t_curr > t_) {
      rot_des_ = quat_des_.toRotationMatrix();
      return;
    } else {
      Eigen::Quaterniond quat = quat_init_.slerp(t_curr / (t_), quat_des_);
      rot_des_ = quat.toRotationMatrix();
      return;
    }
  }

  double s = 0;
  double s_dot = 0;

  //Computing slerp interpolation for the orientation
  Eigen::Quaterniond quat = quat_init_.slerp(t_curr / (t_flat_ + 2 * t_acc_), quat_des_);

  rot_des_ = quat.toRotationMatrix();

  // Computing the trajectory based on the current interpolation time
  if (t_curr < t_acc_) {
    s = 0.5 * a_max * t_curr * t_curr;
    s_dot = a_max * t_curr;
  } else if (t_curr >= t_acc_ && t_curr < t_acc_ + t_flat_) {
    s = 0.5 * v_max * scale_ * t_acc_ + v_max * scale_ * (t_curr - t_acc_ );
    s_dot = v_max * scale_;
  } else if (t_curr >= t_acc_ + t_flat_ && t_curr < t_flat_ + 2 * t_acc_) {
    s = 0.5 * v_max * scale_ * t_acc_ + v_max * scale_ * t_flat_ - 0.5 * a_max *
      (t_curr - t_flat_ - t_acc_) *
      (t_curr - t_flat_ - t_acc_);
    s_dot = v_max * scale_ - a_max * (t_curr - t_flat_ - t_acc_ );
  } else {
    s = (position_end_ - position_start_).norm();
    s_dot = 0;
    rot_des_ = quat_des_.toRotationMatrix();
  }


  position_curr_ = position_start_ + s * un_dir_vec;
  vel_curr_ = s_dot * un_dir_vec;

}

void CartesianSpaceController::updateParams()
{
  TsidPositionControl::updateParams();

  v_max = params_.ee_vmax;

}

} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tsid_controllers::CartesianSpaceController,
  controller_interface::ControllerInterface)
