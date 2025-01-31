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
namespace tsid_controllers {
using std::placeholders::_1;

CartesianSpaceController::CartesianSpaceController()
    : tsid_controllers::TsidPositionControl(), dt_(0, 0) {}

controller_interface::CallbackReturn CartesianSpaceController::on_init() {
  return TsidPositionControl::on_init();
}

controller_interface::CallbackReturn CartesianSpaceController::on_configure(
    const rclcpp_lifecycle::State &prev_state) {
  auto result = TsidPositionControl::on_configure(prev_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  local_frame_ = getParams().local_frame;

  if (local_frame_) {
    RCLCPP_INFO(
        get_node()->get_logger(),
        "The reference is considered as expressed in the end effector frame");
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
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
  for (const auto &ee : ee_names_) {
    task_ee_.push_back(new tsid::tasks::TaskSE3Equality(
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

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianSpaceController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto result = TsidPositionControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  // Setting initial reference for the end effector tasks
  for (auto ee : ee_names_) {
    H_ee_0_[ee_id_[ee]] = robot_wrapper_->framePosition(
        TsidPositionControl::formulation_->data(),
        TsidPositionControl::model_.getFrameId(ee));
    traj_ee_.push_back(tsid::trajectories::TrajectorySE3Constant(
        "traj_ee", H_ee_0_[ee_id_[ee]]));
    tsid::trajectories::TrajectorySample sample_posture_ee =
        traj_ee_[ee_id_[ee]].computeNext();
    task_ee_[ee_id_[ee]]->setReference(sample_posture_ee);

    RCLCPP_INFO(get_node()->get_logger(), " Initial position ee %s : %f %f %f",
                ee.c_str(), H_ee_0_[ee_id_[ee]].translation()[0],
                H_ee_0_[ee_id_[ee]].translation()[1],
                H_ee_0_[ee_id_[ee]].translation()[2]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
CartesianSpaceController::update(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  TsidPositionControl::updateParams();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;
  compute_problem_and_set_command(state.first, state.second); // q and v
  auto h_ee_ = TsidPositionControl::robot_wrapper_->framePosition(
      TsidPositionControl::formulation_->data(),
      TsidPositionControl::model_.getFrameId(ee_names_[0]));

  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = h_ee_.translation()[0];
  current_pose.position.y = h_ee_.translation()[1];
  current_pose.position.z = h_ee_.translation()[2];
  current_pose.orientation.x = 0;
  current_pose.orientation.y = 0;
  current_pose.orientation.z = 0;
  current_pose.orientation.w = 1;

  publisher_curr_pos->publish(current_pose);

  return controller_interface::return_type::OK;
}

void CartesianSpaceController::setPoseCallback(
    tsid_controller_msgs::msg::EePos::ConstSharedPtr msg) {
  if (get_node()->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    for (int i = 0; i < msg->ee_name.size(); i++) {
      if (std::find(ee_names_.begin(), ee_names_.end(), msg->ee_name[i]) ==
          ee_names_.end()) {
        RCLCPP_WARN(get_node()->get_logger(),
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

          Eigen::Matrix3d rot_des = quat.toRotationMatrix() * h_ee_.rotation();

          pinocchio::SE3 se3(rot_des, desired_pose_[ee_id_[ee]]);
          tsid::math::SE3ToVector(se3, ref);
        }

        tsid::trajectories::TrajectorySample sample_posture_ee =
            traj_ee_[ee_id_[ee]].computeNext();
        sample_posture_ee.setValue(ref);
        task_ee_[ee_id_[ee]]->setReference(sample_posture_ee);

        auto ref_ee = task_ee_[ee_id_[ee]]->getReference();
        auto ref_pos = ref_ee.getValue();

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
          msg->ee_name.end()) {
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
                2, 2); // useless because mask not taking orientation, but
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
}

} // namespace tsid_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tsid_controllers::CartesianSpaceController,
                       controller_interface::ControllerInterface)