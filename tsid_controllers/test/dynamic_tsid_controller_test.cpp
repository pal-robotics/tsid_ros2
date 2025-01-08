// Copyright 2024 PAL Robotics SL.
// All Rights Reserved
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.


#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "tsid_controllers/cartesian_space_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"


using namespace controller_interface;


class TestDynamicTsidControl : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase()
  {

  }

  void SetUp()
  {
    // initialize controller
    controller_ =
      std::make_unique<  tsid_controllers::CartesianSpaceController>();
  }

  void TearDown()
  {
    controller_.reset(nullptr);
  }

  void SetUpController()
  {

    auto node_options = controller_->define_custom_node_options();
    std::vector<rclcpp::Parameter> parameter_overrides;
    parameter_overrides.push_back(rclcpp::Parameter("joint_names", joint_name_));
    node_options.parameter_overrides(parameter_overrides);
    const auto result = controller_->init(
      "dynamic_tsid_controller", "", 100.0, "", node_options);
    ASSERT_EQ(result, controller_interface::return_type::OK);
    controller_->get_node()->set_parameter(rclcpp::Parameter("update_rate", 2000));

  }

protected:
  std::unique_ptr<  tsid_controllers::CartesianSpaceController>
  controller_;

  std::vector<double> joint_states_ = {1.1, 2.1, 3.1};
  std::vector<double> joint_commands_ = {3.1};


  // dummy joint state values used for tests
  std::vector<std::string> joint_name_;

};


// Test

TEST_F(TestDynamicTsidControl, JointNameNotSet)
{
  this->SetUpController();

  // configure failed, 'joint_names' parameter is not set
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);

}


TEST_F(TestDynamicTsidControl, ConfigureSuccess)
{
  this->joint_name_.push_back("joint_1");
  this->joint_name_.push_back("joint_2");

  this->SetUpController();

  // configure successful
  ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // check interface configuration
  auto cmd_if_conf = this->controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, testing::SizeIs(2lu));
  ASSERT_THAT(cmd_if_conf.names, testing::UnorderedElementsAre("joint_1/effort", "joint_2/effort"));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto state_if_conf = this->controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, testing::SizeIs(6lu));
  std::string state_if_conf1 = state_if_conf.names[0];
  EXPECT_EQ(state_if_conf1, "joint_1/position");
  std::string state_if_conf2 = state_if_conf.names[1];
  EXPECT_EQ(state_if_conf2, "joint_1/velocity");
  std::string state_if_conf3 = state_if_conf.names[2];
  EXPECT_EQ(state_if_conf3, "joint_1/effort");
  std::string state_if_conf4 = state_if_conf.names[3];
  EXPECT_EQ(state_if_conf4, "joint_2/position");
  std::string state_if_conf5 = state_if_conf.names[4];
  EXPECT_EQ(state_if_conf5, "joint_2/velocity");
  std::string state_if_conf6 = state_if_conf.names[5];
  EXPECT_EQ(state_if_conf6, "joint_2/effort");
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}


TEST_F(TestDynamicTsidControl, ActivateSuccess)
{
  this->joint_name_.push_back("joint_1");
  this->joint_name_.push_back("joint_2");

  this->SetUpController();

  hardware_interface::StateInterface joint_1_pos_state_{this->joint_name_[0],
    hardware_interface::HW_IF_POSITION, &joint_states_[0]};
  hardware_interface::StateInterface joint_1_vel_state_{this->joint_name_[0],
    hardware_interface::HW_IF_VELOCITY, &joint_states_[1]};
  hardware_interface::StateInterface joint_1_eff_state_{this->joint_name_[0],
    hardware_interface::HW_IF_EFFORT, &joint_states_[2]};
  hardware_interface::CommandInterface joint_1_cmd_{this->joint_name_[0],
    hardware_interface::HW_IF_EFFORT,
    &joint_commands_[0]};

  hardware_interface::StateInterface joint_2_pos_state_{this->joint_name_[1],
    hardware_interface::HW_IF_POSITION, &joint_states_[0]};
  hardware_interface::StateInterface joint_2_vel_state_{this->joint_name_[1],
    hardware_interface::HW_IF_VELOCITY, &joint_states_[1]};
  hardware_interface::StateInterface joint_2_eff_state_{this->joint_name_[1],
    hardware_interface::HW_IF_EFFORT, &joint_states_[2]};
  hardware_interface::CommandInterface joint_2_cmd_{this->joint_name_[1],
    hardware_interface::HW_IF_EFFORT,
    &joint_commands_[1]};


  std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint_1_cmd_);
  command_ifs.emplace_back(joint_2_cmd_);

  std::vector<hardware_interface::LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  state_ifs.emplace_back(joint_1_eff_state_);
  state_ifs.emplace_back(joint_2_pos_state_);
  state_ifs.emplace_back(joint_2_vel_state_);
  state_ifs.emplace_back(joint_2_eff_state_);


  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  // activate successful
  ASSERT_EQ(
    controller_->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    this->controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}
