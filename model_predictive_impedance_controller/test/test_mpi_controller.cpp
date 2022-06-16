// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include <stddef.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_mpi_controller.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using CmdType = trajectory_msgs::msg::JointTrajectory;

namespace
{
rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}  // namespace

void MPIControllerTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void MPIControllerTest::TearDownTestCase() { rclcpp::shutdown(); }

void MPIControllerTest::SetUp()
{
  controller_ = std::make_unique<FriendMPIController>();
}

void MPIControllerTest::TearDown() { controller_.reset(nullptr); }

void MPIControllerTest::SetUpController()
{
  const auto result = controller_->init("test_mpi_controller");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint1_ci_);
  command_ifs.emplace_back(joint2_ci_);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint1_sip_);
  state_ifs.emplace_back(joint1_siv_);
  state_ifs.emplace_back(joint1_sie_);
  state_ifs.emplace_back(joint2_sip_);
  state_ifs.emplace_back(joint2_siv_);
  state_ifs.emplace_back(joint2_sie_);

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(MPIControllerTest, JointsParameterNotSet)
{
  SetUpController();

  // configure failed, 'joints' parameter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MPIControllerTest, JointsParameterIsEmpty)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", std::vector<std::string>()});

  // configure failed, 'joints' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MPIControllerTest, ConfigureAndActivateParamsSuccess)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"sampling_time", 0.005});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MPIControllerTest, ActivateWithWrongJointNamesFails)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", std::vector<std::string>{"joint1", "joint4"}});
  controller_->get_node()->set_parameter({"sampling_time", 0.005});
  // // activate failed, 'joint4' is not a valid joint name for the hardware
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(MPIControllerTest, CommandSuccessTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"sampling_time", 0.005});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful though no command has been send yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::OK);

  // check joint commands and states are still the default ones
  ASSERT_EQ(joint1_ci_.get_value(), 0.0);
  ASSERT_EQ(joint2_ci_.get_value(), 0.0);
  ASSERT_EQ(joint1_sip_.get_value(), 0.0);
  ASSERT_EQ(joint2_sip_.get_value(), 0.0);
  ASSERT_EQ(joint1_siv_.get_value(), 0.0);
  ASSERT_EQ(joint2_siv_.get_value(), 0.0);
  ASSERT_EQ(joint1_sie_.get_value(), 0.0);
  ASSERT_EQ(joint2_sie_.get_value(), 0.0);

  // send command
  auto command_ptr = std::make_shared<CmdType>();
  command_ptr->joint_names = joint_names_;
  command_ptr->points.resize(1);
  command_ptr->points[0].positions = {1,2};
  command_ptr->points[0].velocities = {0,0};
  command_ptr->points[0].accelerations = {0,0};
  
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::OK);

  // check joint commands have been modified
  ASSERT_EQ(joint1_ci_.get_value(), 50.0);
  ASSERT_EQ(joint2_ci_.get_value(), 100.0);
}

TEST_F(MPIControllerTest, WrongCommandCheckTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"sampling_time", 0.005});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // send command with wrong number of joints
  auto command_ptr = std::make_shared<CmdType>();
  command_ptr->joint_names = {"joint1"};
  command_ptr->points.resize(1);
  command_ptr->points[0].positions = {1};
  command_ptr->points[0].velocities = {0};
  command_ptr->points[0].accelerations = {0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update failed, command size does not match number of gpios
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::ERROR);

  // check joint commands are still the default ones
  ASSERT_EQ(joint1_ci_.get_value(), 0.0);
  ASSERT_EQ(joint2_ci_.get_value(), 0.0);
}

TEST_F(MPIControllerTest, NoCommandCheckTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"sampling_time", 0.005});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful, no command received yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::OK);

  // check joint commands are still the default ones
  ASSERT_EQ(joint1_ci_.get_value(), 0.0);
  ASSERT_EQ(joint2_ci_.get_value(), 0.0);
}

TEST_F(MPIControllerTest, CommandSuccessTestWithConstraints)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"sampling_time", 0.005});
  controller_->get_node()->set_parameter({"constraints." + joint_names_[0] + ".acceleration", std::vector<double>{-10, 10}});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful though no command has been send yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::OK);

  // check joint commands and states are still the default ones
  ASSERT_EQ(joint1_ci_.get_value(), 0.0);
  ASSERT_EQ(joint2_ci_.get_value(), 0.0);
  ASSERT_EQ(joint1_sip_.get_value(), 0.0);
  ASSERT_EQ(joint2_sip_.get_value(), 0.0);
  ASSERT_EQ(joint1_siv_.get_value(), 0.0);
  ASSERT_EQ(joint2_siv_.get_value(), 0.0);
  ASSERT_EQ(joint1_sie_.get_value(), 0.0);
  ASSERT_EQ(joint2_sie_.get_value(), 0.0);

  // send command
  auto command_ptr = std::make_shared<CmdType>();
  command_ptr->joint_names = joint_names_;
  command_ptr->points.resize(1);
  command_ptr->points[0].positions = {1,2};
  command_ptr->points[0].velocities = {0,0};
  command_ptr->points[0].accelerations = {0,0};
  
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::OK);

  // check joint commands have been modified
  ASSERT_EQ(joint1_ci_.get_value(), 50.0);
  ASSERT_EQ(joint2_ci_.get_value(), 100.0);
}