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
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_mpi_controller.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
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
  controller_->assign_interfaces(std::move(command_ifs), {});
}

TEST_F(MPIControllerTest, JointsParameterNotSet)
{
  SetUpController();

  // configure failed, 'joints' parameter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(MPIControllerTest, JointsParameterIsEmpty)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", std::vector<std::string>()});

  // configure failed, 'joints' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(MPIControllerTest, ConfigureAndActivateParamsSuccess)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MPIControllerTest, ActivateWithWrongGpiosNamesFails)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joints", std::vector<std::string>{"joint1", "joint4"}});
  // // activate failed, 'gpio4' is not a valid gpio name for the hardware
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}