// Copyright 2022, ICube Laboratory, University of Strasbourg
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

#include "mpi_controller/mpi_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mpi_controller
{
using hardware_interface::LoanedCommandInterface;

MPIController::MPIController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn MPIController::on_init()
{
  try
  {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("stiffness", std::vector<double>());
    auto_declare<double>("damping_ratio", 1);
    auto_declare<std::vector<double>>("mass", std::vector<double>());
    auto_declare<int>("control_horizon", 1);
    auto_declare<double>("sampling_time", 0);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MPIController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  // for(auto i = 0ul; i < joint_names_.size(); i++){
  //   auto_declare<std::vector<double>>("constraints." + joint_names_[i] + ".position", std::vector<double>());
  //   auto_declare<std::vector<double>>("constraints." + joint_names_[i] + ".velocity", std::vector<double>());
  //   auto_declare<std::vector<double>>("constraints." + joint_names_[i] + ".acceleration", std::vector<double>());
  // }

  // getting the impedance parameters
  std::vector<double> stiffness = get_node()->get_parameter("stiffness").as_double_array();
  std::vector<double> mass = get_node()->get_parameter("mass").as_double_array();
  double damping_ratio = get_node()->get_parameter("damping_ratio").as_double();

  if(stiffness.empty())
    stiffness.resize(joint_names_.size(),50.0);
  if(mass.empty())
    mass.resize(joint_names_.size(),1.0);
  if(damping_ratio < 0)
    damping_ratio = 0.7;

  if(stiffness.size() != mass.size()){
    RCLCPP_ERROR(get_node()->get_logger(), "incoherent size of impedance parameters");
    return CallbackReturn::FAILURE;
  }

  for(auto i = 0ul; i < stiffness.size();i++){
    if (stiffness[i] < 0 || mass[i] < 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "wrong impedance parameters");
      return CallbackReturn::FAILURE;
    }
  }

  stiffnessMat_ = Eigen::MatrixXd::Identity(joint_names_.size(),joint_names_.size());
  massMat_ = Eigen::MatrixXd::Identity(joint_names_.size(),joint_names_.size());
  dampingMat_ = Eigen::MatrixXd::Identity(joint_names_.size(),joint_names_.size());

  for(int i=0;i<joint_names_.size();i++){
    stiffnessMat_(i,i) = stiffness[i];
    massMat_(i,i) = mass[i];
    dampingMat_(i,i) = 2*damping_ratio*sqrt(stiffnessMat_(i,i)/massMat_(i,i))*massMat_(i,i);
  }
  int nu = joint_names_.size();
  int nx = 3*joint_names_.size();
  int N = get_node()->get_parameter("control_horizon").as_int();
  double Ts = get_node()->get_parameter("sampling_time").as_double();
  if(Ts == 0){
    RCLCPP_ERROR(get_node()->get_logger(), "missing sampling_time parameter");
    return CallbackReturn::FAILURE;
  }

  Eigen::MatrixXd K(nu,nx);
  K.block(0,0,nu,nu) = massMat_.block(0,0,nu,nu).inverse()*dampingMat_.block(0,0,nu,nu);
  K.block(0,nu,nu,nu) = massMat_.block(0,0,nu,nu).inverse()*stiffnessMat_.block(0,0,nu,nu);
  K.block(0,2*nu,nu,nu) = -massMat_.block(0,0,nu,nu).inverse()*Eigen::MatrixXd::Identity(nu,nu);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx,nx);
  A.block(nu,0,nu,nu) = Ts*Eigen::MatrixXd::Identity(nu,nu);

  Eigen::MatrixXd B= Eigen::MatrixXd::Zero(nx,nu);
  B.block(0,0,nu,nu) = Ts*Eigen::MatrixXd::Identity(nu,nu);

  Eigen::MatrixXd selectX = Eigen::MatrixXd::Zero(nx,1);
  Eigen::MatrixXd selectu = Eigen::MatrixXd::Zero(nu,1);

  Eigen::MatrixXd Xmax = Eigen::MatrixXd::Zero(nx,1);
  Eigen::MatrixXd Xmin = Eigen::MatrixXd::Zero(nx,1);
  Eigen::MatrixXd umax = Eigen::MatrixXd::Zero(nu,1);
  Eigen::MatrixXd umin = Eigen::MatrixXd::Zero(nu,1);

  // selectX = select [velocity, position, external torque => no sens for != 0] (0 or 1)
  // selectu = select acceleration (0 or 1)
  // Xmax = [velocity, position, external torque => no sens for != 0]
  // Xmin = [velocity, position, external torque => no sens for != 0]
  // umax = max acceleration
  // umin = min acceleration

  // for(auto i = 0ul; i < joint_names_.size(); i++){
  //   std::vector<double> plimits = get_node()->get_parameter("constraints." + joint_names_[i] + ".position").as_double_array();
  //   std::vector<double> vlimits = get_node()->get_parameter("constraints." + joint_names_[i] + ".velocity").as_double_array();
  //   std::vector<double> alimits = get_node()->get_parameter("constraints." + joint_names_[i] + ".acceleration").as_double_array();

  //   if(!vlimits.empty()){
  //     Xmax(i,0) = vlimits[1];
  //     Xmin(i,0) = vlimits[0];
  //   } 
  //   if(!plimits.empty()){
  //     selectX(nu+i,0) = 1;
  //     Xmax(nu+i,0) = plimits[1];
  //     Xmin(nu+i,0) = plimits[0];
  //   }
  //   if(!alimits.empty()){
  //     selectu(i,0) = 1;
  //     umax(i,0) = alimits[1];
  //     umin(i,0) = alimits[0];
  //   }
  // }

  mpic_ = new MPIC(nx,nu,N);
  mpic_->setTimeStep(Ts);
  mpic_->setSystem(A,B,K);
  mpic_->addConstraintsX(selectX,Xmax,Xmin);
  mpic_->addConstraintsU(selectu,umax,umin);
  mpic_->computeQP();

  stateX_ = Eigen::VectorXd::Zero(mpic_->getDimX());
  rk_ = MatrixXd::Zero(mpic_->getHorizon()*(mpic_->getDimU()+mpic_->getDimX()),1);
  rku_tmp_ = MatrixXd::Zero(mpic_->getDimU()*mpic_->getHorizon(),1);
  rkX_tmp_ = MatrixXd::Zero(mpic_->getDimX()*mpic_->getHorizon(),1);
  uOpt_ = MatrixXd::Zero(mpic_->getDimU(),1);

  solved_first_ = false;

  // the desired reference trajectory is queried from the reference topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As impedance control targets the effort interface, it can be directly defined here
// without the need of getting as parameter. The effort interface is then affected to
// all controlled joints.
controller_interface::InterfaceConfiguration
MPIController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_)
  {
    conf.names.push_back(joint_name + "/" + "acceleration");
  }
  return conf;
}
// Impedance control requires both velocity and position states. For this reason
// there can be directly defined here without the need of getting as parameters.
// The state interfaces are then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
MPIController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * 2);
  for (const auto & joint_name : joint_names_)
  {
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      conf.names.push_back(joint_name + "/" + "external_effort");
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names)
  {
    for (auto & command_interface : unordered_interfaces)
    {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn MPIController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_ACCELERATION, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
// When deactivating the controller, the effort command on all joints is set to 0
CallbackReturn MPIController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (auto index = 0ul; index < joint_names_.size(); ++index)
    {
        command_interfaces_[index].set_value(0.0);
    }
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type MPIController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // getting the data from the subscriber using the rt pipe
  auto reference = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!reference || !(*reference))
  {
    return controller_interface::return_type::OK;
  }

  //checking reference data validity
  if ((*reference)->joint_names.size() != joint_names_.size() ||
      (*reference)->points[0].positions.size() != joint_names_.size())  {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,"command size does not match number of interfaces");
    return controller_interface::return_type::ERROR;
  }
  // checkoing if new reference
  if((*reference)->header.stamp != previous_stamp_){
    reference_index_ = 0;
    previous_stamp_ = (*reference)->header.stamp;
  }
  // updating reference for prediction
  for (int i = 0; i < mpic_->getHorizon(); ++i) {
    if((reference_index_+i)<(*reference)->points.size()){
      for (int j =0; j < joint_names_.size(); j++) {
          rku_tmp_(mpic_->getDimU()*i+j) = (*reference)->points[reference_index_+i].accelerations[j];
          rkX_tmp_(mpic_->getDimX()*i+j) = (*reference)->points[reference_index_+i].velocities[j];
          rkX_tmp_(mpic_->getDimX()*i+mpic_->getDimU()+j) = (*reference)->points[reference_index_+i].positions[j];
          rkX_tmp_(mpic_->getDimX()*i+2*mpic_->getDimU()+j) = 0.0;
      }
    }
    else{
      for (int j=0; j < mpic_->getDimU(); j++) {
          rku_tmp_(mpic_->getDimU()*i+j) = 0.0;
          rkX_tmp_(mpic_->getDimX()*i+j) = 0.0;
          rkX_tmp_(mpic_->getDimX()*i+mpic_->getDimU()+j) = (*reference)->points[(*reference)->points.size()+i].positions[j];
          rkX_tmp_(mpic_->getDimX()*i+2*mpic_->getDimU()+j) = 0.0;
      }
    }
  }
  rk_ << rku_tmp_, rkX_tmp_;
  reference_index_ ++;

  //Model Predictive Impedance Control loop
  Eigen::VectorXd q = Eigen::VectorXd::Zero(joint_names_.size());
  Eigen::VectorXd qv = Eigen::VectorXd::Zero(joint_names_.size());
  Eigen::VectorXd te = Eigen::VectorXd::Zero(joint_names_.size());

  for (auto index = 0ul; index < joint_names_.size(); ++index){
    // the stats are given in the same order as defines in state_interface_configuration
    q(index) = state_interfaces_[3*index].get_value();
    qv(index) = state_interfaces_[3*index+1].get_value();
    te(index) = state_interfaces_[3*index+2].get_value();
  }

  stateX_.head(mpic_->getDimU()) = qv.head(mpic_->getDimU());
  stateX_.segment(mpic_->getDimU(),mpic_->getDimU()) = q.head(mpic_->getDimU());
  stateX_.tail(mpic_->getDimU())= te.head(mpic_->getDimU());

  if(!solved_first_){
    mpic_->firstSolveMPIC(stateX_,rk_);
    solved_first_ = true;
  }
  else mpic_->updateSolveMPIC(stateX_,rk_);

  if(mpic_->_QPfail)
    RCLCPP_WARN(get_node()->get_logger(), "MPIC: failed solving QP");
  else 
    uOpt_ = mpic_->getuOpt();

  for (auto index = 0ul; index < joint_names_.size(); ++index){
    command_interfaces_[index].set_value(uOpt_(index));
  }

  return controller_interface::return_type::OK;
}

}  // namespace mpi_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mpi_controller::MPIController, controller_interface::ControllerInterface)
