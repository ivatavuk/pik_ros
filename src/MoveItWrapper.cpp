/**
 * @file MoveItWrapper.cpp
 * @author Ivo Vatavuk
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2023
 */

#include "MoveItWrapper.hpp"
#include <iostream>

MoveItWrapper::MoveItWrapper(ros::NodeHandle& nh, const std::string& robot_name,
                             const std::string& joint_model_group_name)
  : nh_(nh), robot_name_(robot_name), joint_model_group_name_(joint_model_group_name)
{
  robot_model_name_ = "/" + robot_name_ + "/robot_description";

  std::cout << "MoveItWrapper: RobotModelLoader initialization...\n";

  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_model_name_);

  auto kinematic_model = robot_model_loader_->getModel();

  if (kinematic_model)
  {
    joint_model_group_ = (kinematic_model)->getJointModelGroup(joint_model_group_name_);
    if (joint_model_group_)
    {
      kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
      kinematic_state_->setToDefaultValues();
    }
  }

  std::cout << "MoveItWrapper: MoveGroupInterface initialization...\n";
  MoveGroupInterface::Options opt(joint_model_group_name_, robot_model_name_);
  move_group_ = std::make_shared<MoveGroupInterface>(opt);
  std::cout << "MoveItWrapper: Initialization successfull!\n";
}

Eigen::MatrixXd MoveItWrapper::getJacobian(const std::string& link_name, const Eigen::VectorXd& q,
                                           bool use_quaternion_representation) const
{
  kinematic_state_->setJointGroupPositions(joint_model_group_, q);

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  bool jacobian_computed =
      kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(link_name),
                                    reference_point_position, jacobian, use_quaternion_representation);

  if (!jacobian_computed)
  {
    throw std::runtime_error("Jacobian not computed!");
  }
  return jacobian;
}

Eigen::VectorXd MoveItWrapper::getFramePose(const std::string& link_name, const Eigen::VectorXd& q) const
{
  kinematic_state_->setJointGroupPositions(joint_model_group_, q);
  auto transform = kinematic_state_->getFrameTransform(link_name);

  auto translation = transform.translation();
  auto rotation = transform.rotation();

  Eigen::VectorXd return_vector = Eigen::VectorXd::Zero(7);
  Eigen::Quaternion<double> quaternion(rotation);
  return_vector << translation[0], translation[1], translation[2], quaternion.w(), quaternion.x(), quaternion.y(),
      quaternion.z();
  return return_vector;
}

void MoveItWrapper::goToJointPosition(const Eigen::VectorXd& q, double acc_scaling_factor,
                                      double vel_scaling_factor) const
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  move_group_->setJointValueTarget(eigenVec2StdVec(q));
  move_group_->setMaxAccelerationScalingFactor(acc_scaling_factor);
  move_group_->setMaxVelocityScalingFactor(vel_scaling_factor);
  move_group_->plan(plan);

  auto err = move_group_->execute(plan);

  if (err != 1)
  {
    throw std::runtime_error("MoveItWrapper::goToJointPosition error");
  }
}

uint32_t MoveItWrapper::getNDof() const
{
  return joint_model_group_->getActiveVariableCount();
}

std::vector<double> MoveItWrapper::eigenVec2StdVec(const Eigen::VectorXd& eig_vec)
{
  std::vector<double> vec;
  for (uint32_t i = 0; i < eig_vec.rows(); i++)
  {
    vec.push_back(eig_vec[i]);
  }
  return vec;
}