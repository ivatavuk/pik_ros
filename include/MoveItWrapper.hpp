/**
 * @file MoveItWrapper.hpp
 * @author Ivo Vatavuk
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2023
 */

#ifndef MOVEIT_WRAPPER_HPP_
#define MOVEIT_WRAPPER_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <vector>
#include <Eigen/Dense>

typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;
typedef moveit::planning_interface::MoveGroupInterfacePtr MoveGroupInterfacePtr;

class MoveItWrapper
{
public:
  MoveItWrapper(ros::NodeHandle& nh, const std::string& robot_name, const std::string& joint_model_group_name);

  void goToJointPosition(const Eigen::VectorXd& q, double acc_scaling_factor, double vel_scaling_factor) const;

  Eigen::MatrixXd getJacobian(const std::string& link_name, const Eigen::VectorXd& q,
                              bool use_quaternion_representation = false) const;

  Eigen::VectorXd getFramePose(const std::string& link_name, const Eigen::VectorXd& q) const;

  uint32_t getNDof() const;

private:
  MoveGroupInterfacePtr move_group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr kinematic_state_;

  ros::NodeHandle nh_;
  std::string robot_name_ = "";
  std::string joint_model_group_name_;
  std::string robot_model_name_;

  static std::vector<double> eigenVec2StdVec(const Eigen::VectorXd& eig_vec);
};

#endif  // MOVEIT_WRAPPER_HPP_