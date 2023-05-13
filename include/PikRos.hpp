/**
 * @file PikRos.hpp
 * @author Ivo Vatavuk
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2023
 */

#ifndef PIK_ROS_H_
#define PIK_ROS_H_

#include "MoveItWrapper.hpp"

#include <Eigen/Dense>
#include <PtscEigen.hpp>

namespace PikRos
{

/** @enum Ik task type */
enum IkTaskType
{
  FRAME_POSITION, /** < Frame position task, has a value of [x, y, z]^T in meters */
  FRAME_ORIENTATION, /** < Frame orientation task, has a quaternion value [w, x, y, z]^T */
  FRAME_POSE, /** < Frame pose task, a combination of FRAME_POSITION and FRAME_ORIENTATION tasks */
  FRAME_APPROACH_AXIS /** < Frame approach axis task, has a value of a 3D vector [x, y, z]^T */
};

/** @enum Approach axis
 * 
 * Which axis of the target frame is considered its approach axis
 */
enum ApproachAxis
{
  X,
  Y,
  Z
};

/** @brief Ik task structure */
struct IkTask
{
  /**
   * @brief Construct a new Ik Task object
   * 
   * @param t_type IkTask type
   * @param t_frame_name MoveIt frame name
   * @param t_desired_value IkTask desired value
   * @param t_approach_axis Which frame axis is considered the approach axis. Used only for the task type FRAME_APPROACH_AXIS
   */
  IkTask( IkTaskType t_type, const std::string &t_frame_name, 
          const Eigen::VectorXd &t_desired_value, 
          ApproachAxis t_approach_axis = Z);

  /** @brief Get the approach axis Vector */
  Eigen::Vector3d getAppAxisVector() const;

  IkTaskType type;
  std::string frame_name;
  Eigen::VectorXd desired_value;
  ApproachAxis approach_axis;
};

/** @brief Solver settings */
struct Settings 
{ 
  uint32_t max_iterations = 10000; /**< Maximum number of solve iterations */
  double  err_norm_threshold = 1e-3; /**< Threshold for the sum of task error norms */ 
  double gradient_threshold = 1e-3; /**< Threshold for the sum of task gradients */
  double polish_gradient_threshold = 1e-2; /**< When sum of task gradients reaches this threshold solution polishing starts */
  double dq_limit = (double)(10.0 * M_PI / 180.0); /**< dq limit */
  double polish_dq_limit = (double)(3.0 * M_PI / 180.0); /**< dq limit for solution polishing */
  double lin_err_clamp_magnitude = 0.2; /**< Clamp magnitude for linear tasks */
  double ang_err_clamp_magnitude = 15.0 * M_PI / 180.0; /**< Clamp magnitude for angular tasks */
  bool use_constrained_opt = true;
  bool polish_solution = false; 
  bool debug_mode = false; /**< Move the arm in gazebo for every step with verbose output */
  double timeout = 2.0; /**< Solver timeout in seconds */
};

/** @brief Gradient class */
class Gradient
{
public:
  Gradient(double initial_value);
  void updateGradient(double current_value);
  double getGradient() const;
private:
  double current_value_;
  double ex_value_;
  double gradient_;
};

/**
 * @brief Prioritized inverse kinematics solver
 * @details
 * Solves the positional inverse kinematics problem given a vector of Ik tasks with descending priorities
 */
class Pik
{
public:
  /**
   * @brief Construct a new Pik object
   * @param nh ROS node handle
   * @param robot_name Namespace for robot description 
   * @param joint_model_group_name Joint model group name in MoveIt
   * @param settings PikRos::Settings object
   */
  Pik(ros::NodeHandle &nh, const std::string &robot_name, 
      const std::string &joint_model_group_name,
      const Settings &settings = Settings());

  /**
   * @brief Solves a prioritized inverse kinematics problem
   * @param ik_tasks Ik tasks std::vector<IkTask> from higher priority to lower  
   * @param q_initial_guess Initial guess for a joint position vector q
   * @return Eigen::VectorXd solution joint positions
   */
  Eigen::VectorXd solve(const std::vector<IkTask> &ik_tasks, 
                        const Eigen::VectorXd& q_initial_guess );

  /**
   * @brief Moves the robot to desired joint positions
   * @param q Eigen::VectorXd Joint position vector
   */
  void goToJointPosition(const Eigen::VectorXd &q) const;

  /**
   * @brief Prints task errors for a given joint position vector
   * @param q Eigen::VectorXd Joint position vector
   * @param ik_tasks std::vector<IkTask> Inverse kinematics tasks;
   */
  void printErrors(const Eigen::VectorXd &q, const std::vector<IkTask> &ik_tasks) const;

private:
  MoveItWrapper moveit_wrapper_;
  Settings settings_;
  const uint32_t n_dof_;

  Eigen::Vector3d getOrientationErrorRPY( Eigen::Quaterniond desired_quat, 
                                          Eigen::Quaterniond current_quat,
                                          bool clamp_err = false ) const;
          
  bool reachedErrNormThreshold(const Eigen::VectorXd &q, const std::vector<IkTask> &ik_tasks) const;
  static double getAbsGradientSum(const std::vector<Gradient> &gradients);
  bool reachedGradientThreshold(const std::vector<Gradient> &gradients) const;
  
  Eigen::MatrixXd getTaskJacobian( const Eigen::VectorXd &q, IkTask task ) const; 
  Eigen::VectorXd solvePtscProblem(const std::vector<PtscEigen::Task> &ptsc_tasks) const;

  Eigen::VectorXd getTaskValue( const Eigen::VectorXd &q, IkTask task ) const; //These functions go to IkTask!
  Eigen::VectorXd getTaskError( const Eigen::VectorXd &q, IkTask task, bool clamp_err = false ) const; 

  static Eigen::VectorXd clampMagnitude(const Eigen::VectorXd &input_vec, double magnitude);
  static Eigen::VectorXd limitQ(const Eigen::VectorXd &q);
  static Eigen::VectorXd limitDq(const Eigen::VectorXd &dq, double dq_limit);
  static Eigen::Quaterniond vec2quat(const Eigen::VectorXd &q_vec);

  bool reached_polish_solution_threshold_ = false;

  static int sign(double a);

  bool reachedStoppingCriteria() const; 
  void initializeSolver(const std::vector<IkTask> &ik_tasks,
                        const Eigen::VectorXd& q_initial_guess); 

  Eigen::VectorXd current_q_;
  std::vector<PikRos::IkTask> ik_tasks_;
  double solver_elapsed_time_; //in seconds
  std::vector<Gradient> current_gradients_;
  uint32_t current_ik_iteration_;
};
};
#endif //PIK_ROS_H_