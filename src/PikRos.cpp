/**
 * @file PikRos.cpp
 * @author Ivo Vatavuk
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2023
 */

#include "PikRos.hpp"
#include <chrono>
#include <numeric>

using Pik = PikRos::Pik;
using IkTask = PikRos::IkTask;
using Gradient = PikRos::Gradient;

//-------------------IkTask-------------------
IkTask::IkTask(IkTaskType t_type, const std::string &t_frame_name, const Eigen::VectorXd &t_desired_value, ApproachAxis t_approach_axis)
: type(t_type), frame_name(t_frame_name), desired_value(t_desired_value), approach_axis(t_approach_axis)
{
  if (type == FRAME_POSITION) {
    if(desired_value.rows() != 3) { 
      ROS_FATAL("IkTask constructor => desired_value of FRAME_POSITION task has to be 3 dimensional");
      ros::shutdown();
      exit(1);
    }
  }
  if (type == FRAME_ORIENTATION) {
    if(desired_value.rows() != 4) {
      ROS_FATAL("IkTask constructor => desired_value of FRAME_ORIENTATION task has to be 4 dimensional");
      ros::shutdown();
      exit(1);
    }
    desired_value.normalize();
  }
  if (type == FRAME_POSE) {
    if(desired_value.rows() != 7) {
      ROS_FATAL("IkTask constructor => desired_value of FRAME_POSE task has to be 7 dimensional");
      ros::shutdown();
      exit(1);
    }
  }
  if (type == FRAME_APPROACH_AXIS) {
    if(desired_value.rows() != 3) {
      ROS_FATAL("IkTask constructor => desired_value of FRAME_APPROACH_AXIS task has to be 3 dimensional");
      ros::shutdown();
      exit(1);
    }
    desired_value.normalize();
  }
};

Eigen::Vector3d IkTask::getAppAxisVector() const
{
  Eigen::Vector3d app_axis_vec;
  if(approach_axis == X)
    app_axis_vec << 1, 0, 0;
  if(approach_axis == Y)
    app_axis_vec << 0, 1, 0;
  if(approach_axis == Z)
    app_axis_vec << 0, 0, 1;
  return app_axis_vec;
}

//-------------------Gradient-------------------
Gradient::Gradient(double initial_value) 
: current_value_(initial_value), 
ex_value_(std::numeric_limits<double>::max()),
gradient_(std::numeric_limits<double>::max()) 
{};

void Gradient::updateGradient(double current_value)
{
  gradient_ = current_value - ex_value_;
  ex_value_ = current_value_;
  current_value_ = current_value;
};
double Gradient::getGradient() const { return gradient_; };

//-------------------Pik-------------------
Pik::Pik( ros::NodeHandle &nh, const std::string &robot_name, 
          const std::string &joint_model_group_name, 
          const Settings &settings ) 
: moveit_wrapper_(nh, robot_name, joint_model_group_name), 
  settings_(settings), n_dof_(moveit_wrapper_.getNDof()) 
{}

Eigen::VectorXd Pik::solve( const std::vector<IkTask> &ik_tasks,
                            const Eigen::VectorXd& q_initial_guess )
{
  //initialize solver
  initializeSolver(ik_tasks, q_initial_guess);

  const auto start_time = std::chrono::steady_clock::now();

  //sequentially solve the ptsc problem
  while(!reachedStoppingCriteria())
  {
    current_ik_iteration_++;
    if(settings_.debug_mode) goToJointPosition(current_q_);

    std::vector<PtscEigen::Task> ptsc_tasks;
    //For each task
    for(uint32_t i = 0; i < ik_tasks.size(); i++)
    {
      //Push back a ptsc task of i-th priority
      Eigen::MatrixXd task_J = getTaskJacobian(current_q_, ik_tasks[i]);
      Eigen::VectorXd task_err = getTaskError(current_q_, ik_tasks[i], 
                                              true ); //clamp_err
      ptsc_tasks.push_back( PtscEigen::Task(task_J, task_err) ); 
      
      //Update gradient of i-th priority
      current_gradients_[i].updateGradient( getTaskError(current_q_, ik_tasks[i]).norm() );
    }
    
    //Check if reached polish solution threshold
    if( settings_.polish_solution && !reached_polish_solution_threshold_ &&
        getAbsGradientSum(current_gradients_) < settings_.polish_gradient_threshold )
    {
      if(settings_.debug_mode) std::cout << "Entered polish solution!\n";
      reached_polish_solution_threshold_ = true;
    }

    Eigen::VectorXd dq = solvePtscProblem(ptsc_tasks);
    current_q_ = limitQ(current_q_ + dq);
    
    //Calculate elapsed time
    const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);
    solver_elapsed_time_ = elapsed_time.count() / 1000.0;

    //Step debug mode movement
    if(settings_.debug_mode)
    {
      std::cout << "Press Enter to step\n";
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
  }
  return current_q_;
}

void Pik::goToJointPosition(const Eigen::VectorXd &q) const
{
  moveit_wrapper_.goToJointPosition(q, 1.0, 1.0);
}

void Pik::printErrors(const Eigen::VectorXd &q, const std::vector<IkTask> &ik_tasks) const
{
  for(uint32_t i = 0; i < ik_tasks.size(); i++)
  {
    std::cout << "Task " << i + 1 << " error = \n" << getTaskError(q, ik_tasks[i]) << "\n";
    std::cout << "Task " << i + 1 << " error norm = " << getTaskError(q, ik_tasks[i]).norm() << "\n";
  }
}

Eigen::MatrixXd Pik::getTaskJacobian( const Eigen::VectorXd &q, IkTask task ) const
{ 
  if(task.type == FRAME_POSE)
    return moveit_wrapper_.getJacobian(task.frame_name, q, false);
  if(task.type == FRAME_POSITION)
    return moveit_wrapper_.getJacobian(task.frame_name, q, false).block(0, 0, 3, n_dof_);
  if(task.type == FRAME_ORIENTATION)
    return moveit_wrapper_.getJacobian(task.frame_name, q, false).block(3, 0, 3, n_dof_);
  if(task.type == FRAME_APPROACH_AXIS)
  {
    Eigen::Quaterniond tool_quaternion = vec2quat( getTaskValue(q, task) );
    auto base_jacobian = moveit_wrapper_.getJacobian(task.frame_name, q, false).block(3, 0, 3, n_dof_);
    //rotate jacobian to local axis
    return (tool_quaternion.toRotationMatrix().inverse() * base_jacobian).block(0, 0, 2, n_dof_);  
  }

  ROS_FATAL("PrioritizedIK::getTaskJacobian => task type unrecognized!");
  ros::shutdown();
  exit(1);
  return Eigen::MatrixXd();
}

Eigen::VectorXd Pik::getTaskValue( const Eigen::VectorXd &q, IkTask task ) const
{
  if(task.type == FRAME_POSE)
    return moveit_wrapper_.getFramePose(task.frame_name, q);
  if(task.type == FRAME_POSITION)
    return moveit_wrapper_.getFramePose(task.frame_name, q).segment(0, 3);
  if(task.type == FRAME_ORIENTATION)
    return moveit_wrapper_.getFramePose(task.frame_name, q).segment(3, 4);
  if(task.type == FRAME_APPROACH_AXIS)
    return moveit_wrapper_.getFramePose(task.frame_name, q).segment(3, 4);

  ROS_FATAL("PrioritizedIK::getTaskValue => task type unrecognized!");
  ros::shutdown();
  exit(1);
  return Eigen::VectorXd();
} 

Eigen::VectorXd Pik::getTaskError( const Eigen::VectorXd &q, IkTask task, bool clamp_err ) const
{
  if(task.type == FRAME_POSE) 
  {
    Eigen::Vector3d desired_position = task.desired_value.segment(0, 3);
    Eigen::Quaterniond desired_quaternion = vec2quat( task.desired_value.segment(3, 4) );

    Eigen::Vector3d pos_err = desired_position - getTaskValue(q, task).segment(0, 3);
    if(clamp_err) 
      pos_err = clampMagnitude(pos_err, settings_.lin_err_clamp_magnitude);

    Eigen::Vector3d ori_err = getOrientationErrorRPY( desired_quaternion , 
                                                      vec2quat( getTaskValue(q, task).segment(3, 4) ), 
                                                      clamp_err );
     
    Eigen::VectorXd return_vec(6);
    return_vec << pos_err, ori_err;
    return return_vec;
  }
  if(task.type == FRAME_POSITION) 
  {
    if(!clamp_err)
      return task.desired_value - getTaskValue(q, task);
    else
      return clampMagnitude((task.desired_value - getTaskValue(q, task)), settings_.lin_err_clamp_magnitude);
  }
  if(task.type == FRAME_ORIENTATION)
  {
    return getOrientationErrorRPY(  vec2quat( task.desired_value ), 
                                    vec2quat( getTaskValue(q, task) ), 
                                    clamp_err );
  }
  if(task.type == FRAME_APPROACH_AXIS)
  {
    Eigen::Quaterniond tool_quaternion = vec2quat( getTaskValue(q, task) );

    Eigen::Vector3d desired_approach_vector = task.desired_value; 
    Eigen::Vector3d current_approach_vector = tool_quaternion.toRotationMatrix() * task.getAppAxisVector();

    double err_angle = acos( current_approach_vector.dot(desired_approach_vector) );

    if(clamp_err && abs(err_angle) > settings_.ang_err_clamp_magnitude)
      err_angle = settings_.ang_err_clamp_magnitude * sign(err_angle);

    Eigen::Vector3d err_axis = current_approach_vector.cross(desired_approach_vector).normalized();

    Eigen::Vector3d local_err_3d = tool_quaternion.toRotationMatrix().inverse() * (err_angle * err_axis);
    return Eigen::Vector2d(local_err_3d.x(), local_err_3d.y());
  }
  ROS_FATAL("PrioritizedIK::getTaskError => task type unrecognized!");
  ros::shutdown();
  exit(1);
  return Eigen::VectorXd();
} 

Eigen::VectorXd Pik::limitQ(const Eigen::VectorXd &q)
{
  Eigen::VectorXd return_q = q;
  for (uint32_t i = 0; i < q.rows(); i++) {
    if(q[i] > M_PI)
      return_q[i] = q[i] - 2*M_PI * (int) (q[i] / M_PI);
    else if (q[i] < -M_PI) 
      return_q[i] = q[i] - 2*M_PI * (int) (q[i] / M_PI);
    else 
      return_q[i] = q[i];
  }
  return return_q;
}

Eigen::VectorXd Pik::limitDq(const Eigen::VectorXd &dq, double dq_limit)
{
  Eigen::VectorXd return_dq = dq;
  double max_dq = 0;
  double scaling_factor = 1.0;
  for (uint32_t i = 0; i < dq.rows(); i++) {
    if(abs(dq[i]) > max_dq) {
      max_dq = abs(dq[i]);
    }
  }

  if(max_dq < dq_limit) return return_dq;

  scaling_factor = dq_limit / max_dq;

  for (uint32_t i = 0; i < dq.rows(); i++) {
    return_dq[i] = dq[i] * scaling_factor;
  }
  return return_dq;
}

Eigen::Quaterniond Pik::vec2quat(const Eigen::VectorXd &q_vec)
{
  if(q_vec.rows() != 4) { 
    ROS_FATAL("vec2quat => input vec error");
    ros::shutdown();
    exit(1);
  }

  return Eigen::Quaterniond(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
}

Eigen::Vector3d Pik::getOrientationErrorRPY(Eigen::Quaterniond desired_quat, 
                                            Eigen::Quaterniond current_quat,
                                            bool clamp_err ) const
{
  auto quaternion_err = desired_quat * current_quat.conjugate();

  quaternion_err.normalize();
  double err_angle = current_quat.angularDistance(desired_quat);
  double sin_angle = sin(err_angle / 2.0);
  
  if(clamp_err && abs(err_angle) > settings_.ang_err_clamp_magnitude)
    err_angle = settings_.ang_err_clamp_magnitude * sign(err_angle);
  
  if(err_angle == 0) return Eigen::Vector3d::Zero();

  Eigen::Vector3d axis (quaternion_err.x() / sin_angle, 
                        quaternion_err.y() / sin_angle, 
                        quaternion_err.z() / sin_angle );
  return axis * err_angle;
}

Eigen::VectorXd Pik::solvePtscProblem(const std::vector<PtscEigen::Task> &ptsc_tasks) const
{
  Eigen::VectorXd dq, limited_dq;
  double dq_limit = settings_.dq_limit;
  if(reached_polish_solution_threshold_) dq_limit = settings_.polish_dq_limit;
  
  if(!settings_.use_constrained_opt)
  {
    PtscEigen::PTSC ptsc_eigen(ptsc_tasks);
    dq = ptsc_eigen.solve();
    limited_dq = limitDq(dq, dq_limit);
  }
  else 
  {
    Eigen::VectorXd upper_bounds = Eigen::VectorXd::Zero(n_dof_);
    Eigen::VectorXd lower_bounds = Eigen::VectorXd::Zero(n_dof_);
    for(uint32_t i = 0; i < n_dof_; i++) {
      upper_bounds(i) = dq_limit;
      lower_bounds(i) = -dq_limit;
    }
    PtscEigen::PTSC ptsc_eigen(ptsc_tasks, lower_bounds, upper_bounds);
    limited_dq = ptsc_eigen.solve();
    limited_dq = limitDq(limited_dq, dq_limit);
  }
  return limited_dq;
}

bool Pik::reachedErrNormThreshold(const Eigen::VectorXd &q, const std::vector<IkTask> &ik_tasks) const
{
  double err_norm = 0.0;
  for(auto ik_task : ik_tasks)
    err_norm += abs( getTaskError(q, ik_task).norm() );
  
  return (err_norm < settings_.err_norm_threshold);
}

double Pik::getAbsGradientSum(const std::vector<Gradient> &gradients)
{
  return std::accumulate(gradients.begin(), gradients.end(), 0.0,
              [](double acc, const Gradient& g){ return acc + std::abs(g.getGradient()); });
}

bool Pik::reachedGradientThreshold(const std::vector<Gradient> &gradients) const
{
  return (getAbsGradientSum(gradients) < settings_.gradient_threshold);
}

Eigen::VectorXd Pik::clampMagnitude(const Eigen::VectorXd &input_vec, double magnitude)
{
  double input_vec_magnitude = input_vec.norm();
  
  if(input_vec_magnitude > magnitude) 
    return magnitude * input_vec / input_vec_magnitude;
  else
    return input_vec;
}

int Pik::sign(double a)
{
  if(a > 0) return 1;
  if(a < 0) return -1;
  return 0;
}

bool Pik::reachedStoppingCriteria() const
{
  if(reachedErrNormThreshold(current_q_, ik_tasks_))
  {
    std::cout << "Solved, tasks are feasible!\n";
    return true;
  }
  if ( reachedGradientThreshold(current_gradients_) )
  {
    std::cout << "Reached local minimum!\n";
    return true;
  }
  if(solver_elapsed_time_ > settings_.timeout && !settings_.debug_mode)
  {
    std::cout << "Reached timeout!\n";
    return true;
  }
  if(current_ik_iteration_ >= settings_.max_iterations)
  {
    std::cout << "Reached max iterations!\n";
    return true;
  }
  return false;
}

void Pik::initializeSolver( const std::vector<IkTask> &ik_tasks,
                            const Eigen::VectorXd& q_initial_guess )
{
  solver_elapsed_time_ = 0;
  current_ik_iteration_ = 0;
  ik_tasks_ = ik_tasks;
  current_q_ = q_initial_guess;
  reached_polish_solution_threshold_ = false;
  current_gradients_.clear();
  for(auto ik_task : ik_tasks)
    current_gradients_.push_back( Gradient( getTaskError(current_q_, ik_task).norm() ));
}