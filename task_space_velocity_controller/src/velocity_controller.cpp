/** -------------------------------------------------------------------
 * @file:   velocity_controller.cpp
 * @author:  Group 2 Robotics Project 2023/2024 UNISA
 * @date:    Gen 20, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "task_space_velocity_controller/velocity_controller.hpp"
#include <controller_interface/controller_interface.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logging.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace fanuc_m20ia_35m_planning_demo
{
class ActionNode : public rclcpp::Node
{

};
}  // namespace fanuc_m20ia_35m_planning_demo


namespace velocity_controller
{
VelocityController::VelocityController(): controller_interface::ControllerInterface()
{
}

rclcpp_action::GoalResponse VelocityController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionCartesianTrajectory::Goal> goal)
{

  RCLCPP_INFO(get_node()->get_logger(), "Received goal request");
  // If the controller is not running, we reject the goal  
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Accepted goal request");

  // If the goal is not valid, we reject it
  if(goal->trajectory.points.size() < 1){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get hold position. size() < 1");
    return rclcpp_action::GoalResponse::REJECT;
  }

  (void)uuid;

  // Otherwise, we accept the goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse VelocityController::handle_cancel(
    const std::shared_ptr<GoalHandleCartesianTrajectory> goal_handle)
  {

    RCLCPP_INFO(get_node()->get_logger(), "Canceling active action goal because cancel callback received.");
    // Set the goal flag to false, so that the update method can stop the execution
    this->trajectory_index_ = 0;
    active_goal_ = false;
    (void)goal_handle;
    // Return with the result that the goal was canceled
    return rclcpp_action::CancelResponse::ACCEPT;
  }



void VelocityController::handle_accepted(const std::shared_ptr<GoalHandleCartesianTrajectory> goal_handle)
{
  // We get the goal from the action server
  const auto goal = goal_handle->get_goal();
  // Check if the goal is valid
  if(goal->trajectory.points.size() < 1){ //QUI
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get hold position. size() < 1");
    return;
  }

  // Interpolate the trajectory
  hold_position_msg_ptr_ = interpolate(goal->trajectory);

  // Set the goal flag to true, so that the update method can start the execution
  this->trajectory_index_ = 0;
  this->current_active_goal_ = goal_handle;
  this->active_goal_ = true;

  RCLCPP_INFO(get_node()->get_logger(), "accepted goal");
}

moveit_msgs::msg::CartesianTrajectory VelocityController::interpolate(const moveit_msgs::msg::CartesianTrajectory& trajectory)
{
  moveit_msgs::msg::CartesianTrajectory new_trajectory;
  
  // Set the new sampling interval
  double new_sampling_interval = 1.0 / this->update_rate_;

  // If the trajectory has only one point, we don't need to interpolate
  if(trajectory.points.size() == 1){
    RCLCPP_INFO(get_node()->get_logger(), "Trajectory has only one point. No interpolation needed");
    return trajectory;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Interpolating trajectory. Points: %ld", trajectory.points.size());
  RCLCPP_INFO(get_node()->get_logger(), "Interpolating trajectory. new_sampling_interval: %f", new_sampling_interval);
  
  // Iterate over the points of the trajectory
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    
    // If we are at the last point, we don't need to interpolate
    if(i==trajectory.points.size()-1){
      break;
    }

    // Get the start and end point of the trajectory
    auto start_point = trajectory.points[i];
    auto end_point = trajectory.points[i+1];
    // Get the time of the start and end point
    double t1 = start_point.time_from_start.sec + start_point.time_from_start.nanosec * 1e-9;
    double t2 = end_point.time_from_start.sec + end_point.time_from_start.nanosec * 1e-9;
    // Compute the number of samples to interpolate
    size_t num_samples = static_cast<size_t>((t2 - t1) * this->update_rate_);
  
    // Iterate over the samples
    for (size_t j = 0; j <= num_samples; ++j) {
      // Compute the time of the interpolated point
      double t = t1 + j * new_sampling_interval;

      geometry_msgs::msg::Pose interpolated_pose;
      geometry_msgs::msg::Twist interpolated_velocity;

      // Interpolate the pose and the velocity
      interpolatePoseAndVelocities(start_point.point.pose, end_point.point.pose, start_point.point.velocity, end_point.point.velocity, (t - t1) / (t2 - t1), interpolated_pose, interpolated_velocity);

      // Add the interpolated point to the new trajectory
      moveit_msgs::msg::CartesianTrajectoryPoint interpolated_point;
      interpolated_point.time_from_start = rclcpp::Duration::from_seconds(t);
      interpolated_point.point.pose = interpolated_pose;
      interpolated_point.point.velocity = interpolated_velocity;
      new_trajectory.points.push_back(interpolated_point);
    }
  }
  return new_trajectory;
}



void VelocityController::interpolatePoseAndVelocities(const geometry_msgs::msg::Pose& pose1,
                                  const geometry_msgs::msg::Pose& pose2,
                                  const geometry_msgs::msg::Twist& vel1,
                                  const geometry_msgs::msg::Twist& vel2,
                                  double alpha,
                                  geometry_msgs::msg::Pose& interpolated_pose,
                                  geometry_msgs::msg::Twist& interpolated_velocity) 
{
  // Compute the quaternion of the poses
  Eigen::Quaternion<double> rot1(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
  Eigen::Quaternion<double> rot2(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
  // Compute the translation of the poses
  Eigen::Vector3d trans1(pose1.position.x, pose1.position.y, pose1.position.z);
  Eigen::Vector3d trans2(pose2.position.x, pose2.position.y, pose2.position.z);

  // Interpolate the poses
  Eigen::Vector3d interpolated_trans = (1.0 - alpha) * trans1 + alpha * trans2;
  Eigen::Quaternion<double> interpolated_rot = rot1.slerp(alpha, rot2); // Spherical linear interpolation of the quaternions (SLERP)

  // Compute the interpolated velocities
  Eigen::Vector3d interpolated_linear_vel = (1.0 - alpha) * Eigen::Vector3d(vel1.linear.x, vel1.linear.y, vel1.linear.z) +
                                            alpha * Eigen::Vector3d(vel2.linear.x, vel2.linear.y, vel2.linear.z);
  Eigen::Vector3d interpolated_angular_vel = (1.0 - alpha) * Eigen::Vector3d(vel1.angular.x, vel1.angular.y, vel1.angular.z) +
                                              alpha * Eigen::Vector3d(vel2.angular.x, vel2.angular.y, vel2.angular.z);

  // Set the interpolated pose and velocities
  tf2::Quaternion tf2_quat;
  tf2_quat.setValue(interpolated_rot.x(), interpolated_rot.y(), interpolated_rot.z(), interpolated_rot.w());
  tf2::toMsg(tf2::Transform(tf2_quat, tf2::Vector3(interpolated_trans.x(), interpolated_trans.y(), interpolated_trans.z())),
              interpolated_pose);

  // Set the interpolated velocities
  interpolated_velocity.linear.x = interpolated_linear_vel.x();
  interpolated_velocity.linear.y = interpolated_linear_vel.y();
  interpolated_velocity.linear.z = interpolated_linear_vel.z();
  interpolated_velocity.angular.x = interpolated_angular_vel.x();
  interpolated_velocity.angular.y = interpolated_angular_vel.y();
  interpolated_velocity.angular.z = interpolated_angular_vel.z();
}



controller_interface::CallbackReturn VelocityController::on_init()
{
  // Init parameters
  RCLCPP_INFO(get_node()->get_logger(), "Initializing controller '%s'", get_node()->get_name());

  // Initialize some class attributes  
  this->joint_names_ = std::vector<std::string>();
  this->kinematic_state_ = nullptr;
  this->controller_gain_ = std::vector<double>();

  // Declaring parameters
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("joint_model_group_name", "");
  auto_declare<int>("frequency", 0);
  //auto_declare<double>("controller_gain", 0.0);
  auto_declare<std::vector<double>>("controller_gain",std::vector<double>());
  auto_declare<double>("position_tolerance", 0.0);
  auto_declare<double>("angle_tolerance", 0.0);

  return CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
VelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL; // INDIVIDUAL configuration needs a detailed list of required interface names Those are usually provided as parameters. The full interface names have structure <joint_name>/<interface_type>.
  
  if (this->joint_names_.size() == 0)
  {
    fprintf(stderr,"During ros2_control interface configuration, joints_number is not valid; it should be positive (>0). Actual joints_number is %ld\n",this->joint_names_.size());
    std::exit(EXIT_FAILURE);
  }

  // Add velocity as command interface
  conf.names.reserve(this->joint_names_.size());
  for (const auto & joint_name : this->joint_names_)
  {
    conf.names.push_back(joint_name + "/velocity");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
VelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  if (this->joint_names_.size() == 0)
  {
    fprintf(stderr,"During ros2_control interface configuration, joints_number is not valid; it should be positive (>0). Actual joints_number is %ld\n",this->joint_names_.size());
    std::exit(EXIT_FAILURE);
  }

  // Add position and velocity as state interfaces
  conf.names.reserve(2*this->joint_names_.size());
  for (const auto & joint_name : this->joint_names_)
  {
    conf.names.push_back(joint_name + "/position");
    conf.names.push_back(joint_name + "/velocity");
  }
  return conf;
}



controller_interface::CallbackReturn VelocityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  hold_position_msg_ptr_ = moveit_msgs::msg::CartesianTrajectory();


  RCLCPP_INFO(logger, "Configuring controller '%s'", get_node()->get_name());
  
  // Get parameters and check if they are valid    
  if( !get_node()->get_parameter("controller_gain", this->controller_gain_)
      || !get_node()->get_parameter("joint_model_group_name", this->joint_model_group_name_)
      || !get_node()->get_parameter("frequency", this->update_rate_)
      || !get_node()->get_parameter("position_tolerance", this->position_tolerance_)
      || !get_node()->get_parameter("angle_tolerance", this->angle_tolerance_)
      ){

    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure controller '%s'", get_node()->get_name());
    return CallbackReturn::ERROR;
  }

  //DEBUG INFO
  RCLCPP_INFO(get_node()->get_logger(), "joint_model_group_name: '%s'", this->joint_model_group_name_.c_str());
  //RCLCPP_INFO(get_node()->get_logger(), "controller_gain: '%f'", this->controller_gain_);
  for(const auto& gain : this->controller_gain_) {
    RCLCPP_INFO(get_node()->get_logger(), "controller_gain: '%f'", gain);
  }
  RCLCPP_INFO(get_node()->get_logger(), "update_rate: '%d'", this->update_rate_);
  RCLCPP_INFO(get_node()->get_logger(), "position_tolerance: '%f'", this->position_tolerance_);
  RCLCPP_INFO(get_node()->get_logger(), "angle_tolerance: '%f'", this->angle_tolerance_);

  // Load the robot model
  // Note that std::make_shared<robot_model_loader::RobotModelLoader>() requires a node to be passed as argument that is not a lifecycle node. So we create a temporary node to load the robot model and then we destroy it.
  rclcpp::Node::SharedPtr temp_node = std::make_shared<rclcpp::Node>("robot_model_loader");

  bool keep_alive = true;
  std::thread t([&temp_node, &keep_alive, this]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(temp_node);
    RCLCPP_INFO(get_node()->get_logger(), "Starting node to load robot model");

    while (keep_alive)
    {
      executor.spin_some(50ms);
    }
    RCLCPP_INFO(get_node()->get_logger(), "Node to load robot model stopped");
  });

  // Spawn slave node to load robot sodel
  this->robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(temp_node,"robot_description",true);
  this->kinematic_model_ = this->robot_model_loader_->getModel();

  if(this->kinematic_model_ == nullptr){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load robot model");
    keep_alive=false;
    t.join();
    return CallbackReturn::ERROR;
  }
  keep_alive=false;
  t.join();
  RCLCPP_INFO(get_node()->get_logger(), "Robot model loaded");

  // Print the model frame to see if it is correct
  RCLCPP_INFO(get_node()->get_logger(), "Model frame: %s", this->kinematic_model_->getModelFrame().c_str());

  // Create a kinematic state - this represents the configuration for the robot represented by kinematic_model
  this->kinematic_state_ = std::make_shared<moveit::core::RobotState>(this->kinematic_model_);
  
  if(this->kinematic_state_ == nullptr){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create kinematic state");
    return CallbackReturn::ERROR;
  }

  // Get the joint model group
  this->joint_model_group_ = this->kinematic_model_->getJointModelGroup(this->joint_model_group_name_);

  if(this->joint_model_group_ == nullptr){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get joint model group");
    return CallbackReturn::ERROR;
  }else{
    RCLCPP_INFO(get_node()->get_logger(), "Joint model group: %s", this->joint_model_group_->getName().c_str());
  }


  // Get the joint names of the model group of the robot
  this->joint_names_ = joint_model_group_->getVariableNames();

  // Print the joint names
  RCLCPP_INFO(get_node()->get_logger(), "The joint are: ");
  for(const auto& str : this->joint_names_) {
    RCLCPP_INFO(get_node()->get_logger(), "joint: '%s'", str.c_str());
  }

  // Create the action server
  RCLCPP_INFO(get_node()->get_logger(), "Creating action server");
  using namespace std::placeholders;
  this->action_server_ = rclcpp_action::create_server<ActionCartesianTrajectory>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_cartesian_trajectory", //TODO: Implement a parameter to set the action name
    std::bind(&VelocityController::handle_goal, this, _1, _2),
    std::bind(&VelocityController::handle_cancel, this, _1),
    std::bind(&VelocityController::handle_accepted, this, _1));
  
  RCLCPP_INFO(get_node()->get_logger(), "Action server created");

  return CallbackReturn::SUCCESS;
}



controller_interface::return_type VelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  
  // Check if there is an active goal to execute. If not, set all joints velocities to 0 and return.
  if(this->active_goal_){
    
    geometry_msgs::msg::TransformStamped error = geometry_msgs::msg::TransformStamped();

    // Get the actual position and velocities of the joints
    std::vector<double> actual_position= std::vector<double>(this->joint_names_.size(), 0.0);
    std::vector<double> actual_velocities= std::vector<double>(this->joint_names_.size(), 0.0);

    // Get the reference pose from the trajectory
    this->desired_state_ = this->hold_position_msg_ptr_.points[this->trajectory_index_];
    
    // Get the actual position and velocities of the joints, through the state interfaces
    for(size_t i = 0; i < this->joint_names_.size(); i++){
      actual_position[i] = this->joint_state_interface_position_[i].get().get_value();
      actual_velocities[i] = this->joint_state_interface_velocity_[i].get().get_value();
    }

    // Get the actual pose of the end effector
    this->kinematic_state_->setJointGroupPositions(this->joint_model_group_, actual_position);
    auto isometry2 = this->kinematic_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());
    auto actual_pose = tf2::eigenToTransform(isometry2);

    // Compute the error between the desired pose and the actual pose
    error.transform.translation.x = this->desired_state_.point.pose.position.x - actual_pose.transform.translation.x;
    error.transform.translation.y = this->desired_state_.point.pose.position.y - actual_pose.transform.translation.y;
    error.transform.translation.z = this->desired_state_.point.pose.position.z - actual_pose.transform.translation.z;
    
    tf2::Quaternion actual_quaternion;
    tf2::fromMsg(actual_pose.transform.rotation, actual_quaternion);

    tf2::Quaternion desired_quaternion = tf2::Quaternion();

 

    desired_quaternion.setX(this->desired_state_.point.pose.orientation.x);
    desired_quaternion.setY(this->desired_state_.point.pose.orientation.y);
    desired_quaternion.setZ(this->desired_state_.point.pose.orientation.z);
    desired_quaternion.setW(this->desired_state_.point.pose.orientation.w);

    // We want concord quaternions, they are manually changed 
    if(actual_quaternion.dot(desired_quaternion) < 0){
      desired_quaternion.setX(-desired_quaternion.x());
      desired_quaternion.setY(-desired_quaternion.y());
      desired_quaternion.setZ(-desired_quaternion.z());
      desired_quaternion.setW(-desired_quaternion.w());
    }


    // Now, we publish the feedback. Note that the feedback is published only if the trajectory index is greater than 0, because the feedback values are: actual pose of the current iteration and desired pose of the previous iteration. So, if the trajectory index is 0, there is no previous iteration.
    if(this->trajectory_index_ > 0){

      // Compute the jacobian of the velocity
      Eigen::MatrixXd jacobian_velocities;
      if(!this->kinematic_state_->getJacobian(this->joint_model_group_, this->kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()), Eigen::Vector3d(0.0, 0.0, 0.0), jacobian_velocities)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get jacobian");
        return controller_interface::return_type::ERROR;
      }
      // Compute the joint velocities v = J(q)*q_dot
      Eigen::VectorXd velocities = jacobian_velocities * Eigen::VectorXd::Map(actual_velocities.data(), actual_velocities.size());

      
      // One of the message fields is the actual pose of the end effector. This is computed using the actual pose of the end effector and the joint velocities.
      moveit_msgs::msg::CartesianTrajectoryPoint current_state;
      current_state.point.pose.position.x = actual_pose.transform.translation.x;
      current_state.point.pose.position.y = actual_pose.transform.translation.y;
      current_state.point.pose.position.z = actual_pose.transform.translation.z;
      
      // Before setting the orientation in the feedback, we need check the quaternion to see if they are concoherent. If they are not, we need to invert the quaternion.
      Eigen::Quaterniond q(actual_pose.transform.rotation.w, 
                     actual_pose.transform.rotation.x, 
                     actual_pose.transform.rotation.y, 
                     actual_pose.transform.rotation.z);

      auto& point = this->hold_position_msg_ptr_.points[this->trajectory_index_ -1];
      Eigen::Quaterniond desired_quaternion(point.point.pose.orientation.w,
                          point.point.pose.orientation.x, 
                          point.point.pose.orientation.y, 
                          point.point.pose.orientation.z);

      // We want concord quaternions, they are manually changed 

      if(q.dot(desired_quaternion) < 0){
        q.x() = -q.x();
        q.y() = -q.y();
        q.z() = -q.z();
        q.w() = -q.w();
      }

      // Copying the normalized quaternion to the current state feedback message
      current_state.point.pose.orientation.x = q.x();
      current_state.point.pose.orientation.y = q.y();
      current_state.point.pose.orientation.z = q.z();
      current_state.point.pose.orientation.w = q.w();
      // Setting the velocities
      current_state.point.velocity.linear.x = velocities(0, 0);
      current_state.point.velocity.linear.y = velocities(1, 0);
      current_state.point.velocity.linear.z = velocities(2, 0);
      current_state.point.velocity.angular.x = velocities(3, 0);
      current_state.point.velocity.angular.y = velocities(4, 0);
      current_state.point.velocity.angular.z = velocities(5, 0);
      
      moveit_msgs::msg::CartesianTrajectoryPoint error_state;
      error_state.point.pose.position.x = point.point.pose.position.x - actual_pose.transform.translation.x;
      error_state.point.pose.position.y = point.point.pose.position.y - actual_pose.transform.translation.y;
      error_state.point.pose.position.z = point.point.pose.position.z - actual_pose.transform.translation.z;
      error_state.point.pose.orientation.x = point.point.pose.orientation.x -  current_state.point.pose.orientation.x;
      error_state.point.pose.orientation.y = point.point.pose.orientation.y - current_state.point.pose.orientation.y;
      error_state.point.pose.orientation.z = point.point.pose.orientation.z - current_state.point.pose.orientation.z;
      error_state.point.pose.orientation.w = point.point.pose.orientation.w - current_state.point.pose.orientation.w;
      error_state.point.velocity.linear.x = point.point.velocity.linear.x - current_state.point.velocity.linear.x;
      error_state.point.velocity.linear.y = point.point.velocity.linear.y - current_state.point.velocity.linear.y;
      error_state.point.velocity.linear.z = point.point.velocity.linear.z - current_state.point.velocity.linear.z;
      error_state.point.velocity.angular.x = point.point.velocity.angular.x - current_state.point.velocity.angular.x;
      error_state.point.velocity.angular.y = point.point.velocity.angular.y - current_state.point.velocity.angular.y;
      error_state.point.velocity.angular.z = point.point.velocity.angular.z - current_state.point.velocity.angular.z;



      // Create the feedback message 
      auto feedback = std::make_shared<ActionCartesianTrajectory::Feedback>();
      feedback->actual = current_state;
      feedback->desired = point;
      feedback->error = error_state;
      // Publish the feedback
      this->current_active_goal_->publish_feedback(feedback); 
    }

    // Check orientation tolerance using quaternions
    double angle_error = tf2::angle(actual_quaternion, desired_quaternion);

    // Check position tolerance
    double position_error = sqrt(pow(actual_pose.transform.translation.x - desired_state_.point.pose.position.x, 2) +
                                 pow(actual_pose.transform.translation.y - desired_state_.point.pose.position.y, 2) +
                                 pow(actual_pose.transform.translation.z - desired_state_.point.pose.position.z, 2));
    


    // After computing the error and published the feedback, we do an additional check only to the first point of the trajectory. We check if the actual pose of the end effector is too far from the desired pose (according to a postion and angle tolerance). If it is too far, we abort the goal.
    auto result = std::make_shared<ActionCartesianTrajectory::Result>();
    if (this->trajectory_index_ == 0 && (position_error > this->position_tolerance_ || angle_error > this->angle_tolerance_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Aborting trajectory. Position error: %f, Angle error: %f, tolerance: %f, angle_tolerance: %f", position_error, angle_error, this->position_tolerance_, this->angle_tolerance_);
      this->active_goal_ = false;
      result->error_code = -2;//ActionCartesianTrajectory::INVALID_POSE;
      this->current_active_goal_->abort(result);
      return controller_interface::return_type::ERROR;
    }
    // Following the CLIK control law, using the simplifaction based on unit quaternions and geometric jacobian, we compute the joint velocities to reach the desired pose, the formula is:
    // q_dot = J_d^+(q)*[pd_dot + Kp*ep; wd + Ko*eo] where 
    // eo = eta_d(q)*epsilon_e(q) - eta_e(q)*epsilon_d(q) - S(epsilon_d(q))*epsilon_e(q)
    // and the S operator is the skew matrix of a vector, and the eta and epsilon are the scalar and vector part of the quaternion.

    double eta_d = desired_quaternion.w();  
    tf2::Vector3 epsilon_d(desired_quaternion.x(), desired_quaternion.y(), desired_quaternion.z());  
    double eta_e = actual_quaternion.w();  
    tf2::Vector3 epsilon_e(actual_quaternion.x(), actual_quaternion.y(), actual_quaternion.z());  

    tf2::Vector3 term1 = eta_e * epsilon_d;

    tf2::Vector3 term2 = eta_d * epsilon_e;

    tf2::Matrix3x3 skew_matrix_d;
    skew_matrix_d.setValue(0, -epsilon_d.z(), epsilon_d.y(),
                            epsilon_d.z(), 0, -epsilon_d.x(),
                            -epsilon_d.y(), epsilon_d.x(), 0);

    tf2::Vector3 term3 = skew_matrix_d * epsilon_e;

    tf2::Vector3 orientation_error = term1 - term2 - term3;

    // Create the error vector
    Eigen::MatrixXd error_vector = Eigen::MatrixXd::Zero(6, 1);
    error_vector(0, 0) = error.transform.translation.x; // remember that the "error" is the difference between the desired pose and the actual pose
    error_vector(1, 0) = error.transform.translation.y;
    error_vector(2, 0) = error.transform.translation.z;
    error_vector(3, 0) = orientation_error.x();
    error_vector(4, 0) = orientation_error.y();
    error_vector(5, 0) = orientation_error.z();
    
    // Only now we can compute the jacobian
    Eigen::MatrixXd jacobian;
    if(!this->kinematic_state_->getJacobian(this->joint_model_group_, this->kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()), Eigen::Vector3d(0.0, 0.0, 0.0), jacobian)){
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get jacobian");
      return controller_interface::return_type::ERROR;
    }

    // Compute the pseudo inverse of the jacobian at min quads

    Eigen::MatrixXd jacobian_min_quad;
    double epsilon = 0.01; // epsilon value to avoid infinite velocities in singularities
    Eigen::MatrixXd jacobianT = jacobian.transpose();
    Eigen::MatrixXd jacobian2 = jacobian * jacobianT;
    jacobian_min_quad = jacobianT * (jacobian2 + epsilon * Eigen::MatrixXd::Identity(jacobian2.rows(), jacobian2.cols())).inverse();

    // Create the velocities vector
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(6, 1);
    velocities(0, 0) = this->desired_state_.point.velocity.linear.x;
    velocities(1, 0) = this->desired_state_.point.velocity.linear.y;
    velocities(2, 0) = this->desired_state_.point.velocity.linear.z;
    velocities(3, 0) = this->desired_state_.point.velocity.angular.x;
    velocities(4, 0) = this->desired_state_.point.velocity.angular.y;
    velocities(5, 0) = this->desired_state_.point.velocity.angular.z;

    // CONTROL LAW
    // Compute the joint velocities, using the jacobian of the position and the error vector.
    //Create a diagonal matrix with the controller gain
    Eigen::MatrixXd gain_vector_ = Eigen::MatrixXd::Zero(this->controller_gain_.size(), 1);
    for(size_t i = 0; i < this->controller_gain_.size(); i++){
      gain_vector_(i, 0) = this->controller_gain_.at(i);
    }
    Eigen::MatrixXd joint_velocities = jacobian_min_quad * (gain_vector_.asDiagonal()*error_vector) + jacobian_min_quad * (velocities); // where error_vector cotains ep and eo, and velocities contains pd_dot and wd and gain_vector_ contains Kp and Ko

    // Set the joint velocities. 
    for(size_t i = 0; i < this->joint_names_.size(); i++){
      this->joint_command_interface_[i].get().set_value(joint_velocities(i, 0));
    }

    // update the trajectory index
    this->trajectory_index_++;

    // Check if the trajectory is completed
    if(this->trajectory_index_ >= (int)this->hold_position_msg_ptr_.points.size()){
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory completed");
      result->error_code = 0; //this is equivalent to ActionCartesianTrajectory::SUCCESSFUL;
      this->current_active_goal_->succeed(result);
      RCLCPP_INFO(get_node()->get_logger(), "Goal succeeded");      

      this->trajectory_index_ = 0;
      this->active_goal_ = false;
    }

  }else{ // If there is no active goal, we set all joints velocities to 0, so even if the trajectory is canceled, the robot will stop
    // Setting all joints velocities to 0
    for(size_t i = 0; i < this->joint_names_.size(); i++){
      this->joint_command_interface_[i].get().set_value(0.0);
    }
  }

  return controller_interface::return_type::OK;
}



controller_interface::CallbackReturn VelocityController::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating controller '%s'", get_node()->get_name());
  
  // Get the command interfaces
  if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, this->joint_names_, "velocity", this->joint_command_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu velocity command interfaces, got %zu.", this->joint_names_.size(),
       this->joint_command_interface_.size());
    return CallbackReturn::ERROR;
  }

  // Get the position state interfaces of the joints
  if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, this->joint_names_, "position", this->joint_state_interface_position_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position state interfaces, got %zu.", this->joint_names_.size(),
       this->joint_state_interface_position_.size());
    return CallbackReturn::ERROR;
  }

  // Get the velocity state interfaces
  if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, this->joint_names_, "velocity", this->joint_state_interface_velocity_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu velocity state interfaces, got %zu.", this->joint_names_.size(),
       this->joint_state_interface_velocity_.size());
    return CallbackReturn::ERROR;
  }
  
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_deactivate(
  const rclcpp_lifecycle::State &)
{

  // If the server is working on a goal, abort it
  if(this->active_goal_){
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller ... Aborting active goal");
    this->active_goal_ = false;
    this->trajectory_index_ = 0;
    auto result = std::make_shared<ActionCartesianTrajectory::Result>();
    result->error_code = -1;//ActionCartesianTrajectory::ABORTED;
    this->current_active_goal_->abort(result);
  }else{
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller ...");
  }

  // Clear the command interfaces
  this->joint_command_interface_.clear();
  // Clear the state interfaces
  this->joint_state_interface_position_.clear();

  return CallbackReturn::SUCCESS;
}


}  // namespace velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controller::VelocityController, controller_interface::ControllerInterface)


