/** -------------------------------------------------------------------
 * 
 * @file:   velocity_controller.hpp
 * @author:  Group 2 Robotics Project 2023/2024 UNISA
 * @date:    Gen 20, 2024
 *
 * @brief This lifecycle node implements a velocity controller.
 * The methods on_init(), on_configure(), on_activate(), on_deactivate() are
 * called by the Controller Manager when the controller is loaded, configured,
 * activated and deactivated, respectively. The Controller Manager will 
 * load this controller and will call the update() method at the
 * desired frequency, specified in the configuration file of the controller. 
 * The update() method will use the desired pose together with the desired
 * velocity, then it will compute the pseudo-inverse of the jacobian matrix at min quad,
 * the error between the current pose and the desired pose multiplied by the
 * controller gain matrix. In this way, the desired joint velocities are computed.
 * Then the desired joint velocities are sent to the hardware interface. 
 * The hardware interface will send the desired joint velocities to the
 * robot. The robot will move according to the desired joint velocities.
 * The controller, starts an action server. The action server will receive by the action client
 * a trajectory composed by a sequence of poses and velocities. The controller
 * will interpolate the trajectory at the desired frequency and will send the
 * feedback to the action client. 
 * 
 * The controller gain matrix is specified in the configuration file of the controller.
 * 
 * -------------------------------------------------------------------
 */

#ifndef VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_

#include <chrono>
#include <functional> 
#include <memory>
#include <string>
#include <vector>

#include "action_cartesian_trajectory/action/follow_cartesian_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/robot_state/robot_state.h>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "task_space_velocity_controller/visibility_controller.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_server_goal_handle.h"

#include <controller_manager/controller_manager.hpp>

using ActionCartesianTrajectory = action_cartesian_trajectory::action::FollowCartesianTrajectory;
using GoalHandleCartesianTrajectory = rclcpp_action::ServerGoalHandle<ActionCartesianTrajectory>;

using namespace std::chrono_literals;  

namespace velocity_controller
{
/** @class VelocityController
 * @brief This class implements a velocity controller.
  */
class VelocityController : public controller_interface::ControllerInterface
{
public:
  
  /**
   * @brief This constructor initializes the controller.
  */
  VELOCITY_CONTROLLER_PUBLIC
  VelocityController();

  /**
   * @brief This method configure the command interface.
   */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief This method configure the state interface.
   */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief This method is called by the controller manager. 
   * It computes the joint velocities and sends them to the hardware interface, 
   * which are calculated according to a control law.
   */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  /**
   * @brief This method is called by the controller manager. It initializes the controller, 
   * specifically it initializes the variables, reserve memory, and most importantly, 
   * declare node parameters used by the controller. 
   */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief This method is called by the controller manager. 
   * It configures the controller, in particular controller parameters are read,
   * variables are configured and the action server is started.
  */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief This method is called by the controller manager. It activates the controller.
  */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief This method is called by the controller manager. It deactivates the controller.
  */
  VELOCITY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


protected:
  /**
   * @brief This is the instance of the action server.
  */
  rclcpp_action::Server<ActionCartesianTrajectory>::SharedPtr action_server_;
  
  /**
   * @brief This method is called by the action server. It handles the goal.
  */
  VELOCITY_CONTROLLER_PUBLIC
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionCartesianTrajectory::Goal> goal);
  
  /**
   * @brief This method is called by the action server. It handles the cancel request.
  */
  VELOCITY_CONTROLLER_PUBLIC
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCartesianTrajectory> goal_handle);

  /**
   * @brief This method is called by the action server. It handles the accepted request.
  */
  VELOCITY_CONTROLLER_PUBLIC
  void handle_accepted(const std::shared_ptr<GoalHandleCartesianTrajectory> goal_handle);

  /**
   * @brief This attribute is the current desired pose of the robot, which will be the reference for the controller.
  */
  moveit_msgs::msg::CartesianTrajectoryPoint desired_state_;

  /**
   * @brief This attribute stores the desired trajectory received by the action client.
  */
  moveit_msgs::msg::CartesianTrajectory hold_position_msg_ptr_;

  /**
   * @brief This attribute is a flag that indicates if there is a goal to be elaborated by the action server.
  */
  bool active_goal_ = false;

  /**
   * @brief This attribute is the current goal of the action server received by the action client. 
  */
  std::shared_ptr<GoalHandleCartesianTrajectory> current_active_goal_;
  
  /**
   * @brief This method is an helper of the interpolate() method. It interpolates the pose and the velocities.
  */
  void interpolatePoseAndVelocities(const geometry_msgs::msg::Pose& pose1,
                                  const geometry_msgs::msg::Pose& pose2,
                                  const geometry_msgs::msg::Twist& vel1,
                                  const geometry_msgs::msg::Twist& vel2,
                                  double alpha,
                                  geometry_msgs::msg::Pose& interpolated_pose,
                                  geometry_msgs::msg::Twist& interpolated_velocity);

  /**
   * @brief This method is called by the controller. It interpolates the trajectory.
  */
  moveit_msgs::msg::CartesianTrajectory interpolate(const moveit_msgs::msg::CartesianTrajectory& trajectory);


private:
 
  /**
   * @brief This attribute store the name of the joints of the model group; these names are computed dynamically.
  */
  std::vector<std::string> joint_names_;

  /**
   * @brief This attribute is the controller gain, loaded from the configuration file.
  */
  //double controller_gain_;
  std::vector<double> controller_gain_;

  //std::string robot_description_;

  /**
   * @brief This attribute is the robot model loader.
  */
  std::shared_ptr<const robot_model_loader::RobotModelLoader> robot_model_loader_;

  /**
   * @brief This attribute is the robot model.
  */
  moveit::core::RobotModelPtr kinematic_model_;

  /**
   * @brief This attribute is the current pose of the robot.
  */
  std::shared_ptr<moveit::core::RobotState> kinematic_state_; 
  /**
   * @brief This attribute is the joint model group name.
  */
  std::string joint_model_group_name_;
  /**
   * @brief This attribute is the joint model group.
  */
  moveit::core::JointModelGroup* joint_model_group_;


  // Eigen::Isometry3d world_to_end_effector_transform_;

  /**
   * @brief This attribute is index of the current pose in the trajectory.
  */
  int trajectory_index_ = 0; 
  /**
   * @brief This attribute is the joint command interface.
  */
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interface_;
  /**
   * @brief This attribute is the joint state interface for the position.
  */
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interface_position_;
  /**
   * @brief This attribute is the joint state interface for the velocity.
  */
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interface_velocity_;

  /**
   * @brief This attribute store the update rate of the controller manager, loaded from the configuration file.
  */
  int update_rate_;
  /**
   * @brief This attribute store the tolerance of the distance tolerance between the current pose and the desired pose, loaded from the configuration file. 
  */
  double position_tolerance_;      // in meters
  /**
   * @brief This attribute store the tolerance of the angle tolerance between the current pose and the desired pose, loaded from the configuration file. 
  */
  double angle_tolerance_; // in radians. Angle tolerance between two quaternions. 
  
};
}  // namespace velocity_controller

#endif  // VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_
