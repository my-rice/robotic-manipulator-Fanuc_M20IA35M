/** -------------------------------------------------------------------
 * 
 * @file:   action_node.hpp
 * @author:  Group 2 Robotics Project 2023/2024 UNISA
 * @date:    Feb 11, 2024
 *
 * @brief This node is used to execute the action server for the Cartesian trajectory and the joint trajectory
 * and to store the trajectory in a bag file.
 * 
 * -------------------------------------------------------------------
 */
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_interfaces/srv/set_trajectory_start.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <filesystem>
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "action_cartesian_trajectory/action/follow_cartesian_trajectory.hpp"
#include <rosbag2_cpp/writer.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"

// Action node class definition
namespace fanuc_m20ia_35m_planning_demo
{
class ActionNode : public rclcpp::Node
{
public:
ActionNode();
/**
   * @brief This attribute is used to store the action client for the Cartesian trajectory
   */
rclcpp_action::Client<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SharedPtr client_action; 
/**
   * @brief This attribute is used to store the options for sending the goal to the action server
*/
rclcpp_action::Client<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SendGoalOptions send_goal_options; 
/**
   * @brief This function is used as a callback for the goal response received by the action server
*/
void goal_response_callback(const rclcpp_action::ClientGoalHandle<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SharedPtr & goal_handle); 
/**
   * @brief This function is used as a callback for the feedback received by the action server
*/
void feedback_callback(rclcpp_action::ClientGoalHandle<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SharedPtr, const std::shared_ptr<const action_cartesian_trajectory::action::FollowCartesianTrajectory::Feedback> feedback); 
/**
   * @brief This function is used as a callback for the result received by the action server
*/
void result_callback(const rclcpp_action::ClientGoalHandle<action_cartesian_trajectory::action::FollowCartesianTrajectory>::WrappedResult & result); 

/**
   * @brief This attribute is used to store the action client for the joint trajectory
*/
rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joint_client_action; 
/**
   * @brief This attribute is used to store the options for sending the goal to the action server
*/
rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions joint_send_goal_options;  
/**
   * @brief This function is used as a callback for the goal response received by the action server
*/
void joint_goal_response_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle); 
/**
   * @brief This function is used as a callback for the feedback received by the action server
*/
void joint_feedback_callback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback); 
/**
   * @brief This function is used as a callback for the result received by the action server
*/
void joint_result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result); // Callback for the result

// Useful functions
/**
   * @brief This function is used to set the filename for the bag file
   */
bool setFilename(std::string filename, bool work_space); 
/**
   * @brief This function is used to return the filename for the bag file
   */
std::string getFilename(); 
/**
   * @brief This function is used to set the flag for storing the trajectory
   */
bool setStoreTrajectory(bool store_trajectory, std::string filename, bool work_space); 

/**
   * @brief This attribute is used to store the promise for the future
   */
std::promise<bool> promise; 
/**
   * @brief This attribute is used to store the future for the promise
   */
std::future<bool> future; 

private:
std::unique_ptr<rosbag2_cpp::Writer> writer_; // Writer for the actual trajectory
std::unique_ptr<rosbag2_cpp::Writer> secondary_writer_; // Writer for the desired trajectory
std::filesystem::path currentPath; // Path for the current directory
std::string filename_; // Filename for the bag file
bool store_trajectory_; // Flag for storing the trajectory 
sensor_msgs::msg::JointState joint_state_msg_; // Message for the joint state


};

}  // namespace fanuc_m20ia_35m_planning_demo