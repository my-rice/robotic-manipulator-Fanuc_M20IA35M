/** -------------------------------------------------------------------
 * 
 * @file:   action_node.cpp
 * @author:  Group 2 Robotics Project 2023/2024 UNISA
 * @date:    Feb 11, 2024
 *
 * @brief This node is used to execute the action server for the Cartesian trajectory and the joint trajectory
 * and to store the trajectory in a bag file. For further details, please refer to the action_node.hpp file.
 * 
 * -------------------------------------------------------------------
 */
#include <action_node.hpp>
// Action node class implementation 
using namespace fanuc_m20ia_35m_planning_demo;

ActionNode::ActionNode() : Node("action_node")
{

  RCLCPP_INFO(this->get_logger(), "Action server started.");
  currentPath = std::filesystem::current_path();
}  // namespace fanuc_m20ia_35m_planning_demo

bool ActionNode::setFilename(std::string filename, bool work_space){
  RCLCPP_INFO(this->get_logger(), "Setting filename");
  this->filename_=filename; // Set the filename for the bag file
  std::string bagFilePath = currentPath.string() + "/matlab_scripts/results_bagfiles/"; // Set the path for the bag file
  std::string bagFileName = "real_trajectories/Real" + this->filename_; // Set the entire name for the bag file that will store the actual trajectory
  std::string bagFileSecondaryName = "desired_trajectories/Desired" + this->filename_; // Set the entire name for the bag file that will store the desired trajectory
  try{ // Try to create the bag files
    writer_ = std::make_unique<rosbag2_cpp::Writer>(); // Create a writer for the actual trajectory
    secondary_writer_ = std::make_unique<rosbag2_cpp::Writer>(); // Create a writer for the desired trajectory
    writer_->open(bagFilePath + bagFileName); // Open the bag file for the actual trajectory

    secondary_writer_->open(bagFilePath + bagFileSecondaryName); // Open the bag file for the desired trajectory
    if(work_space){ 
      writer_->create_topic({"poses", "geometry_msgs/msg/Pose",  rmw_get_serialization_format(), ""}); // Create a topic for the actual trajectory, if the task space is used, the topic will be "poses"
      secondary_writer_->create_topic({"poses", "geometry_msgs/msg/Pose",  rmw_get_serialization_format(), ""}); // Create a topic for the desired trajectory, if the task space is used, the topic will be "poses"
    }
    else{ 
      writer_->create_topic({"joint_states", "sensor_msgs/msg/JointState",  rmw_get_serialization_format(), ""}); // Create a topic for the actual trajectory, if the joint space is used, the topic will be "joint_states"
      secondary_writer_->create_topic({"joint_states", "sensor_msgs/msg/JointState",  rmw_get_serialization_format(), ""}); // Create a topic for the desired trajectory, if the joint space is used, the topic will be "joint_states"
    }
  }
  catch(std::exception& e){ // If an error occurs, print an error message
    RCLCPP_ERROR(this->get_logger(), "Error while opening the bag file");
    return false;

  }

  return true;

}

bool ActionNode::setStoreTrajectory(bool store_trajectory, std::string filename, bool work_space){
  if(store_trajectory){ // If the trajectory has to be stored, set the filename and create the bag files
    if(!setFilename(filename, work_space)){
      RCLCPP_ERROR(this->get_logger(), "Error while setting the filename");
      return false;
    }
  }
  this->store_trajectory_=store_trajectory; // Set the flag for storing the trajectory
  return true;
}

std::string ActionNode::getFilename(){
  return this->filename_; // Return the filename
}

void ActionNode::goal_response_callback(const rclcpp_action::ClientGoalHandle<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server"); // If the goal was rejected by the server, print an error message
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result"); // If the goal was accepted by the server, print a message
    }
  }


void ActionNode::feedback_callback(rclcpp_action::ClientGoalHandle<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SharedPtr, const std::shared_ptr<const action_cartesian_trajectory::action::FollowCartesianTrajectory::Feedback> feedback)
{
  if(this->store_trajectory_){ // If the trajectory has to be stored, write the actual and desired poses in the bag files
    auto desired_pose = feedback->desired; // Get the desired pose
    auto actual_pose = feedback->actual; // Get the actual pose
    try{ // Try to write the poses in the bag files
    writer_->write(desired_pose, "poses", this->now()); // Write the desired pose in the bag file for the actual trajectory
    secondary_writer_->write(actual_pose, "poses", this->now()); // Write the actual pose in the bagfile for the desired trajectory
    }
    catch(std::exception& e){ // If an error occurs, print an error message
      RCLCPP_ERROR(this->get_logger(), "Error while writing the bag file");
    }
  }
}


void ActionNode::result_callback(const rclcpp_action::ClientGoalHandle<action_cartesian_trajectory::action::FollowCartesianTrajectory>::WrappedResult & result)
{
  switch (result.code) { // Check the result code of the action server and print the corresponding message, then set the value of the promise to true or false depending on the result code
    case rclcpp_action::ResultCode::SUCCEEDED:
      promise.set_value(true);
      
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      promise.set_value(false);
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      promise.set_value(false);
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      promise.set_value(false);
      return;
  }
  if(this->store_trajectory_){ // If the trajectory was stored, close the bag files 
    writer_->close();
    secondary_writer_->close();
  }
  
  RCLCPP_INFO(this->get_logger(), "Goal executed by the server");
}

void ActionNode::joint_goal_response_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle){
 if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server"); // If the goal was rejected by the server, print an error message
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result"); // If the goal was accepted by the server, print a message
    }

}
void ActionNode::joint_feedback_callback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback){
   if(this->store_trajectory_){ // If the trajectory has to be stored, write the actual and desired joint positions in the bag files
    auto desired_joint_positions = feedback->desired.positions; // Get the desired joint positions
    auto actual_joint_positions = feedback->actual.positions; // Get the actual joint positions
    try{ // Try to write the joint positions in the bag files
    joint_state_msg_.position = desired_joint_positions; // Set the desired joint positions in the message
    writer_->write(this->joint_state_msg_, "joint_states", this->now()); // Write the desired joint positions in the bag file for the actual trajectory
    joint_state_msg_.position = actual_joint_positions; // Set the actual joint positions in the message 
    secondary_writer_->write(this->joint_state_msg_, "joint_states", this->now()); // Write the actual joint positions in the bagfile for the desired trajectory
    }
    catch(std::exception& e){ // If an error occurs, print an error message
      RCLCPP_ERROR(this->get_logger(), "Error while writing the bag file");
    }
  }
}
void ActionNode::joint_result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result){
  switch (result.code) { // Check the result code of the action server and print the corresponding message, then set the value of the promise to true or false depending on the result code
      case rclcpp_action::ResultCode::SUCCEEDED:
        promise.set_value(true);
        
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        promise.set_value(false);
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        promise.set_value(false);
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        promise.set_value(false);
        return;
    }
    if(this->store_trajectory_){ // If the trajectory was stored, close the bag files 
      writer_->close();
      secondary_writer_->close();
    }
    
    RCLCPP_INFO(this->get_logger(), "Goal executed by the server");
}

