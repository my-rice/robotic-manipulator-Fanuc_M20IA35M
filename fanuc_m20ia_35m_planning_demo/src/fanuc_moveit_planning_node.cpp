/** -------------------------------------------------------------------
 * 
 * @file:   fanuc_moveit_planning_node.cpp
 * @author:  Group 2 Robotics Project 2023/2024 UNISA
 * @date:    Feb 7, 2024
 *
 * @brief This node is used to plan the trajectory for the robot, it is robot
 * agnostic and can be used with any robot that has a MoveIt configuration
 * package. The node is used to plan the trajectory for the robot, it can
 * plan the trajectory in the joint space or in the task space (cartesian space) and 
 * this can be chosen by the user at runtime. The node can also execute the trajectory
 * in the two spaces, using the joint action server or the cartesian action server,
 * and store the trajectory executed in a bagfile to be used for analysis. 
 * The planner used is the Pilz industrial motion planner, that is a plugin for
 * MoveIt and it is used to plan the trajectory, this is preferred to the default
 * planner of MoveIt because it is more suitable for the type of trajectory that
 * the robot has to execute, considering the action space of the robot and the
 * size of the end effector. More information about this choices can be found in the
 * documentation of the project.
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

#include <action_node.hpp>


/** @class Fanuc_m20ia_35m_PlanningDemoClient
 * @brief Class for the planning demo client
  */
class Fanuc_m20ia_35m_PlanningDemoClient : public rclcpp::Node
{
public:
  Fanuc_m20ia_35m_PlanningDemoClient()
  : Node("fanuc_m20ia_35m_planning_demo_client") 
  {
    // Declare parameters for the joint model group name, base link, end effector link, continuous planning, work space and controller action
    this->declare_parameter<std::string>("joint_model_group_name"); 
    this->declare_parameter<std::string>("base_link");
    this->declare_parameter<std::string>("end_effector_link");
    this->declare_parameter<int>("continuos_planning");
    this->declare_parameter<int>("work_space");
    this->declare_parameter<std::string>("controller_action");
    this->declare_parameter<std::string>("trajectory_name");
    if(this->get_parameter("joint_model_group_name", this->joint_model_group_name_) && this->get_parameter("base_link", this->base_link_) && this->get_parameter("end_effector_link", this->end_effector_link_) && this->get_parameter("continuos_planning", this->continuos_planning_) && this->get_parameter("work_space", this->work_space_) && this->get_parameter("controller_action", this->controller_action_) && this->get_parameter("trajectory_name", this->trajectory_name_)){
      // If the parameters are found, print an information message and start the planning
      RCLCPP_INFO(this->get_logger(), "Planning started from joint_model_group: %s", this->joint_model_group_name_.c_str());
     
      start_planning();

    }
    else{ // If at least one parameter is not found, print an error message and shutdown the node
      RCLCPP_ERROR(this->get_logger(), "Planning group name, base link, end effector link or controller action not found");
      rclcpp::shutdown();
    }

  }


protected:
  /**
   * @brief This attribute is used to store the joint model group name
   */
  std::string joint_model_group_name_; 
  /** 
   * @brief This attribute is used to store the base link for the visual tools 
   */
  std::string base_link_;
  /**
   * @brief This attribute is used to store the end effector link
   */
  std::string end_effector_link_; 
  /**
   * @brief This attribute is used to store the flag for continuous planning (0 for false so the planning is done only once, 1 for true so the planning is done continuously)
   */
  int continuos_planning_= 0; 
  /**
   * @brief This attribute is used to store the flag for the work space (0 for joint space, 1 for task space)
   */
  int work_space_ = 0; 
  /**
   * @brief This attribute is used to store the controller action name for the trajectory execution
   */
  std::string controller_action_; 
  /**
   * @brief This attribute is used to store the trajectory name given as parameter to the node
   */
  std::string trajectory_name_; 
  /**
   * @brief This function is used to get the translations and rotations of a link by computing the direct kinematics of the robot state
  */
  std::vector<double> getTranslationsAndRotations(moveit::core::RobotStatePtr robot_state, std::string link_name){
    std::vector<double> translationsAndRotations(7, 0.0); // Vector for the translations and rotations, initialized to 0.0 by default and with size 7 (3 for the translations and 4 for the rotations)
    auto isometry = robot_state->getGlobalLinkTransform(link_name); // Get the global link transform

    auto t = tf2::eigenToTransform(isometry); // Convert the Eigen transform to a transform message
    tf2::Quaternion quaternion; // Quaternion for the rotations
    tf2::fromMsg(t.transform.rotation, quaternion); // Convert the rotation from the transform message to the quaternion
    // Set the translations and rotations in the vector
    translationsAndRotations[0] = t.transform.translation.x;
    translationsAndRotations[1] = t.transform.translation.y;
    translationsAndRotations[2] = t.transform.translation.z;
    translationsAndRotations[3] = quaternion.getW();
    translationsAndRotations[4] = quaternion.getX();
    translationsAndRotations[5] = quaternion.getY();
    translationsAndRotations[6] = quaternion.getZ();
    return translationsAndRotations; // Return the vector
  }
  /**
   * @brief This function is used to compute a point to point trajectory in the joint space, it is used to plan the trajectory in the joint space and to check if the trajectory is valid
   */
  bool computePointToPoint(int numberoftries,  moveit::planning_interface::MoveGroupInterface &move_group_interface, geometry_msgs::msg::Pose target_pose, double position_tolerance, double orientation_tolerance, moveit::planning_interface::MoveGroupInterface::Plan &plan, std::vector<geometry_msgs::msg::Pose> waypoints = {} , bool returnback=false, bool* computedInversed=nullptr){
  int trie = 1; // Variable for the number of tries
  bool check = false; // Flag for the check of the solution, if a good solution is found, the flag is set to true, otherwise it remains false
  move_group_interface.setPoseTarget(target_pose); // Set the pose target for the move group interface

  while(!check && trie<=numberoftries){ // While the check is false and the number of tries is less than or equal to the maximum number of tries
    RCLCPP_INFO(this->get_logger(), "Planning attempt number %d", trie);
    move_group_interface.plan(plan); // Plan the trajectory

    auto robot_model = move_group_interface.getRobotModel(); // Get the robot model
    auto joint_model_group = robot_model->getJointModelGroup(this->joint_model_group_name_); // Get the joint model group

    moveit::core::RobotStatePtr custom_robot_state = std::make_shared<moveit::core::RobotState>(robot_model); // Create a custom robot state
    custom_robot_state->setToDefaultValues();  // Set the custom robot state to the default values

    std::vector<double> joint_values(move_group_interface.getJointNames().size(), 0.0); // Initialize the joint values to 0.0 by default

    for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i) {
      for (size_t j = 0; j < plan.trajectory_.joint_trajectory.points[i].positions.size(); ++j) {
        if(i==plan.trajectory_.joint_trajectory.points.size()-1){ 
          joint_values[j]=(plan.trajectory_.joint_trajectory.points[i].positions[j]); // Set the joint values to the positions of the last point of the trajectory
        }
      }
    }
    custom_robot_state->setJointGroupPositions(joint_model_group, joint_values); // Set the joint group positions in the custom robot state

    std::vector<double> trans_rot = getTranslationsAndRotations(custom_robot_state,joint_model_group->getLinkModelNames().back()); // Get the translations and rotations by calling the getTranslationsAndRotations function that computes direct kinematics
    
   
    if (std::abs(trans_rot[0] - target_pose.position.x) <= position_tolerance &&
      std::abs(trans_rot[1] - target_pose.position.y) <= position_tolerance &&
      std::abs(trans_rot[2] - target_pose.position.z) <= position_tolerance && ((
      std::abs(trans_rot[3] - target_pose.orientation.w) <= orientation_tolerance &&
      std::abs(trans_rot[4] - target_pose.orientation.x) <= orientation_tolerance &&
      std::abs(trans_rot[5] - target_pose.orientation.y) <= orientation_tolerance &&
      std::abs(trans_rot[6] - target_pose.orientation.z) <= orientation_tolerance) 
      || 
      (std::abs(trans_rot[3] + target_pose.orientation.w) <= orientation_tolerance &&
      std::abs(trans_rot[4] + target_pose.orientation.x) <= orientation_tolerance &&
      std::abs(trans_rot[5] + target_pose.orientation.y) <= orientation_tolerance &&
      std::abs(trans_rot[6] + target_pose.orientation.z) <= orientation_tolerance)))
    { // If the difference between the translations and rotations of the last point of the trajectory and the target pose is less than or equal to the position and orientation tolerance the plan is valid
      check = true;
    }
    else // otherwise the plan is not valid
    {
      trie++;
    }
    
  }
  
  if(check==false && returnback){ // If the check is false and the returnback flag is true, it means that the robot has to return back to the initial position and the plan was failed,
        // so a new plan is computed by inverting the trajectory used to reach the target pose, but in the inverse way
        std::reverse(waypoints.begin(), waypoints.end()); // Reverse the waypoints
        moveit_msgs::msg::RobotTrajectory trajectory; // Create a robot trajectory
        const double jump_threshold = 100.0;  // Jump threshold for the computeCartesianPath function, set to 100.0 by default to avoid extremely large jumps between consecutive points
        const double eef_step = 0.01; // End effector step for the computeCartesianPath function, set to 0.01 by default to have a small step between consecutive points
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Compute the cartesian path 
         
        robotStateToRobotStateMsg(*move_group_interface.getCurrentState(), plan.start_state_); // Set the start state of the plan to the current state of the move group interface
        plan.trajectory_ = trajectory; // Set the trajectory of the plan to the computed trajectory
        if(fraction==1.0){ // If the fraction is equal to 1.0, the plan is valid and the check is set to true, also the computedInversed flag is set to true to indicate that the trajectory was computed by inverting the original one
          check=true;
          *computedInversed=true;
        }
  }
  
  
  return check; // Return the check flag to indicate if the plan is valid or not
}

/**
   * @brief This function is used to convert a joint trajectory to a cartesian trajectory
   */
moveit_msgs::msg::CartesianTrajectory fromJointTrajectoryToCartesianTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, moveit::planning_interface::MoveGroupInterface &move_group_interface){
  moveit_msgs::msg::CartesianTrajectory task_space_trajectory; // Create a task space trajectory 

  task_space_trajectory.header = trajectory.joint_trajectory.header; // Set the header of the task space trajectory to the header of the joint trajectory
  task_space_trajectory.tracked_frame = move_group_interface.getPoseReferenceFrame(); // Set the tracked frame of the task space trajectory to the pose reference frame of the move group interface
 
  task_space_trajectory.points.resize(trajectory.joint_trajectory.points.size()); // Resize the points of the task space trajectory to the size of the points of the joint trajectory

  auto robot_model = move_group_interface.getRobotModel(); // Get the robot model
  auto joint_model_group = robot_model->getJointModelGroup(this->joint_model_group_name_); // Get the joint model group
  moveit::core::RobotStatePtr custom_robot_state = std::make_shared<moveit::core::RobotState>(robot_model); // Create a custom robot state
  custom_robot_state->setToDefaultValues();  // Set the custom robot state to the default values
  std::vector<double> joint_values(move_group_interface.getJointNames().size(), 0.0); // Initialize the joint values to 0.0 by default
  std::vector<double> joint_velocities(move_group_interface.getJointNames().size(), 0.0); // Initialize the joint velocities to 0.0 by default
 
  for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
      for (size_t j = 0; j < trajectory.joint_trajectory.points[i].positions.size(); ++j) {
          joint_values[j] = trajectory.joint_trajectory.points[i].positions[j]; // Set the joint values to the positions of the points of the joint trajectory
          joint_velocities[j] = trajectory.joint_trajectory.points[i].velocities[j]; // Set the joint velocities to the velocities of the points of the joint trajectory
      }

      custom_robot_state->setJointGroupPositions(joint_model_group, joint_values); // Set the joint group positions in the custom robot state
      std::vector<double> data = getTranslationsAndRotations(custom_robot_state, joint_model_group->getLinkModelNames().back()); // Get the translations and rotations by calling the getTranslationsAndRotations function that computes direct kinematics
      Eigen::MatrixXd jacobian; // Create a Jacobian matrix

      if(!custom_robot_state->getJacobian(joint_model_group, custom_robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), Eigen::Vector3d(0.0, 0.0, 0.0), jacobian)){ // If the Jacobian is not computed, print an error message and shutdown the node otherwise return the jacobian
        RCLCPP_ERROR(this->get_logger(), "Failed to get jacobian");
        rclcpp::shutdown();
      }

      Eigen::VectorXd eigen_joint_velocities = Eigen::Map<Eigen::VectorXd>(joint_velocities.data(), joint_velocities.size()); // Map the joint velocities to an Eigen vector
      Eigen::VectorXd cartesian_velocities = jacobian * eigen_joint_velocities; // Compute the cartesian velocities by multiplying the jacobian and the joint velocities v = J(q) * q_dot

      // Set the point of the task space trajectory having the position, orientation and velocities of the respective point of the joint trajectory
      task_space_trajectory.points[i].point.pose.position.x = data[0];
      task_space_trajectory.points[i].point.pose.position.y = data[1];
      task_space_trajectory.points[i].point.pose.position.z = data[2];
      task_space_trajectory.points[i].point.pose.orientation.w = data[3];
      task_space_trajectory.points[i].point.pose.orientation.x = data[4];
      task_space_trajectory.points[i].point.pose.orientation.y = data[5];
      task_space_trajectory.points[i].point.pose.orientation.z = data[6];
      
      task_space_trajectory.points[i].point.velocity.linear.x = cartesian_velocities[0];
      task_space_trajectory.points[i].point.velocity.linear.y = cartesian_velocities[1];
      task_space_trajectory.points[i].point.velocity.linear.z = cartesian_velocities[2];
      task_space_trajectory.points[i].point.velocity.angular.x = cartesian_velocities[3];
      task_space_trajectory.points[i].point.velocity.angular.y = cartesian_velocities[4];
      task_space_trajectory.points[i].point.velocity.angular.z = cartesian_velocities[5];

      task_space_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start;
  }
  return task_space_trajectory; // Return the task space trajectory
}
/**
   * @brief This function is used to check the validity of the parameters of the node
  */
void check_validity(moveit::planning_interface::MoveGroupInterface &move_group_interface){
  if(this->work_space_!= 0 && this->work_space_!=1){ // If the work space parameter is not valid, print an error message and shutdown the node
    RCLCPP_ERROR(this->get_logger(), "Work space parameter not valid, shutting down...");
    rclcpp::shutdown();
  }
  bool link_founded = false;
  for(auto const& link_name : move_group_interface.getRobotModel()->getLinkModelNames()) // Check if the base link is found, otherwise print an error message and shutdown the node
  {
    if(link_name == this->base_link_)
    {
      link_founded = true;
      break;
    }
  }
  if(!link_founded)
  {
    RCLCPP_ERROR(this->get_logger(), "Base link specified not found, shutting down...");
    rclcpp::shutdown();
  }
  if(!move_group_interface.getRobotModel()->getJointModelGroup(this->joint_model_group_name_)->hasLinkModel(this->end_effector_link_)) // Check if the end effector link is found in the current joint model group, otherwise print an error message and shutdown the node
  {
    RCLCPP_ERROR(this->get_logger(), "End effector link specified not found, shutting down...");
    rclcpp::shutdown();
  }
}

/**
   * @brief This is the main function of the node, it is used to start the planning and the trajectory execution
  */
void start_planning(){

  auto const node = std::make_shared<fanuc_m20ia_35m_planning_demo::ActionNode>(); // Create an action node for the trajectory execution, documented in the action_node.hpp and action_node.cpp files

  rclcpp::executors::SingleThreadedExecutor executor; // Create a single threaded executor

  executor.add_node(node); // Add the action node to the executor
  
  auto spinner = std::thread([&executor]() 
  { // Create a spinner thread to spin the executor
    executor.spin();
    rclcpp::shutdown();
  });
  using moveit::planning_interface::MoveGroupInterface;


  auto move_group_interface = MoveGroupInterface(node, this->joint_model_group_name_); // Create a move group interface with the joint model group name
  check_validity(move_group_interface); // Check the validity of the parameters 
 
  
  move_group_interface.setMaxVelocityScalingFactor(1.0); // Set the maximum velocity scaling factor to 1.0, to avoid scaling the velocity and have the maximum velocity for the robot
  move_group_interface.setMaxAccelerationScalingFactor(1.0); // Set the maximum acceleration scaling factor to 1.0, to avoid scaling the acceleration and have the maximum acceleration for the robot
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ // Create moveit visual tools for the visualisation of the planning and the trajectory execution in RViz starting from the base link
    node, this->base_link_, rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.loadRemoteControl();
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // Create a planning scene interface
  moveit_msgs::msg::CollisionObject collision_object; // Create a collision object for the floor
  collision_object.header.frame_id = move_group_interface.getPlanningFrame(); // Set the frame id of the collision object to the planning frame of the move group interface
  collision_object.id = "floor"; 
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 10; // Set the dimensions of the floor to 10x10x0.005, to have a floor that represent the one present in Gazebo
  primitive.dimensions[1] = 10;
  primitive.dimensions[2] = 0.005;
  geometry_msgs::msg::Pose floor_pose;
  floor_pose.orientation.w = 1.0;
  floor_pose.position.z = -0.005;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(floor_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner"); // Set the planning pipeline id to the pilz industrial motion planner
  move_group_interface.setPlannerId("LIN"); // Set the planner id to LIN, to have a linear interpolation between the points of the trajectory, PTP is not used because it is not suitable without acceleration limits that are not present in the datasheets of the robot and online
  auto const logger = rclcpp::get_logger("fanuc_m20ia_35m_planning_demo_client"); // Create a logger for the node
  bool check = true; // Flag for the continuous planning, if true the planning is done continuously, otherwise it is done only once
  bool failed = false; // Flag for the service response, if true the service response is failed, otherwise it is not failed
  rosbag2_cpp::Reader reader_; // Create a reader for the bagfile that contains the trajectory
  geometry_msgs::msg::Pose posa_home = move_group_interface.getCurrentPose().pose; // Get the current pose of the robot, to return back to the initial pose after the trajectory execution
  rclcpp::Serialization<geometry_msgs::msg::Pose> serialization_; // Create a serialization for the geometry messages pose
  // Variables for the trajectory name, as convention the trajectory name is the current date and time: Trajectory_%Y-%m-%d_%H:%M:%S
  std::chrono::_V2::system_clock::time_point now; 
  time_t timestamp; 
  std::filesystem::path currentPath; // Path for the current directory to read the bagfile from the correct directory
  std::string filename; // Filename for the bagfile, named as said in the convention
  double position_tolerance = 0.01; // Position tolerance for the planning, set to 0.01 by default
  double orientation_tolerance = 0.01;  // Orientation tolerance for the planning, set to 2 by default
  int numberoftries = 1; // Number of tries for the planning, set to 1 by default
  moveit::planning_interface::MoveGroupInterface::Plan plan; // Plan for the trajectory
  std::string trajectory_name; // Trajectory name 
  bool first = false; // Flag for the first planning, to check if a trajectory is passed to the action node or not
  if(this->trajectory_name_!=""){ // If the trajectory name is not empty, the first planning is done with the trajectory passed in input
    first = true;
  }

  rclcpp::Client<trajectory_interfaces::srv::SetTrajectoryStart>::SharedPtr client =
    node->create_client<trajectory_interfaces::srv::SetTrajectoryStart>("set_trajectory"); // Create a client for the set trajectory service
  
  if(this->work_space_==1){ // If the work space is 1, the action server for the trajectory execution is the cartesian one, otherwise it is the joint one
    node->client_action=rclcpp_action::create_client<action_cartesian_trajectory::action::FollowCartesianTrajectory>(node, this->controller_action_);
    using namespace std::placeholders;
    node->send_goal_options = rclcpp_action::Client<action_cartesian_trajectory::action::FollowCartesianTrajectory>::SendGoalOptions();
    node->send_goal_options.goal_response_callback =std::bind(&fanuc_m20ia_35m_planning_demo::ActionNode::goal_response_callback, node, _1);
    node->send_goal_options.feedback_callback = std::bind(&fanuc_m20ia_35m_planning_demo::ActionNode::feedback_callback, node, _1, _2);
    node->send_goal_options.result_callback = std::bind(&fanuc_m20ia_35m_planning_demo::ActionNode::result_callback, node, _1);
  }else{
    node->joint_client_action=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node, this->controller_action_);
    using namespace std::placeholders;
    node->joint_send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    node->joint_send_goal_options.goal_response_callback =std::bind(&fanuc_m20ia_35m_planning_demo::ActionNode::joint_goal_response_callback, node, _1);
    node->joint_send_goal_options.feedback_callback = std::bind(&fanuc_m20ia_35m_planning_demo::ActionNode::joint_feedback_callback, node, _1, _2);
    node->joint_send_goal_options.result_callback = std::bind(&fanuc_m20ia_35m_planning_demo::ActionNode::joint_result_callback, node, _1);
  }


  auto const draw_title = [&moveit_visual_tools](auto text) 
  { // Lambda function to draw the title in the RViz visual tools, with a big text to have a clear visualization and with a z translation of 2.5 to have the title above the robot
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 2.5;  
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XXXXLARGE);
  };

  auto const  prompt = [&moveit_visual_tools](auto text) 
  { // Lambda function to prompt the user to press the next button in the RViz visual tools
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path = [&moveit_visual_tools,
    jmg = move_group_interface.getRobotModel()->getJointModelGroup(this->joint_model_group_name_), link=move_group_interface.getRobotModel()->getLinkModel(this->end_effector_link_)]
    (auto const trajectory) 
  { // Lambda function to draw the trajectory in the RViz visual tools
    moveit_visual_tools.publishTrajectoryLine(trajectory, link, jmg, rviz_visual_tools::LIME_GREEN);
  };

  auto const setup_client = [&logger, &node, &moveit_visual_tools, &draw_trajectory_tool_path, &draw_title, &prompt,  this](bool isTrajectory, std::string trajectory_name = "", auto const plan){ // Lambda function to setup the client for the trajectory execution
    draw_trajectory_tool_path(plan.trajectory_); // Draw the trajectory in the RViz visual tools
    moveit_visual_tools.trigger(); 
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute"); // Prompt the user to press the next button in the RViz visual tools
    draw_title("Executing");
    moveit_visual_tools.trigger();

    if(!node->setStoreTrajectory(isTrajectory, trajectory_name, this->work_space_)){
      RCLCPP_ERROR(logger, "Failed to set the store trajectory flag in the action node, so the trajectory is not stored!");
    } // Set the store trajectory flag in the action node, to store the trajectory in the bagfile if the flag is true
    node->promise=std::promise<bool>(); // Create a promise for the future of the trajectory execution
    node->future=node->promise.get_future(); // Get the future of the trajectory execution from the promise to check if the goal is reached or not
  };

  auto const execute = [&logger, &move_group_interface, &moveit_visual_tools, 
    &draw_title, &prompt, &draw_trajectory_tool_path, &node, &setup_client]
    (auto const success, auto const plan, bool isTrajectory, std::string trajectory_name = "")
  { // Lambda function to execute the trajectory in the joint space
    if (success) 
    {
      if(!node->joint_client_action->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(logger, "Controller action server not available after waiting...");
        return;
      } // If the joint client action is not available after waiting, print an error message and return from the function
      auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal(); // Create a goal message for the joint trajectory
      goal_msg.trajectory = plan.trajectory_.joint_trajectory; // Set the trajectory of the goal message to the joint trajectory of the plan

      setup_client(isTrajectory, trajectory_name, plan); // Call the setup client function to setup the client for the trajectory execution

      node->joint_client_action->async_send_goal(goal_msg, node->joint_send_goal_options); // Send the goal message to the joint client action
       if(node->future.get()){ // If the future is valid, print an information message if the goal is reached, otherwise print an error message
        RCLCPP_INFO(logger, "Goal reached");
      }
      else{
        RCLCPP_ERROR(logger, "Goal not reached");
      }

    } 
    else
    { // If the planning is failed, print an error message
      draw_title("Planning\xa0\x66\x61iled!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  };


  auto const execute_cartesian = [&logger, &move_group_interface, &moveit_visual_tools, 
    &draw_title, &prompt, &draw_trajectory_tool_path, &node, &setup_client]
    (auto const success, auto const plan, bool isTrajectory, std::string trajectory_name = "", moveit_msgs::msg::CartesianTrajectory cartesianTrajectory)
  { // Lambda function to execute the trajectory in the task space (cartesian space)
    if (success) 
    {
       if (!node->client_action->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(logger, "Controller action server not available after waiting...");
        return;
      } // If the client action is not available after waiting, print an error message and return from the function
      auto goal_msg = action_cartesian_trajectory::action::FollowCartesianTrajectory::Goal(); // Create a goal message for the cartesian trajectory
      goal_msg.trajectory = cartesianTrajectory; // Set the trajectory of the goal message to the cartesian trajectory

      setup_client(isTrajectory, trajectory_name, plan); // Call the setup client function to setup the client for the trajectory execution

      node->client_action->async_send_goal(goal_msg, node->send_goal_options); // Send the goal message to the client action
      if(node->future.get()){ // If the future is valid, print an information message if the goal is reached, otherwise print an error message
        RCLCPP_INFO(logger, "Goal reached");
      }
      else{
        RCLCPP_ERROR(logger, "Goal not reached");
      }
     
      
    } 
    else
    { // If the planning is failed, print an error message
      draw_title("Planning\xa0\x66\x61iled!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  };

   auto handle_service_response = [&logger, &check,&failed](rclcpp::Client<trajectory_interfaces::srv::SetTrajectoryStart>::SharedFuture future) { // Lambda function to handle the service response of the set trajectory service
      if (!future.get()->success) { // If the success flag of the future is false, print an error message and set the failed flag to true
          failed = true;
      }
  };
  
  while (check) { // While the check is true, the planning is done continuously, otherwise it is done only once
    if(this->continuos_planning_==1){ // If the continuous planning flag is true, the planning is done continuously, otherwise it is done only once
      check = false;
    }
    currentPath = std::filesystem::current_path(); 

    if(first){
      prompt("Press 'Next' in the RvizVisualToolsGui window to plan the parabolic path passed in input: " + this->trajectory_name_);
      filename = currentPath.string() + "/matlab_scripts/bagfiles/" + this->trajectory_name_ + "/" + this->trajectory_name_ + "_0.db3"; // Set the filename for the bagfile to the current path and the trajectory name 
      first=false;
      trajectory_name = this->trajectory_name_; // Set the trajectory name to the trajectory name passed in input
    }
    else{
      prompt("Press 'Next' in the RvizVisualToolsGui window to plan the new parabolic path and wait for MATLAB server");
      moveit_visual_tools.deleteAllMarkers();
      draw_title("Planning MATLAB");
      moveit_visual_tools.trigger();
      if (!client->wait_for_service(std::chrono::seconds(1))) { // If the client is not available after waiting, print an error message and break from the loop to shutdown the planning because the MATLAB service is fundamental for the planning 
          RCLCPP_ERROR(logger, "Matlab service is down! Shutting down the planning...");
          break;
      }

      auto request = std::make_shared<trajectory_interfaces::srv::SetTrajectoryStart::Request>(); // Create a request for the set trajectory service
      now = std::chrono::system_clock::now();
      timestamp = std::chrono::system_clock::to_time_t(now);
      std::stringstream trajectory_string;
      trajectory_string << "Trajectory_" << std::put_time(std::localtime(&timestamp), "%Y-%m-%d_%H:%M:%S"); // Set the trajectory name as the current date and time as convention

      request->trajectory_infos = trajectory_string.str(); // Set the trajectory infos of the request to the trajectory name
      trajectory_name = trajectory_string.str(); // Set the trajectory name to the trajectory name string

      filename = currentPath.string() + "/matlab_scripts/bagfiles/" + trajectory_string.str() + "/" + trajectory_string.str() + "_0.db3"; // Set the filename for the bagfile to the current path and the trajectory name 
      failed=false;
      auto result = client->async_send_request(request, handle_service_response); // Send the request to the client and handle the service response
      auto status = result.wait_until(std::chrono::system_clock::now() + std::chrono::seconds(300)); // Wait for the service response for 5 minutes (MATLAB is slow to compute the trajectory)
      if (status == std::future_status::timeout) { // If the status is timeout, print an error message and set the failed flag to true
        RCLCPP_ERROR(logger, "Service call failed, too much time to compute the trajectory, retry");
        failed = true;
      }else{
        result.get(); // Get the result of the service response
      }
    }
    if(failed==false){ // If the service didn't fail, in the meaning that the MATLAB server is up and runnin and the trajectory is feasible to be executed (is in the action space of the robot), the planning is done

    if(!first){
      prompt("Saving the trajectory:" + trajectory_name + ", press next");}

    try{
    reader_.open(filename);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "Error opening the bagfile: %s", e.what());
      rclcpp::shutdown();
    }
    auto const waypoints = [&move_group_interface, &reader_, &serialization_]
    { // Lambda function to read the waypoints from the bagfile
      std::vector<geometry_msgs::msg::Pose> waypoints;
      while (reader_.has_next()) // While the bagfile is not finished, read the next message
      {
          rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next(); // Read the next message
          rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);  // Create a serialized message
          geometry_msgs::msg::Pose::SharedPtr ros_msg = std::make_shared<geometry_msgs::msg::Pose>(); // Create a shared pointer for the geometry message pose
          serialization_.deserialize_message(&serialized_msg, ros_msg.get()); // Deserialize the message to get the geometry message pose
          // Set the position and orientation of the pose to the position and orientation of the geometry message pose
          geometry_msgs::msg::Pose temporary_pose;
          temporary_pose.position.x = ros_msg->position.x;
          temporary_pose.position.y = ros_msg->position.y;
          temporary_pose.position.z = ros_msg->position.z;
          temporary_pose.orientation.x = ros_msg->orientation.x;
          temporary_pose.orientation.y = ros_msg->orientation.y;
          temporary_pose.orientation.z = ros_msg->orientation.z;
          temporary_pose.orientation.w = ros_msg->orientation.w;
          waypoints.push_back(temporary_pose);  // Push back the temporary pose in the waypoints vector
      }
      return waypoints; // Return the waypoints vector made of the temporary poses contained in the bagfile
    }();

    prompt("Press 'Next' in the RvizVisualToolsGui window to move the robot to the first point of the trajectory");
    moveit_visual_tools.deleteAllMarkers();
    draw_title("Planning 1st point");
    moveit_visual_tools.trigger();
    auto first_trajectory_pose= waypoints[0]; // Get the first trajectory pose from the waypoints vector where the robot has to move in order to start the trajectory

    move_group_interface.setPoseTarget(first_trajectory_pose); // Set the pose target for the move group interface to the first trajectory pose

    
    if(computePointToPoint(numberoftries, move_group_interface, first_trajectory_pose, position_tolerance, orientation_tolerance, plan)){ // If the trajectory to the first point is computed, the robot moves to the first point
          RCLCPP_INFO(logger, "The planner found a solution, not an approximate one");
          if(this->work_space_==0){ // If the work space is 0, the trajectory is executed in the joint space, otherwise it is executed in the task space (cartesian space)
            execute(true, plan, false);
          }
          else{
            execute_cartesian(true,plan, false, trajectory_name, fromJointTrajectoryToCartesianTrajectory(plan.trajectory_, move_group_interface));
          }

          prompt("Press 'Next' to see the parabola trajectory chosen in MATLAB");
          draw_title("Plotting the trajectory...");

          moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::RED, rviz_visual_tools::SMALL); // Publish the path of the waypoints in the RViz visual tools to visualize the trajectory create in MATLAB
          moveit_visual_tools.trigger();
          

          prompt("Press 'Next' in the RvizVisualToolsGui window to execute the planned trajectory");
          moveit_visual_tools.deleteAllMarkers();
          draw_title("Planning the trajectory and executing...");
          moveit_visual_tools.trigger();

          auto const [fraction, cartesian_plan] = [&logger, &move_group_interface, &waypoints]
          { // Lambda function to compute the cartesian plan
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 100.0; // Jump threshold for the computeCartesianPath function, set to 100.0 by default to avoid extremely large jumps between consecutive points
            const double eef_step = 0.01; // End effector step for the computeCartesianPath function, set to 0.01 by default to have a small step between consecutive points
            double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Compute the cartesian path
            
            RCLCPP_INFO(logger, "Achieved %.2f%% of Cartesian path", fraction * 100.0); // Print the achieved percentage of the cartesian path for debug purposes
            robotStateToRobotStateMsg(*move_group_interface.getCurrentState(), plan.start_state_); // Set the start state of the plan to the current state of the move group interface
            plan.trajectory_ = trajectory; 
            return std::make_pair(fraction, plan); // Return the fraction and the plan
          }();

          if(this->work_space_==0){ // If the work space is 0, the trajectory is executed in the joint space, otherwise it is executed in the task space (cartesian space)
            execute(fraction==1.0, cartesian_plan, true, trajectory_name);}
          else{
            execute_cartesian(fraction==1.0, cartesian_plan, true, trajectory_name, fromJointTrajectoryToCartesianTrajectory(cartesian_plan.trajectory_, move_group_interface));
          }

        
          prompt("Press 'Next' in the RvizVisualToolsGui window to move the robot to the initial pose");
          moveit_visual_tools.deleteAllMarkers();
          draw_title("Planning initial pose");
          moveit_visual_tools.trigger();
          
          
          bool computedInversed=false; // Flag for the computed inversed trajectory, if true the trajectory was computed by inverting the original one
          if(computePointToPoint(numberoftries, move_group_interface, posa_home, position_tolerance, orientation_tolerance, plan, waypoints, fraction==1.0, &computedInversed)){ // If the trajectory to the initial pose is computed, the robot moves to the initial pose
                  if(computedInversed){ // If the computed inversed flag is true, print an information message to indicate that the trajectory was computed by inverting the original one so the robot has to return back to the initial pose in two steps
                    RCLCPP_INFO(logger, "The planner has not found a solution, so it has computed the inverse of the trajectory, computing the trajectory to the initial pose in two steps");
                    if(this->work_space_==0){ // If the work space is 0, the trajectory is executed in the joint space, otherwise it is executed in the task space (cartesian space)
                      execute(true,plan,false);

                    }
                    else{
                      execute_cartesian(true,plan, false, trajectory_name, fromJointTrajectoryToCartesianTrajectory(plan.trajectory_, move_group_interface));
                    }
                    if(computePointToPoint(numberoftries, move_group_interface, posa_home, position_tolerance, orientation_tolerance, plan)){ // Plan the second step of the trajectory to the initial pose, if the trajectory is computed, the robot moves to the initial pose
                      RCLCPP_INFO(logger, "The second step of the trajectory to the initial pose has been computed");
                      if(this->work_space_==0){ // If the work space is 0, the trajectory is executed in the joint space, otherwise it is executed in the task space (cartesian space)
                        execute(true,plan,false);
                      }
                      else{ 
                        execute_cartesian(true,plan, false, trajectory_name, fromJointTrajectoryToCartesianTrajectory(plan.trajectory_, move_group_interface));
                      }
                  }
                  }
                  else{ // If the computed inversed flag is false, print an information message to indicate that the trajectory was computed in one step
                    RCLCPP_INFO(logger, "The planner has found a solution, not an approximate one");
                    if(this->work_space_==0){ // If the work space is 0, the trajectory is executed in the joint space, otherwise it is executed in the task space (cartesian space)
                      execute(true,plan,false);
                    }
                    else{
                      execute_cartesian(true,plan, false, trajectory_name, fromJointTrajectoryToCartesianTrajectory(plan.trajectory_, move_group_interface));
                    }
                  }
            }
            else{ // If the trajectory to the initial pose is not computed, print an error message
              RCLCPP_ERROR(logger, "The planner has not found a solution, or just an approximate one");
            }
    }
    else{ // If the trajectory to the first point is not computed, print an error message
      RCLCPP_ERROR(logger, "The planner has not found a solution, or just an approximate one");
    }
  }
  else{ // If the service failed, print an error message
      RCLCPP_ERROR(logger, "Trajectory not calculated, because the service failed");
  }

}
// End of the planning and trajectory execution process, prompt the user to press the next button in the RViz visual tools to exit
// and clean the RViz visual tools
moveit_visual_tools.deleteAllMarkers();
draw_title("Done with planning");
moveit_visual_tools.trigger();
prompt("Press 'Next' in the RvizVisualToolsGui window to exit");
moveit_visual_tools.deleteAllMarkers();
moveit_visual_tools.trigger();
rclcpp::shutdown();
  }
};  


/**
   * @brief This is the constructor of the Fanuc_m20ia_35m_PlanningDemoClient class
  */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fanuc_m20ia_35m_PlanningDemoClient>());
  rclcpp::shutdown();
  return 0;
}