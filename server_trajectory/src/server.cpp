/** -------------------------------------------------------------------
 * 
 * @file:   server.cpp
 * @author:  Group 2 Robotics Project 2023/2024 UNISA
 * @date:    Feb 7, 2024
 *
 * @brief This node is used to start the trajectory planning in MATLAB
 * receiving the parameters from the client node and returning the result
 * 
 * -------------------------------------------------------------------
 */
#include "rclcpp/rclcpp.hpp"
#include "trajectory_interfaces/srv/set_trajectory_start.hpp"
#include <filesystem>

#include <memory>
/**
   * @brief This function is used to start the trajectory planning in MATLAB
   */
void start_matlab(const std::shared_ptr<trajectory_interfaces::srv::SetTrajectoryStart::Request> request,
         std::shared_ptr<trajectory_interfaces::srv::SetTrajectoryStart::Response> response)
{
 
  std::string command = "./matlab_scripts/launch_matlab_script.sh"; // The path to the script that launches the MATLAB
  command += " " + request->trajectory_infos;  // The parameters for the trajectory planning
  auto returnCode = system(command.c_str()); // Launch the script
   if (returnCode == 0) { // If the script is executed correctly return the success message
        response->success = true;
        response->result="Parabolic path created";

    }
    else { // If the script is not executed correctly return the error message
        response->success=false;
        response->result="Error in the parabolic path creation";
    }
  
}

/**
   * @brief This is the main function that creates the node and the service
   */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("set_trajectory_server");


  auto service = node->create_service<trajectory_interfaces::srv::SetTrajectoryStart>(
    "set_trajectory", &start_matlab);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to receive trajectory strings.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}