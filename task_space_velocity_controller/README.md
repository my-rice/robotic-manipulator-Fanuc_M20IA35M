# Velocity Controller Package

This package implements a velocity controller for ROS2 (Robot Operating System 2). The controller is designed to work in the task space, providing precise control over the robot's movements in its environment.
The package provides the following features:
- Velocity control: The controller manages the velocity of the robot, ensuring it moves at the desired pose;
- Task space control: The controller operates in the task space, meaning it controls the robot based on its position, orientation, and velocity in the workspace. This provides more intuitive and precise control compared to joint space control;
- ROS2 integration: The controller is fully integrated with ROS2, allowing it to work seamlessly with other ROS2 packages and tools. It can be easily incorporated into larger ROS2 systems and workflows.

## Usage

To use the velocity controller, it is needed to configure it on the robot through the `ros2_controllers.yaml` file in the `acg_resources_fanuc_m20ia_35m_moveit_config` package. The following parameters are required:
- `controller_gain`: This is the gain vector of the controller.
- `joint_model_group_name`: This is the name of the joint model group that the controller should control. For example, "fanuc_m20ia_35m_slide_to_drilling".
- `robot_description`: This should be set to "robot_description", which is the standard name for the parameter that holds the URDF description of the robot.
- `frequency`: This is the frequency at which the controller should run. A typical value is 1000 (Hz).
- `position_tolerance`: This is the tolerance for the position error of the first pose. A typical value is 0.01 (meters).
- `angle_tolerance`: This is the tolerance for the angle error of the first pose.

Please note that the `command_interfaces` parameter is not present because it is implicitly set to 'velocity' as the package deals with a task space **velocity** controller.

This package cannot be used on its own as it has no launch file in it, but it can be used by other packages and used in a node if its features are needed. 

