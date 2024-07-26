# Parabolic Trajectories Simulation on Fanuc M20iA 35M

The objectives of this project work are:
1. to provide a ROS2-based simulation environment for the Fanuc M20iA 35M robot;
2. to design a ROS2-based planning system that allows planning for sequences of points in the
task space and joint space;
3. to execute trajectories on a simulated robot by using independent joint control;
4. to execute trajectories on a simulated robot by using task space control;
5. to assess the performance of the controller in terms of joint space tracking and task space tracking.


## Usage

- If you want to execute parabolic trajectories simulation using velocity controllers, you have to follow the instructions contained in the [README.md](task_space_velocity_controller/README.md) file in the `task_space_velocity_controller` folder.

- If you want to execute parabolic trajectories simulation using torque controllers (independent joint control), you have to follow the instructions contained in the [README.md](fanuc_m20ia_35m_planning_demo/README.md) file in the `fanuc_m20ia_35m_planning_demo` folder.

- If you want to execute parabolic trajectories simulation neglecting the robot dynamics, you have to follow the instructions contained in the [README.md](acg_resources_fanuc_m20ia_35m_moveit_config/README.md) file in the `acg_resources_fanuc_m20ia_35m_moveit_config` folder.

## Contributors

@my-rice 
@sunpipe
@AdoPagaTroppo