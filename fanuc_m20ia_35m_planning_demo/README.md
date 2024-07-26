# fanuc_m20ia_35m_planning_demo

This package provides a demo for MoveIt!2 planning capabilities by using the `MoveGroupInterface` class.
In particular this package provide to the user the possibility to generate a parabola, interfacing with the 
`trajectory_server` service. After that it plans a path to the first point of the trajectory and call the
controller to execute it with the relative controller. The same happens for the parabola itself and for reset the robot to the initial position.

## Usage with single nodes

WARNING: You should always start this nodes in `unisa_acg_ros2_group2`  directory, otherwise some thing could crash! 


In a first shell launch the `move_group` node and utility nodes, like RViz, from the `acg_resources_fanuc_m20ia_35m_moveit_config` package.
Make sure to select the appropriate RViz configuration:

```bash
ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config moveit_gazebo_ros2_control_demo.launch.py use_effort_interface:=<use_effort_interface> end_effector:=<end_effector>
```

In another shell run the `trajectory_server` node from the `server_trajectory` package:

```bash
ros2 launch server_trajectory demo.launch.py
```
In another shell run this node with:
```bash
ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] controller_action:=<controller_action> work_space:=<work_space> [trajectory_name:=<trajectory_name>]
```
where the parameters specify the joint model group on which make the planning (`joint_model_group_name`), the base link whom the visual tools are related (`world` in the case of this robot, the argument is `base_link`), the end effector link (`end_effector_link`) and, optionally, also a 
flag that indicates if the planning should be one shot [1], or continous till the manual ending of the node by the user [every other number] (`continuous_planning`). The user must specify also the name of the action that the controller make available to send a goal (`controller_action`), and the work space parameter is a flag that indicates what type of planning the user want to do, joint space [0] or cartesian space [1] (`work_space`). Furthemore, if an old trajectory is request to be executed, the user can specify the name of the trajectory
to be executed (`trajectory_name`). If the planning is continuos, the trajectory will be executed and then normally the MATLAB server will take place, otherwise the server is not runned, cause there is no need to compute new paths. 

WARNING: If the old trajectory is generated from this work flow, the user have just to set proprerly the name of the attribute, but if the trajectory is stored in an another bagfile, the user should follow this convention: create a new
folder in `matlab_script/bagfiles` with the name of the trajectory, for example `TrajectoryOld`, and add the bagfile in it in the format name of name of the trajectory + `_0` , so following the previous example: `TrajectoryOld_0.db3`. When the user call the script he should run trajectory_name:=TrajectoryOld
WARNING: If the user would execute and old trajectory, they should remove, from the `matlab_scripts/results_bagfiles` the corresponding folder containing the desired and actual trajectories computed in the past

For example to run the planning for the planning group containing the drilling tool one could use:

```bash
ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=fanuc_m20ia_35m_slide_to_drilling base_link:=world end_effector_link:=drilling_tool controller_action:=fanuc_m20ia_35m_slide_to_end_effector_controller/follow_joint_trajectory work_space:=0
```

Use the **RVizVisualToolsGUI** to step through the demo, to be more specific press the `Next` button in RViz to proceed in the planning pipeline.

## Usage with all nodes togheter

WARNING: You should always start this in ` unisa_acg_ros2_group2`  directory, otherwise some thing could crash!

To just run all the nodes with just one file use:

```bash
ros2 launch fanuc_m20ia_35m_planning_demo demo.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] controller_action:=<controller_action> work_space:=<work_space> [trajectory_name:=<trajectory_name>]
```
specifying the same parameters as said before, for example to run the planning for the planning group containing the drilling tool one could use:

```bash
ros2 launch fanuc_m20ia_35m_planning_demo demo.launch.py joint_model_group_name:=fanuc_m20ia_35m_slide_to_drilling base_link:=world end_effector_link:=drilling_tool controller_action:=fanuc_m20ia_35m_slide_to_end_effector_controller/follow_joint_trajectory work_space:=0
```

# Notes
In this version of planning node, an obstacle mapping the gazebo floor is inserted, if the external simulator will change, or the environment, it is advised to remove the obstacle. It was not created a configuration with no floor because in Gazebo or in reality the slider is mounted on the ground, but if that is not the case, removing the obstacle is the solution.