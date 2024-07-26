from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # Default value for the use_sim_time parameter
    jmg_param = None # Default value for the joint model group name
    base_link = None # Default value for the base link
    end_effector_link = None # Default value for the end effector link
    continuos_planning = 0 # Default value for the continuos planning parameter (1 means no continuos planning, other values means continuos planning)
    work_space = 1 # Default value for the work space parameter (0 means joint space, 1 means task space)
    controller_action = None # Default value for the controller action
    trajectory_name = "" # Default value for the trajectory name
    if sys.argv == None:
        print("No arguments passed, please use ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] work_space:=<work_space> controller_action:=<controller_action> [trajectory_name:=<trajectory_name>], the [] means that the argument is optional")
        sys.exit()
    # Get the parameters from the command line
    for arg in sys.argv:
        if arg.startswith("joint_model_group_name:="):
            jmg_param = (arg.split(":=")[1])
        if arg.startswith("base_link:="):
            base_link = (arg.split(":=")[1])
        if arg.startswith("end_effector_link:="):
            end_effector_link = (arg.split(":=")[1])
        if arg.startswith("continuos_planning:="):
            continuos_planning = int(arg.split(":=")[1])
        if arg.startswith("work_space:="):
            work_space = int(arg.split(":=")[1])
        if arg.startswith("controller_action:="):
            controller_action = (arg.split(":=")[1])
        if arg.startswith("trajectory_name:="):
            trajectory_name = (arg.split(":=")[1])
    # Check if the required parameters are passed and exit if not
    if jmg_param == None:
        print("No joint model group name passed, please use ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] work_space:=<work_space> controller_action:=<controller_action> [trajectory_name:=<trajectory_name>], the [] means that the argument is optional")
        sys.exit()
    if base_link == None:
        print("No base link passed, please use ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] work_space:=<work_space> controller_action:=<controller_action> [trajectory_name:=<trajectory_name>], the [] means that the argument is optional")
        sys.exit()
    if end_effector_link == None:
        print("No end effector link passed, please use ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] work_space:=<work_space> controller_action:=<controller_action> [trajectory_name:=<trajectory_name>], the [] means that the argument is optional")
        sys.exit()
    if controller_action == None:
        print("No controller action passed, please use ros2 launch fanuc_m20ia_35m_planning_demo alone_start.launch.py joint_model_group_name:=<joint_model_group_name> base_link:=<base_link> end_effector_link:=<end_effector_link> [continous_planning:=<continous_planning>] work_space:=<work_space> controller_action:=<controller_action> [trajectory_name:=<trajectory_name>], the [] means that the argument is optional")
        sys.exit()

    return LaunchDescription([
        Node(
        package='fanuc_m20ia_35m_planning_demo',
        executable='fanuc_m20ia_35m_planning_demo',
        output='screen',
        parameters = [{'use_sim_time': use_sim_time, 'joint_model_group_name': jmg_param, 'base_link': base_link, 'end_effector_link': end_effector_link, 'continuos_planning': continuos_planning, 'work_space': work_space, 'controller_action': controller_action, 'trajectory_name': trajectory_name}]
    )
    ])