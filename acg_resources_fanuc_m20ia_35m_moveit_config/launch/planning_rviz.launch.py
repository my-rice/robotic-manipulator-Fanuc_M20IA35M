from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fanuc", package_name="acg_resources_fanuc_m20ia_35m_moveit_config").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)


def generate_moveit_rviz_launch(moveit_config):
    "This function is redefined to receive relative paths for RViz config files"

    """Launch file for rviz"""
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="planning_rviz.rviz",
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    rviz_config = LaunchConfiguration("rviz_config")
    rviz_config_path = PathJoinSubstitution([
        str(moveit_config.package_path),
        'config',
        rviz_config,
    ])

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", rviz_config_path],
        parameters=rviz_parameters,
    )

    return ld