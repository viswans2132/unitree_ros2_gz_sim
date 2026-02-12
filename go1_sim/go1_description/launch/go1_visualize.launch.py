import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_description = "go1_description"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")

    # --- Process xacro -> URDF string ---
    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")

    # If you have xacro args, pass them here via mappings={...}
    robot_description = xacro.process_file(xacro_file).toxml()

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }],
    )

    # Use ONLY for visualization without simulation
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_joint_state_publisher),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_config = os.path.join(pkg_path, "rviz", "go1_vis.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config],
    )

    # --- Launch args ---
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            # For gz simulation, you usually want this false.
            default_value="false",
            description="Publish joint_states from a fixed URDF (for non-sim visualization).",
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
