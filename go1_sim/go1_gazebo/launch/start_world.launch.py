#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_go1_gazebo = get_package_share_directory("go1_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # ✅ This is the key fix: point at the package share directory itself
    go1_desc_share = get_package_share_directory("go1_description")  # .../share/go1_description

    # Resource search paths for gz-sim
    go1_models_path = os.path.join(pkg_go1_gazebo, "models")
    go1_gazebo_share = pkg_go1_gazebo

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_paths = [p for p in [existing, go1_desc_share, go1_models_path, go1_gazebo_share] if p]
    new_resource_path = ":".join(resource_paths)

    world_file_name = LaunchConfiguration("world_file_name")

    world_file_name_launch_arg = DeclareLaunchArgument(
        "world_file_name",
        default_value="cylinder_world.sdf",
        description="World SDF file name (must exist in go1_gazebo/worlds)",
    )

    world_path = PathJoinSubstitution([pkg_go1_gazebo, "worlds", world_file_name])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [world_path, " -r"],
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription([
        world_file_name_launch_arg,
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=new_resource_path),
        gz_sim,
        clock_bridge,
    ])
