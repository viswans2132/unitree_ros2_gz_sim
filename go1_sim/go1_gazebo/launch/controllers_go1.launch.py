from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    controllers = [
        "joint_state_broadcaster",
        # "imu_sensor_broadcaster",  # enable later when IMU is wired
        # "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
        # "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
        # "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller",
        # "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller",
    ]

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[c, "-c", "/controller_manager"],
            output="screen",
            parameters=[{
                "use_sim_time": True,
            }],
        )
        for c in controllers
    ]

    # Delay start to allow gz_ros2_control to initialize controller_manager
    return LaunchDescription([
        clock_bridge,
        TimerAction(period=3.0, actions=spawners)
    ])
