from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    clock_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    # Controllers to LOAD+CONFIGURE (inactive)
    controllers = [
        "joint_state_broadcaster",
        "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
        "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
        "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller",
        "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller",
    ]

    spawners = [
        ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "spawner",
                c, "-c", "/controller_manager",
                "--inactive",
                "--controller-manager-timeout", "10",
            ],
            output="screen",
        )
        for c in controllers
    ]

    # Symmetric activation order (after all are loaded/configured)
    activate_order = [
        # hips
        "FR_hip_controller", "FL_hip_controller", "RR_hip_controller", "RL_hip_controller",
        # calves
        "FR_calf_controller", "FL_calf_controller", "RR_calf_controller", "RL_calf_controller",
        # thighs
        "FR_thigh_controller", "FL_thigh_controller", "RR_thigh_controller", "RL_thigh_controller",
    ]

    activators = [
        ExecuteProcess(
            cmd=["ros2", "control", "set_controller_state", name, "active"],
            output="screen",
        )
        for name in activate_order
    ]

    # Chain spawners: load/configure all controllers first
    chained = []
    for i in range(len(spawners) - 1):
        chained.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawners[i],
                    on_exit=[spawners[i + 1]],
                )
            )
        )

    # After the LAST spawner exits, start activation chain
    activate_chain = []
    if activators:
        activate_chain.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawners[-1],
                    on_exit=[activators[0]],
                )
            )
        )
        for i in range(len(activators) - 1):
            activate_chain.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=activators[i],
                        on_exit=[activators[i + 1]],
                    )
                )
            )

    # Start first spawner after a short delay
    start_first = TimerAction(period=3.0, actions=[spawners[0]])

    return LaunchDescription([
        clock_bridge,
        start_first,
        *chained,
        *activate_chain,
    ])
