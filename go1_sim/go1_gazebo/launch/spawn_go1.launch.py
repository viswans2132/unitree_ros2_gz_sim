import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
import xacro

def dump_urdf(context, *args, **kwargs):
    # TODO: point this at the same top-level xacro your launch uses
    xacro_path = os.path.join(
        get_package_share_directory("go1_description"),
        "urdf",
        "go1.urdf.xacro",
    )

    doc = xacro.process_file(xacro_path)  # add mappings=... if you pass xacro args
    urdf_xml = doc.toprettyxml(indent="  ")

    out_path = "/tmp/go1_spawned.urdf"
    with open(out_path, "w") as f:
        f.write(urdf_xml)

    # Key debug: check for model://
    has_model = "model://go1_description" in urdf_xml
    has_pkg = "package://go1_description" in urdf_xml

    print("\n[DEBUG] Wrote URDF to:", out_path)
    print("[DEBUG] Contains 'model://go1_description' ?", has_model)
    print("[DEBUG] Contains 'package://go1_description' ?", has_pkg)

    # Print the first few mesh lines so we see what URIs actually look like
    lines = [ln for ln in urdf_xml.splitlines() if "mesh filename" in ln]
    print("[DEBUG] First 20 mesh lines:")
    for ln in lines[:20]:
        print("   ", ln.strip())

    return []


def generate_launch_description():
    # go1_share = os.path.join(get_package_prefix("go1_description"), "share")

    # SetEnvironmentVariable(
    #     name="GZ_SIM_RESOURCE_PATH",
    #     value=go1_share + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    # )
    # Launch args
    world_file_name = LaunchConfiguration("world_file_name")
    urdf_file = LaunchConfiguration("urdf_file")  # kept for compatibility with your visualize launch
    # robot_name = LaunchConfiguration("robot_name") 
    robot_name = "GO1"

    use_sim_time = LaunchConfiguration("use_sim_time")


    world_file_name_arg = DeclareLaunchArgument(
        "world_file_name",
        default_value="cylinder_world.sdf",
    )

    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value="robot.xacro",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    go1_share_parent = os.path.join(get_package_prefix('go1_description'), 'share')
    # print(go1_share_parent)

    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=go1_share_parent + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    # print(os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # robot_name_arg = DeclareLaunchArgument(
    #     "robot_name",
    #     default_value="GO1",
    # )

    # Paths
    go1_gazebo_share = get_package_share_directory("go1_gazebo")
    go1_desc_share = get_package_share_directory("go1_description")
    robot_xacro_path = os.path.join(go1_desc_share, "xacro", "robot.xacro")
    robot_description = xacro.process_file(robot_xacro_path).toprettyxml(indent="  ")

    world_path = os.path.join(go1_gazebo_share, "worlds", world_file_name.perform({}) if False else "")
    # NOTE: We can't resolve LaunchConfiguration at generation-time without OpaqueFunction.
    # So we pass the world path via gz_args using substitutions below instead.

    # Pose
    position = [0.0, 0.0, 0.4]
    orientation_rpy = [0.0, 0.0, 0.0]

    # 1) Start gz sim with the world
    # Use ros_gz_sim's gz_sim.launch.py
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution([
                    get_package_share_directory("go1_gazebo"),
                    "worlds",
                    world_file_name,
                ]),
                " -r"
            ]
        }.items(),
    )

    # 2) Start robot_state_publisher + RViz (your existing visualize launch)
    # This publishes /robot_description which ros_gz_sim create can use.
    visualize_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("go1_description"),
                "launch",
                "go1_visualize.launch.py",
            )
        ),
        launch_arguments={
            "use_joint_state_publisher": "False",
            "use_sim_time": "True",
            "urdf_file": urdf_file,
        }.items(),
    )

    # 3) Spawn robot into gz from /robot_description
    # ros_gz_sim create reads the URDF string and gz converts it to SDF internally.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", robot_name,
            "-topic", "robot_description",
            "-x", str(position[0]), "-y", str(position[1]), "-z", str(position[2]),
            "-R", str(orientation_rpy[0]), "-P", str(orientation_rpy[1]), "-Y", str(orientation_rpy[2]),
        ],
    )

    # 4) Controllers
    launch_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("go1_gazebo"),
                "launch",
                "controllers_go1.launch.py",
            )
        )
    )

    # Make controllers start AFTER the robot is spawned (recommended)
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[launch_ros2_control],
        )
    )

    # 5) Your odom TF publisher (keep)
    odom_tf_publisher_node = Node(
        package="go1_navigation",
        executable="nav_tf_publisher",
        name="odom_transform_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    lidar_gz_topic = "/world/default/model/GO1/link/laser_link/sensor/lidar/scan"

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        output="screen",
        arguments=[
            f"{lidar_gz_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            # remap ROS-side topic name to a nice standard
            (
                lidar_gz_topic,
                "/scan"
            )
        ],
    )
    odom_gz_topic = "/world/default/model/GO1/odometry"
    odom_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odom_bridge",
        output="screen",
        arguments=[
            [f"{odom_gz_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        ],
        remappings=[
            (odom_gz_topic,
                "/odom"
            )
        ],
    )

    img = "/world/default/model/GO1/link/camera_face/sensor/camera_face/image"
    info = "/world/default/model/GO1/link/camera_face/sensor/camera_face/camera_info"
    depth = "/world/default/model/GO1/link/camera_face/sensor/camera_face/depth_image"
    points = "/world/default/model/GO1/link/camera_face/sensor/camera_face/points"

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            f"{img}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{depth}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{points}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            (img, "/camera_face/image"),
            (info, "/camera_face/camera_info"),
            (depth, "/camera_face/depth_image"),
            (points, "/camera_face/points"),
        ],
    )

    bridges_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[lidar_bridge, camera_bridge],
        )
    )

    robot_description_publisher = Node(
        package="go1_gazebo",
        executable="robot_description_publisher",
        name="robot_description_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )



    return LaunchDescription([
        set_gz_path,
        world_file_name_arg,
        urdf_file_arg,
        use_sim_time_arg,
        gz_sim_launch,
        visualize_robot,
        # robot_description_publisher,
        spawn_robot,
        bridges_after_spawn,
        controllers_after_spawn,
        # odom_tf_publisher_node,
    ])
