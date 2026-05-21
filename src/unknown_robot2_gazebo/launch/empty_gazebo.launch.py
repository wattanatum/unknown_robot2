import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg_share = get_package_share_directory("unknown_robot2_gazebo")
    description_pkg_share = get_package_share_directory("unknown_robot2_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(
        gazebo_pkg_share,
        "worlds",
        "empty.sdf"
    )

    xacro_file = os.path.join(
        description_pkg_share,
        "urdf",
        "unknown_robot2.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file
        ]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_file}"
        }.items()
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",

            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",

            "/camera/depth/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "rate": 50
            }
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_unknown_robot2",
        output="screen",
        arguments=[
            "-name", "unknown_robot2",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.20",
            "-Y", "0.0"
        ]
    )

    return LaunchDescription([
        gazebo,

        # Start bridge early so /clock is available
        TimerAction(period=1.0, actions=[bridge]),

        # Start TF publishers after /clock starts
        TimerAction(period=2.0, actions=[
            robot_state_publisher,
            joint_state_publisher
        ]),

        # Spawn robot into Gazebo
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])