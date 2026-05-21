from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("unknown_robot2_description"),
        "urdf",
        "unknown_robot2.urdf.xacro"
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare("unknown_robot2_gazebo"),
        "worlds",
        "auto_test_world.sdf"
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file
        ]),
        value_type=str
    )

    # Kill old Gazebo / bridge / odom_to_tf before starting a new simulation.
    # [g]z, [p]arameter_bridge, etc. prevents pkill from killing itself.
    kill_old_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                "pkill -f '[g]z sim' || true; "
                "pkill -f '[p]arameter_bridge' || true; "
                "pkill -f '[s]pawn_unknown_robot2' || true; "
                "pkill -f '[o]dom_to_tf' || true; "
                "sleep 2"
            )
        ],
        output="screen"
    )

    # Gazebo GUI enabled
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_file
            ]
        }.items()
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

    # Keep this because your wheel TF needs joint_states.
    # Low rate reduces CPU load.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "rate": 15.0
            }
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_unknown_robot2",
        output="screen",
        arguments=[
            "-world", "auto_test_world",
            "-name", "unknown_robot2",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.25",
            "-Y", "0.0"
        ]
    )

    # Lightweight bridge for Nav2 + ArUco parking.
    #
    # Important:
    # Do NOT bridge Gazebo /tf:
    #   "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    #
    # That bridge is heavy and can make RViz/Gazebo lag.
    # Instead, odom_to_tf below publishes only:
    #   odom -> base_footprint
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # ROS 2 -> Gazebo
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",

            # Gazebo -> ROS 2 for Nav2
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

            # Gazebo -> ROS 2 for ArUco parking
            "/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

            # Gazebo clock -> ROS 2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # Do NOT enable this unless absolutely needed.
            # It is very heavy and not needed for normal ArUco detection.
            # "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ]
    )

    # Lightweight TF publisher.
    # Converts /odom topic into TF:
    #   odom -> base_footprint
    #
    # This replaces the heavy Gazebo /tf bridge.
    odom_to_tf = Node(
        package="unknown_robot2_gazebo",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "odom_topic": "/odom",
                "parent_frame": "odom",
                "child_frame": "base_footprint",
            }
        ]
    )

    return LaunchDescription([
        # 1. Kill old Gazebo / bridge / odom_to_tf
        kill_old_gazebo,

        # 2. Start Gazebo GUI and robot publishers
        TimerAction(
            period=3.0,
            actions=[
                gazebo,
                robot_state_publisher,
                joint_state_publisher,
            ]
        ),

        # 3. Spawn robot after Gazebo world is ready
        TimerAction(
            period=7.0,
            actions=[
                spawn_robot
            ]
        ),

        # 4. Start bridge and lightweight TF node after robot spawn
        TimerAction(
            period=8.0,
            actions=[
                bridge,
                odom_to_tf,
            ]
        ),
    ])