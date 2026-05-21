from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("unknown_robot2_navigation"),
        "config",
        "nav2_map_params.yaml"
    ])

    map_file = PathJoinSubstitution([
        FindPackageShare("unknown_robot2_navigation"),
        "maps",
        "auto_test_map.yaml"
    ])

    bt_xml_file = PathJoinSubstitution([
        FindPackageShare("unknown_robot2_navigation"),
        "behavior_trees",
        "nav_to_pose_clear_costmap.xml"
    ])

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "yaml_filename": map_file,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "default_nav_to_pose_bt_xml": bt_xml_file,
            }
        ],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel_smoothed"),
        ],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("cmd_vel_in", "cmd_vel_smoothed"),
            ("cmd_vel_out", "cmd_vel"),
        ],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }
        ],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                    "collision_monitor",
                ],
            }
        ],
    )

    return LaunchDescription([
        # Start Nav2 nodes after Gazebo + odom_to_tf are ready
        TimerAction(
            period=10.0,
            actions=[
                map_server,
                amcl,

                controller_server,
                smoother_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                velocity_smoother,
                collision_monitor,
            ]
        ),

        # Start lifecycle managers after Nav2 nodes exist
        TimerAction(
            period=13.0,
            actions=[
                lifecycle_manager_localization,
                lifecycle_manager_navigation,
            ]
        ),
    ])