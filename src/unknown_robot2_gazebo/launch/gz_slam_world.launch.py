from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare("unknown_robot2_gazebo"),
        "worlds",
        "auto_test_world.sdf"
    ])

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            world_file
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo
    ])