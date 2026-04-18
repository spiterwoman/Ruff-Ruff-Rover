from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("rover_bringup"),
        "config",
        "rover_params.yaml",
    )

    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    device_arg = DeclareLaunchArgument("micro_ros_device", default_value="/dev/ttyACM0")
    baud_arg = DeclareLaunchArgument("micro_ros_baud", default_value="115200")

    agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=[
            "serial",
            "--dev",
            LaunchConfiguration("micro_ros_device"),
            "-b",
            LaunchConfiguration("micro_ros_baud"),
        ],
        output="screen",
    )

    whistle = Node(
        package="rover_behavior",
        executable="mic_whistle_node",
        name="mic_whistle_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    turn_test = Node(
        package="rover_behavior",
        executable="turn_to_whistle_test_node",
        name="turn_to_whistle_test_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([params_arg, device_arg, baud_arg, agent, whistle, turn_test])
