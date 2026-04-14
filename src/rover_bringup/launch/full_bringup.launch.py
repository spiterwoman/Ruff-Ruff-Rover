from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("rover_bringup")
    default_params = os.path.join(bringup_share, "config", "rover_params.yaml")
    hardware_launch = os.path.join(bringup_share, "launch", "hardware_bringup.launch.py")

    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    device_arg = DeclareLaunchArgument("micro_ros_device", default_value="/dev/ttyACM0")
    baud_arg = DeclareLaunchArgument("micro_ros_baud", default_value="115200")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "micro_ros_device": LaunchConfiguration("micro_ros_device"),
            "micro_ros_baud": LaunchConfiguration("micro_ros_baud"),
        }.items(),
    )

    behavior = Node(
        package="rover_behavior",
        executable="follow_whistler_node",
        name="follow_whistler_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([params_arg, device_arg, baud_arg, hardware, behavior])
