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

    bridge_client = Node(
        package="rover_vision",
        executable="vision_bridge_client_node",
        name="vision_bridge_client_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([params_arg, bridge_client])
