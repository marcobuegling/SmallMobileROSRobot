"""Launch file for the command_mux node.

Loads parameters from ``config/command_mux.yaml`` by default; override with
the ``params_file`` launch argument if you need a different configuration.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the command_mux node.

    Returns:
        The assembled `LaunchDescription`.
    """
    default_params_file = os.path.join(
        get_package_share_directory("robot_py_control"),
        "config",
        "command_mux.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the command_mux parameters YAML file.",
    )

    command_mux_node = Node(
        package="robot_py_control",
        executable="command_mux_node",
        name="command_mux",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    actions: List = [params_file_arg, command_mux_node]
    return LaunchDescription(actions)
