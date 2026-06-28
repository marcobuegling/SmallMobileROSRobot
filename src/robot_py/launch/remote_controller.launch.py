# Use this launch file in combination with 'remote_robot.launch.py' for operating the robot from a remote device
# Start this launch file only the remote device used for controlling the robot

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node