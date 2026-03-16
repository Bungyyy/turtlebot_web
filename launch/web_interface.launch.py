"""
ROS2 Humble Launch file for Unitree Go2 Web Interface.

Launches:
  1. rosbridge_websocket (for browser <-> ROS2 communication)
  2. rosapi_node          (exposes topic/node info to the web UI)

Prerequisites:
  sudo apt install ros-humble-rosbridge-suite

Usage:
  ros2 launch turtlebot_web web_interface.launch.py
  ros2 launch turtlebot_web web_interface.launch.py rosbridge_port:=9090
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="Port for the rosbridge WebSocket server",
    )

    # Use the official rosbridge_server XML launch file
    rosbridge_dir = get_package_share_directory("rosbridge_server")
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_dir, "launch", "rosbridge_websocket_launch.xml")
        ),
        launch_arguments={"port": LaunchConfiguration("rosbridge_port")}.items(),
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription(
        [
            rosbridge_port_arg,
            LogInfo(msg="Starting Unitree Go2 Web Interface (ROS2 Humble)..."),
            rosbridge_launch,
            rosapi_node,
        ]
    )
