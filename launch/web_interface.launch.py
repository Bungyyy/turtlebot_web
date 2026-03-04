"""
ROS2 Launch file for TurtleBot3 Web Interface.

Launches:
  1. rosbridge_websocket (for browser ↔ ROS communication)
  2. web_video_server    (optional camera stream fallback)

Usage:
  ros2 launch turtlebot_web web_interface.launch.py
  ros2 launch turtlebot_web web_interface.launch.py rosbridge_port:=9090
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port", default_value="9090",
        description="Port for the rosbridge WebSocket server"
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("rosbridge_port"),
            "unregister_timeout": 60.0,
        }],
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription([
        rosbridge_port_arg,
        LogInfo(msg="Starting TurtleBot3 Web Interface bridge..."),
        rosbridge_node,
        rosapi_node,
    ])
