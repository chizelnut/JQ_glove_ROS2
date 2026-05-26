"""Launch one Juqiao glove reader. No visualization.

Usage:
    ros2 launch juqiao_glove single_glove.launch.py \\
        port:=/dev/ttyACM0 side:=right
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    side = LaunchConfiguration("side")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",
                              description="Serial device path"),
        DeclareLaunchArgument("side", default_value="right",
                              description="'left' or 'right'"),
        Node(
            package="juqiao_glove",
            executable="glove_node",
            name="glove",
            namespace=["glove_", side],
            parameters=[{"port": port, "side": side}],
            output="screen",
        ),
    ])
