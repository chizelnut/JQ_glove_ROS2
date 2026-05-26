"""Launch BOTH Juqiao gloves + RViz visualization.

The two gloves are placed side-by-side in `world` (~30 cm apart). Each has
its own namespace (`/glove_left`, `/glove_right`) and its own TF frame
(`glove_left_palm`, `glove_right_palm`).

Usage:
    ros2 launch juqiao_glove dual_glove_viz.launch.py \\
        left_port:=/dev/ttyACM1 right_port:=/dev/ttyACM0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    left_port = LaunchConfiguration("left_port")
    right_port = LaunchConfiguration("right_port")
    rviz = LaunchConfiguration("rviz")

    rviz_config = os.path.join(
        get_package_share_directory("juqiao_glove"),
        "rviz", "dual_glove.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument("left_port",  default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("right_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("rviz", default_value="true"),

        # Static TFs: left glove on the left, right glove on the right, 30cm apart.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="tf_world_glove_left",
            arguments=["-0.15", "0", "0", "0", "0", "0", "world", "glove_left_palm"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="tf_world_glove_right",
            arguments=["0.15", "0", "0", "0", "0", "0", "world", "glove_right_palm"],
        ),

        # Left glove: reader + viz
        Node(
            package="juqiao_glove", executable="glove_node", name="glove",
            namespace="glove_left",
            parameters=[{"port": left_port, "side": "left",
                         "frame_id": "glove_left_palm"}],
            output="screen",
        ),
        Node(
            package="juqiao_glove", executable="viz_node", name="viz",
            namespace="glove_left",
            parameters=[{"side": "left", "frame_id": "glove_left_palm"}],
        ),

        # Right glove: reader + viz
        Node(
            package="juqiao_glove", executable="glove_node", name="glove",
            namespace="glove_right",
            parameters=[{"port": right_port, "side": "right",
                         "frame_id": "glove_right_palm"}],
            output="screen",
        ),
        Node(
            package="juqiao_glove", executable="viz_node", name="viz",
            namespace="glove_right",
            parameters=[{"side": "right", "frame_id": "glove_right_palm"}],
        ),

        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
