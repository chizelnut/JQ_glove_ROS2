"""Launch one Juqiao glove reader + RViz visualization.

Usage:
    ros2 launch juqiao_glove single_glove_viz.launch.py \\
        port:=/dev/ttyACM0 side:=right
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    side = LaunchConfiguration("side")
    rviz = LaunchConfiguration("rviz")

    rviz_config = os.path.join(
        get_package_share_directory("juqiao_glove"),
        "rviz", "glove.rviz",
    )

    ns = ["glove_", side]
    frame = ["glove_", side, "_palm"]

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("side", default_value="right",
                              description="'left' or 'right'"),
        DeclareLaunchArgument("rviz", default_value="true"),

        # Static TF: world -> glove_<side>_palm at the origin
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=["tf_world_glove_", side],
            arguments=["0", "0", "0", "0", "0", "0", "world", frame],
        ),

        Node(
            package="juqiao_glove",
            executable="glove_node",
            name="glove",
            namespace=ns,
            parameters=[{"port": port, "side": side, "frame_id": frame}],
            output="screen",
        ),

        Node(
            package="juqiao_glove",
            executable="viz_node",
            name="viz",
            namespace=ns,
            parameters=[{"side": side, "frame_id": frame}],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=__import__("launch.conditions",
                                 fromlist=["IfCondition"]).IfCondition(rviz),
        ),
    ])
