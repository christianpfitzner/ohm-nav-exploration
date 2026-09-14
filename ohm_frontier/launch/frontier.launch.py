"""The frontier node alone — for when the simulator and nav2 are already running elsewhere.

    ros2 launch ohm_frontier frontier.launch.py robot:=muster
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("min_frontier_cells", default_value="12"),
        DeclareLaunchArgument("avoid_radius", default_value="0.9"),
        Node(
            package="ohm_frontier", executable="frontier_node", name="frontier_node", output="screen",
            parameters=[{
                "robot": LaunchConfiguration("robot"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "min_frontier_cells": LaunchConfiguration("min_frontier_cells"),
                "avoid_radius": LaunchConfiguration("avoid_radius"),
            }],
        ),
    ])
