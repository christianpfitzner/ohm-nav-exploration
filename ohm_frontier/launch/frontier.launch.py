"""The frontier node alone — for when the simulator and nav2 are already running elsewhere.

    ros2 launch ohm_frontier frontier.launch.py robot:=muster
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                # a launch argument arrives as text, and these two are numbers in the node
                "min_frontier_cells": ParameterValue(LaunchConfiguration("min_frontier_cells"),
                                                     value_type=int),
                "avoid_radius": ParameterValue(LaunchConfiguration("avoid_radius"), value_type=float),
            }],
        ),
    ])
