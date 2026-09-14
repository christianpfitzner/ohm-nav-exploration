"""The `maze` hall: 6 × 6 m of corridor one cell wide, where a frontier is usually a dead end.

    ros2 launch ohm_frontier explore_maze.launch.py
    ros2 launch ohm_frontier explore_maze.launch.py headless:=true

The hall: 13 × 11 cells at 0.5 m — 6 × 6 m — with 82 wall cells, the robot in the bottom-left corner and a
goal marked in the file. It is the smallest hall the simulator has, and the only one where the robot's own
turning radius is a real constraint rather than a detail.

Why it is worth a run despite being tiny: in `rooms` a refused frontier is a room the robot could have
reached by another door, and the map grows anyway. Here a frontier is a corridor that ends, so the rules
that exist for failure are the ones on display — `min_frontier_cells` telling a gap between two beams from
a room that is there, and the blacklist letting the *next* clump have its turn instead of asking the same
dead end forever. Turn RViz on and watch how often the arrow changes for the same map: each change with an
unchanged map is a goal that was given up on, not a frontier that moved.

What to watch for: how long the run takes to say "no frontier on this map". A maze is finished when every
corridor end has either been reached or written off, and that sentence is the interesting one — a rule with
too small a `min_frontier_cells` never says it, because the crack in front of the wheel is always a
frontier.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    explore = os.path.join(get_package_share_directory("ohm_frontier"), "launch", "explore.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore),
            launch_arguments={"world": "maze", "robot": LaunchConfiguration("robot"),
                              "headless": LaunchConfiguration("headless"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
    ])
