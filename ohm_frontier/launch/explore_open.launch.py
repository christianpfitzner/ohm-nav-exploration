"""The `open` hall: 30 × 20 m with nothing in it but the four walls.

    ros2 launch ohm_frontier explore_open.launch.py
    ros2 launch ohm_frontier explore_open.launch.py headless:=true

The hall: 60 × 40 cells at 0.5 m, and the only wall cells in the file are its boundary. One spawn at
(2.0, 6.0), a second marker at the far end, no obstacles, no painted lanes, no furniture.

Why it is in the set: it is the run where the ranking has nothing to choose between, and that is the point.
Everything unknown is one clump against the far wall, so the first goal is nearly a formality and everything
the students see is the loop itself — scan, map, frontier, goal, drive, map again. Use it first and `rooms`
second: with one frontier there is no ranking to argue about, so the argument can be about what a frontier
*is* (free floor next to unknown floor, and unknown is not a wall) instead of about which of five openings
is best.

What to watch: the frontier line marching with the map. In an empty hall the unknown territory is one big
block, so the clump — and the arrow at its opening — moves every time the mapper takes in a scan. A run that
looks like it is chasing the same arrow across the hall is not flip-flopping: the frontier moved. That is the
difference the goal state machine in `frontier_node.py` is asked to keep visible, and it is easiest to see
here, where a wrong claim is obvious within two seconds.
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
            launch_arguments={"world": "open", "robot": LaunchConfiguration("robot"),
                              "headless": LaunchConfiguration("headless"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
    ])
