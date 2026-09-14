"""The `arena` hall: an empty 24 × 16 m hall with lanes painted on the floor — and a lesson about maps.

    ros2 launch ohm_frontier explore_arena.launch.py
    ros2 launch ohm_frontier explore_arena.launch.py headless:=true

The hall: 48 × 32 cells at 0.5 m, its wall cells are its boundary only, and 216 cells of it are `|` — which
in `mecanum_lab/worlds.py` is *painted floor*: free to drive over, drawn by nothing, and therefore absent
from every measurement of it. Two spawns, (7.0, 4.5) and (16.0, 4.5), and a goal marker in the middle.

Why it is worth showing after `open`: the file looks like a hall with partitions in it, and the map that
comes out of the lidar does not have them. Nothing is broken — a painted line reflects nothing back, so the
mapper is right that the floor is free, and nav2 is right to plan across it. The students who read
`worlds/arena.txt` before running it will have expected walls, and the two minutes afterwards are the whole
point: **the map is what the sensors say, not what the world file says**, which is the same reason the
frontier node reads the frame out of the map message rather than assuming one, and the reason a mapper that
has not been given a reason to integrate a scan is not broken but waiting.

What to watch: the map, and the absence of anything where the lanes are. Then compare with the simulator's
window beside it: same hall, two answers, both correct about different things. A second robot in this hall
would need the simulator's own `robots:=` argument and a nav2 configuration for more than one robot;
`config/nav2_rooms.yaml` is written for exactly one, which is why these files take one `robot:=`.
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
            launch_arguments={"world": "arena", "robot": LaunchConfiguration("robot"),
                              "headless": LaunchConfiguration("headless"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
    ])
