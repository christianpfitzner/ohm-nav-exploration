"""The `rooms` hall: rooms behind doorways, so one frontier per room nobody has entered.

    ros2 launch ohm_frontier explore_rooms.launch.py
    ros2 launch ohm_frontier explore_rooms.launch.py robot:=carlo headless:=true

Three lines of substance — the hall's name and the two arguments worth typing at a lecture. The
simulator, the mapper, nav2, this package's node and this package's RViz view all come from
`explore.launch.py`, which is also what keeps the tf arrangement stated in one file instead of six.
Anything other than these three arguments is typed there (`ros2 launch ohm_frontier
explore.launch.py --show-args`), and a hall other than this one is typed there too, because a file whose
`world:=` outbids the hall its own name promises is how a demo starts in the wrong hall.

The hall: 44 × 32 cells at 0.5 m, so 22 × 16 m, 300 wall cells, and the robot at (5.0, 4.5).

Why this one is the default: the walls have doorways in them, and a doorway is where a frontier lives.
Each unentered room therefore arrives as its own clump of cells, which is the picture the algorithm is
explained with, and the ranking has something to choose between — the wide doorway down the hall against
the crack beside the wheel.

What to watch: the list of frontiers in RViz, one clump per room, and which arrow the node picks. Then the
arrows disappearing one at a time — `reached (10.51, 12.33)`, `reached (12.21, 11.33)`, five of them inside
two minutes headless on this machine — with the map filling in behind the robot. If the arrows change while
the robot stands still, the driving is not connected rather than the ranking being wrong: the fourth pit in
the README is that, and it looks like a controller that never commands anything.
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
            launch_arguments={"world": "rooms", "robot": LaunchConfiguration("robot"),
                              "headless": LaunchConfiguration("headless"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
    ])
