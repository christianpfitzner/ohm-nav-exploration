"""The deciding part alone: simulator, mapper and the frontier node — no navigation stack at all.

    ros2 launch ohm_frontier explore_no_nav2.launch.py
    ros2 launch ohm_frontier explore_no_nav2.launch.py robot:=carlo headless:=true

It is `explore.launch.py` with `nav2:=false`, which leaves `bt_navigator`, the costmaps and the controller
out of the graph and everything else in it: the simulator in `rooms`, slam_toolbox mapping its lidar, this
package's node deciding, and this package's RViz view.

Nothing drives. The node does not notice — it sends the `NavigateToPose` goal to a topic with no server,
says so in one line, and puts the same decision on `/frontier_goal`, which is where RViz reads it from. So
this is the run in which the *decisions* are the subject and nobody has to wait for a robot:

* the frontier list, in RViz, changing as the map grows — with no planner refusing goals, every change of
  the arrow is the ranking changing its mind, which is the thing to argue about;
* the timeout and the blacklist working with nothing to blame: the goal is sent, the robot never moves
  (nobody is driving it), the goal is given up on when its timeout runs out, its surroundings are written
  off, and the next clump gets its turn. That is the whole failure path of the node, in seconds rather than
  minutes, and not hidden behind a controller whose parameters still need work;
* how much of a run is nav2 at all: the map, the frontiers and the choice are unchanged from
  `explore_rooms.launch.py`, and the only thing missing is the driving.

To put motion into it without bringing the stack back, drive the simulator by hand — the keyboard drives
when no controller node is started — and watch the frontier list answer the pose the keyboard happens to
pick. Anything publishing on `/<robot>/cmd_vel` does just as well, which is what the reactive demos in this
package are for.
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
            launch_arguments={"world": "rooms", "nav2": "false", "robot": LaunchConfiguration("robot"),
                              "headless": LaunchConfiguration("headless"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
    ])
