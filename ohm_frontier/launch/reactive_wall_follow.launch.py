"""Wall following in the simulator: the hall plus one reactive node, no map, no planner, no SLAM.

    ros2 launch ohm_frontier reactive_wall_follow.launch.py
    ros2 launch ohm_frontier reactive_wall_follow.launch.py world:=production robot:=carlo

The wall follower of `wall_following.py` against a real hall. `rooms` is the default because it is the
smallest hall that shows both states: the robot is set down in a pocket with a wall on its right, follows
it out through the doorway, and loses it in the open part of the hall — where the node stops, turns on
the spot and says so, which is the moment worth putting on the projector. `production` is the second
choice and the better one for the shelf aisles: there the rule follows an aisle for a hundred metres, and
the failure to show is at the end of the aisle, where the wall stops and the robot keeps turning right.

Nothing here asks for the tf tree or the lidar's no-echo dialect that `explore.launch.py` is careful
about, and that is deliberate rather than an oversight: with no mapper in the graph the simulator keeps
its own `map → <robot>/odom` edge with nobody to collide with, and this node reads the range array
directly, so it does not care whether a missing echo arrives as 8.0 m or as infinity. It handles both.

The simulator's own window is the display for this demo, so RViz stays off by default;
`rviz:=true` adds it if a scan plot is wanted beside the hall.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: where the simulator lives when it is not installed as a ROS package of its own
FALLBACK_SIM = os.path.expanduser("~/git/mecanum-lab")


def sim_dir() -> str:
    """The simulator's share directory, or its checkout when it is not installed.

    `ros2 launch <path>/launch/lab.launch.py` takes a path as readily as a package name, so a checkout that
    was never built into this workspace still starts. `explore.launch.py` does the same and explains why.
    """
    try:
        return get_package_share_directory("mecanum_lab")
    except PackageNotFoundError:
        return FALLBACK_SIM


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("world", default_value="rooms", description="hall to follow the walls of"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="without the simulator window — of little use for this demo"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="the simulator's RViz too; the hall window is the display here"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("wall_distance", default_value="0.40",
                              description="m, the gap to keep on the right hand"),
        DeclareLaunchArgument("speed", default_value="0.35", description="m/s"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([sim_dir(), "launch", "lab.launch.py"])),
            launch_arguments={"robot": robot, "world": LaunchConfiguration("world"),
                              "headless": LaunchConfiguration("headless"),
                              "use_sim_time": LaunchConfiguration("use_sim_time"),
                              "rviz": LaunchConfiguration("rviz")}.items(),
        ),
        Node(
            package="ohm_frontier", executable="wall_following", name="wall_follower", output="screen",
            parameters=[{
                "robot": robot,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # a launch argument arrives as text and these two are numbers in the node
                "wall_distance": ParameterValue(LaunchConfiguration("wall_distance"), value_type=float),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
            }],
        ),
    ])
