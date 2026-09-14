"""One command for a whole exploration run: simulator, navigation stack, frontier node.

    ros2 launch ohm_frontier explore.launch.py
    ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo

The simulator is started exactly as it is; nothing in it is changed for exploration. It already
publishes what a navigation stack asks a robot for — `/scan`, `/odom`, the tf tree `map → <robot>/odom →
<robot>/base_link → <robot>/laser` — and its `map` frame is the hall's own origin, so the map built from
the lidar is in the same metres as the hall.

nav2 is included through its own `navigation_launch.py` only, without `localization_launch.py`: there is
no AMCL and no map_server here, because the robot knows where it is (the simulator tells it) and the
frontier node publishes the map.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import FindPackageShare, Node

#: where the simulator lives when it is not installed as a ROS package
FALLBACK_SIM = os.path.expanduser("~/git/mecanum-lab")


def generate_launch_description():
    here = get_package_share_directory("ohm_frontier")
    try:
        default_sim = get_package_share_directory("mecanum_lab")
    except PackageNotFoundError:
        default_sim = FALLBACK_SIM

    robot = LaunchConfiguration("robot")
    sim = LaunchConfiguration("sim_dir")
    sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("world", default_value="rooms",
                              description="hall of the simulator to explore"),
        DeclareLaunchArgument("sim_dir", default_value=default_sim,
                              description="the mecanum-lab checkout or its installed share directory"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="let the simulator open its own window as well"),
        DeclareLaunchArgument("nav2_params", default_value=os.path.join(here, "config",
                                                                        "nav2_rooms.yaml")),
    ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim, "launch", "lab.launch.py"])),
        launch_arguments={"world": LaunchConfiguration("world"), "robot": robot,
                          "headless": "true", "use_sim_time": sim_time,
                          "rviz": LaunchConfiguration("rviz")}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])),
        launch_arguments={"params_file": LaunchConfiguration("nav2_params"),
                          "use_sim_time": sim_time,
                          "nav_base_frame": PathJoinSubstitution([robot, "base_link"])}.items(),
    )

    frontiers = Node(
        package="ohm_frontier", executable="frontier_node", name="frontier_node", output="screen",
        parameters=[{"robot": robot, "use_sim_time": sim_time}],
    )

    return LaunchDescription(arguments + [simulator, navigation, frontiers])
