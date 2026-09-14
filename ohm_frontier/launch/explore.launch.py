"""One command for a whole exploration run: simulator, SLAM, navigation stack, frontier node.

    ros2 launch ohm_frontier explore.launch.py
    ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo

Nothing in the simulator is changed or started differently for this: it already publishes `/scan`,
`/odom` and the tf tree, and that is all SLAM and nav2 ask a robot for.

**The one thing that took thought: who publishes `map → <robot>/odom`.** The simulator publishes it
(`mecanum_lab/tf_bcast.py`, and the parent name `map` is fixed there — by intent, because `map →
base_link` being the drifting odometry is the point of the Kalman lab). slam_toolbox wants to publish that
same edge, and a frame with two parents is not a tf tree, so one of the two has to move. The simulator
does not move for anyone, so slam_toolbox gets the odd job instead:

    slam_map  →  map  →  <robot>/odom  →  <robot>/base_link  →  <robot>/laser
      slam       sim         sim              sim                     sim

`odom_frame: map` hands slam_toolbox the hall frame as its odometry reference — which is what an odometry
frame is for a real robot anyway, a pose that drifts — and slam_toolbox then publishes its own frame above
it. `map → base_link` stays the students' drifting odometry, `slam_map → base_link` is the SLAM answer, and
everything that plans or draws from the map uses `slam_map`. That is also the frame the frontier node reads
out of the map message rather than assuming, and what `config/nav2_rooms.yaml` is written for.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

#: where the simulator lives when it is not installed as a ROS package
FALLBACK_SIM = os.path.expanduser("~/git/mecanum-lab")


def share(package: str, why: str = "") -> str:
    """Share directory of an installed package, with a sentence about what to do when it is missing.

    `launch_ros.substitutions.FindPackageShare` is the launch-file way of asking the same thing.
    `launch_ros.actions.FindPackageShare` — which is what this line used to import — does not exist, on
    Jazzy or on Kilted. Asked from here, `ament_index_python` answers the same way on every distro.
    """
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise PackageNotFoundError(
            f"'{package}' is not on the ROS path{' — ' + why if why else ''}. "
            f"Source the workspace it was built in, or install it:"
            f" sudo apt install ros-$ROS_DISTRO-{package}")


def _sim_dir() -> str:
    """Where `launch/lab.launch.py` is: the installed package if there is one, the checkout otherwise."""
    try:
        return get_package_share_directory("mecanum_lab")
    except PackageNotFoundError:
        return FALLBACK_SIM


def generate_launch_description():
    frontiers = share("ohm_frontier")
    arguments = [
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("world", default_value="rooms", description="hall of the simulator to map"),
        DeclareLaunchArgument("sim_dir", default_value=_sim_dir(),
                              description="the mecanum-lab checkout, or its installed share directory"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="let the simulator open its own window as well"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(frontiers, "config",
                                                                        "slam_toolbox.yaml")),
        DeclareLaunchArgument("nav2_params", default_value=os.path.join(frontiers, "config",
                                                                        "nav2_rooms.yaml")),
    ]
    robot, sim, sim_time = (LaunchConfiguration(n) for n in ("robot", "sim_dir", "use_sim_time"))

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim, "launch", "lab.launch.py"])),
        launch_arguments={"world": LaunchConfiguration("world"), "robot": robot,
                          "headless": "true", "use_sim_time": sim_time,
                          "rviz": LaunchConfiguration("rviz")}.items(),
    )

    slam = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
        output="screen",
        parameters=[LaunchConfiguration("slam_params"), {
            "use_sim_time": sim_time,
            "map_frame": "slam_map",            # the frame above the simulator's, see module docstring
            "odom_frame": "map",
            "base_frame": [robot, "/base_link"],
            "scan_topic": ["/", robot, "/scan"],
            "senser": {"topic": ["/", robot, "/scan"]},   # the same setting under the newer name
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share("nav2_bringup"), "launch",
                                                   "navigation_launch.py")),
        launch_arguments={"params_file": LaunchConfiguration("nav2_params"),
                          "use_sim_time": sim_time}.items(),
    )

    frontiers_node = Node(
        package="ohm_frontier", executable="frontier_node", name="frontier_node", output="screen",
        parameters=[{"robot": robot, "use_sim_time": sim_time}],
    )

    return LaunchDescription(arguments + [simulator, slam, navigation, frontiers_node])
