"""One command for a whole exploration run: simulator, SLAM, navigation stack, frontier node.

    ros2 launch ohm_frontier explore.launch.py
    ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo

Nothing in the simulator is changed or started differently for this. What it does need is one word about
tf, because both it and a mapping node want to publish the same edge:

    map  →  <robot>/odom  →  <robot>/base_link  →  <robot>/laser
     slam       sim                sim                  sim

By default the simulator publishes `map → <robot>/odom` as well — `map → base_link` being the drifting
odometry is what its Kalman lab is built on. Two publishers on that one edge would give `<robot>/odom` two
parents, and a tf tree with two parents is not a tree, so this launch file asks for the other tree with
`tf_tree:=slam` (`--set tf.tree=slam` on the command line, `mecanum_lab/tf_bcast.py` implementing it). The
simulator then publishes from `<robot>/odom` down and calls the frame of its own hall coordinates `hall` —
not `map`, because a mapper anchors `map` wherever its first scan found the robot, which is not the corner
of the hall that GPS and truth positions are measured from.

The frontier node reads the frame out of the map message instead of assuming any of this, so it does not
care which tree it is given.
"""
import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
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


def written(template: str, robot: str) -> str:
    """The parameter file of `template` with the robot's name in it, as a path nav2 can be given.

    Both nav2 and slam_toolbox are handed a *file* and name frames and topics absolutely inside it, so
    `robot:=` cannot reach them as a launch argument — the name has to be in the file. Every `<robot>/` in
    `config/*.yaml` is therefore replaced here, which keeps one setting for the whole stack instead of a
    file to edit per robot. Rewritten on every start, so the file in this checkout stays the truth.
    """
    text = open(template).read().replace("<robot>", robot)
    out = os.path.join(tempfile.gettempdir(), f"ohm_frontier_{robot}_{os.path.basename(template)}")
    open(out, "w").write(text)
    return out


def _sim_dir() -> str:
    """Where `launch/lab.launch.py` is: the installed package if there is one, the checkout otherwise."""
    try:
        return get_package_share_directory("mecanum_lab")
    except PackageNotFoundError:
        return FALLBACK_SIM


def generate_launch_description():
    frontiers = share("ohm_frontier")
    config = lambda name: os.path.join(frontiers, "config", name)          # noqa: E731
    arguments = [
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("world", default_value="rooms", description="hall of the simulator to map"),
        DeclareLaunchArgument("sim_dir", default_value=_sim_dir(),
                              description="the mecanum-lab checkout, or its installed share directory"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="let the simulator open its own window as well"),
        DeclareLaunchArgument("slam_params", default_value=config("slam_toolbox.yaml"),
                              description="the mapper; <robot> in it becomes the robot's name"),
        DeclareLaunchArgument("nav2_params", default_value=config("nav2_rooms.yaml"),
                              description="nav2; <robot> in it becomes the robot's name"),
    ]

    robot, sim, sim_time = (LaunchConfiguration(n) for n in ("robot", "sim_dir", "use_sim_time"))

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim, "launch", "lab.launch.py"])),
        launch_arguments={"world": LaunchConfiguration("world"), "robot": robot, "headless": "false",
                          "use_sim_time": sim_time, "rviz": LaunchConfiguration("rviz"),
                          # A mapper needs to hear that a beam did not come back, which the simulator reports
                          # as its range unless told otherwise: see `mecanum_lab/ros_bridge.py`.
                          "tf_tree": "slam", "lidar_no_echo": "inf"}.items(),
    )

    frontiers_node = Node(
        package="ohm_frontier", executable="frontier_node", name="frontier_node", output="screen",
        parameters=[{"robot": robot, "use_sim_time": sim_time}],
    )

    # The two includes whose parameters live in a file, and a file has to carry the robot's name in it —
    # which is only known once the launch arguments are resolved. `OpaqueFunction` is launch's way of
    # saying "build these later", so the name is asked for here instead of being guessed at the top.
    named = OpaqueFunction(function=started_by_name, args=[LaunchConfiguration("slam_params"),
                                                           LaunchConfiguration("nav2_params"), sim_time])

    return LaunchDescription(arguments + [simulator, named, frontiers_node])


def started_by_name(context, *args, **kwargs):
    """The slam_toolbox and nav2 includes, with the robot's name written into the files they read."""
    read = lambda given: perform_substitutions(                      # noqa: E731
        context, normalize_to_list_of_substitutions(given))
    robot, sim_time = read(LaunchConfiguration("robot")), read(args[2])
    slam_template, nav2_template = written(read(args[0]), robot), written(read(args[1]), robot)

    # slam_toolbox's own launch file rather than its executable: it is a lifecycle node, and starting the
    # executable alone leaves it „unconfigured" and silent, which looks exactly like a wrong scan topic.
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share("slam_toolbox", "the mapper"), "launch",
                                                       "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_template, "use_sim_time": sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share("nav2_bringup", "the navigation stack"),
                                                       "launch", "navigation_launch.py")),
            launch_arguments={"params_file": nav2_template, "use_sim_time": sim_time}.items()),
    ]
