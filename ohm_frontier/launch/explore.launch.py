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

Two defaults are worth naming before someone hunts for them in a `--show-args` listing. The hall is
`rooms` and the simulator's window is open, because the run is shown to a lecture: a default that starts
headless shows the students nothing and a default hall nobody chose is a hall the documentation does not
describe. The second is that this file starts **its own** RViz, on `config/explore.rviz`, and asks the
simulator for none — the two views would otherwise fight over the `map` frame, and the one that loses
shows an empty map while the other one is right. `rviz:=false` starts neither, which is what a recorded
run and a CI machine want.
"""
import os
import shutil
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

TRUE = ("true", "1", "yes", "on")        # launch arguments arrive as text; "True" and "on" mean the same
RVIZ_INSTALL_HINT = "sudo apt install ros-$ROS_DISTRO-rviz2"

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
    """A file of `template` with the robot's name in it, as a path something else can be given.

    Both nav2 and slam_toolbox are handed a *file* and name frames and topics absolutely inside it, so
    `robot:=` cannot reach them as a launch argument — the name has to be in the file. RViz has the same
    limitation from the other side: it cannot substitute a name into a display's topic, and every topic of
    a robot here carries that name. Every `<robot>` in `config/*` is therefore replaced here, which keeps
    one setting for the whole stack instead of a file to edit per robot. Rewritten on every start, so the
    file in this checkout stays the truth and an edited copy in the temporary directory is a build product.
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
        # The six halls of the simulator, `~/git/mecanum-lab/worlds/<name>.txt`, 0.5 m per cell. `rooms` is
        # the default because it is the one with doorways: every room behind a doorway is one frontier, so
        # the decisions are legible. `maze` is half a metre of corridor, which a mapper survives badly.
        DeclareLaunchArgument("world", default_value="rooms",
                              description="hall to map: rooms | maze | open | arena | production | track"),
        DeclareLaunchArgument("sim_dir", default_value=_sim_dir(),
                              description="the mecanum-lab checkout, or its installed share directory"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="the simulator without its Pygame window (a lecture sees a "
                                          "window; a recorded run and CI see nothing)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="RViz 2 on this package's frontier view; the simulator is asked "
                                          "for none either way, so there is never a second window on map"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(frontiers, "rviz", "explore.rviz"),
                              description="the frontier view; <robot> in it becomes the robot's name, as "
                                          "in the parameter files"),
        DeclareLaunchArgument("nav2", default_value="true",
                              description="the navigation stack too; false leaves the goals on "
                                          "/frontier_goal with nothing driving (explore_no_nav2.launch.py)"),
        DeclareLaunchArgument("slam_params", default_value=config("slam_toolbox.yaml"),
                              description="the mapper; <robot> in it becomes the robot's name"),
        DeclareLaunchArgument("nav2_params", default_value=config("nav2_rooms.yaml"),
                              description="nav2; <robot> in it becomes the robot's name"),
    ]

    robot, sim, sim_time = (LaunchConfiguration(n) for n in ("robot", "sim_dir", "use_sim_time"))

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim, "launch", "lab.launch.py"])),
        launch_arguments={"world": LaunchConfiguration("world"), "robot": robot,
                          "headless": LaunchConfiguration("headless"),
                          "use_sim_time": sim_time,
                          # This file owns the ROS-side view: `rviz` above decides whether *our* RViz
                          # starts, and the lab's own RViz never does — two viewers with fixed frame `map`
                          # is one too many, and the second one is the one a student then blames.
                          "rviz": "false",
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

    # The view is built late for the same reason the parameter files are: `explore.rviz` names `/<robot>/scan`
    # and `/<robot>/odom`, and the name is only known once the arguments are resolved.
    view = OpaqueFunction(function=started_by_view, args=[LaunchConfiguration("rviz"),
                                                          LaunchConfiguration("rviz_config"), sim_time])

    return LaunchDescription(arguments + [simulator, named, view, frontiers_node])


def started_by_name(context, *args, **kwargs):
    """The slam_toolbox and nav2 includes, with the robot's name written into the files they read.

    `nav2:=false` leaves the stack out and the mapper in: the frontier node needs a map to decide from and
    nothing else, and a run without the stack is the one where the decisions themselves are the subject.
    """
    read = lambda given: perform_substitutions(                      # noqa: E731
        context, normalize_to_list_of_substitutions(given))
    robot, sim_time = read(LaunchConfiguration("robot")), read(args[2])
    slam_template, nav2_template = written(read(args[0]), robot), written(read(args[1]), robot)

    # slam_toolbox's own launch file rather than its executable: it is a lifecycle node, and starting the
    # executable alone leaves it „unconfigured" and silent, which looks exactly like a wrong scan topic.
    started = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share("slam_toolbox", "the mapper"), "launch",
                                                       "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_template, "use_sim_time": sim_time}.items()),
    ]
    if read(LaunchConfiguration("nav2")).lower() in TRUE:
        started.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share("nav2_bringup", "the navigation stack"),
                                                       "launch", "navigation_launch.py")),
            launch_arguments={"params_file": nav2_template, "use_sim_time": sim_time}.items()))
    else:
        started.append(LogInfo(msg="[explore] nav2:=false — nothing drives to the goals; they go out on "
                                   "/frontier_goal and into RViz only"))
    return started


def started_by_view(context, *args, **kwargs):
    """RViz 2 on this package's view, or the one line that says why there is none.

    `rviz2` is not part of the ROS base install, and a launch file that starts it anyway leaves the screen
    ending in `process has died` — which reads as a broken explorer rather than as a missing apt package,
    and is the wrong lesson twice over. So the executable is looked for here, and its absence is answered
    with what to type. `rviz:=false` is a different answer: that one was asked for, so it needs no comment.

    `use_sim_time` is passed on rather than remembered: the simulator stamps its transforms in seconds since
    it started, and a viewer on the wall clock believes the whole map happened 1.7 billion seconds ago and
    draws nothing at all.
    """
    read = lambda given: perform_substitutions(                      # noqa: E731
        context, normalize_to_list_of_substitutions(given))
    if read(args[0]).lower() not in TRUE:
        return []
    if shutil.which("rviz2") is None:
        return [LogInfo(msg=f"[explore] no rviz2 on this machine, so the simulator's window is the only "
                            f"view — `{RVIZ_INSTALL_HINT}` adds the frontier view, `rviz:=false` stops "
                            f"asking for it")]
    command = ["rviz2", "--display-config", written(read(args[1]), read(LaunchConfiguration("robot")))]
    if read(args[2]).lower() in TRUE:
        command += ["--ros-args", "-p", "use_sim_time:=true"]
    return [ExecuteProcess(cmd=command, name="rviz_view", output="screen")]
