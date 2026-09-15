"""Drive to a place in a hall, and watch the controller decide: the fourth control example.

    ros2 launch ohm_frontier move_to_point.launch.py
    ros2 launch ohm_frontier move_to_point.launch.py aim_tolerance:=0.25     # and watch it arrive wide
    ros2 launch ohm_frontier move_to_point.launch.py view:=false             # the hall window only

    ros2 topic pub --once /muster/move_command std_msgs/msg/String  "data: 'go 12.0 10.0'"

`move_to_point.py` in a hall: orient until the place is inside `aim_tolerance` of the nose, then drive
straight at it, re-orienting whenever it leaves that cone. Start it with `go` on the command topic, or point
the frontier node at it (`goal_topic:=/frontier_goal`, which is the default) and let an exploration decision
drive — the same goal nav2 would be handed, driven by a controller that fits on a slide.

`open` is the default hall, for the reason that it contains nothing: 60 × 40 cells of the simulator's 0.05 m
cells — 30 × 20 m — with four walls, so a straight line to a place is a straight line, and the only thing on
screen is the controller. `rooms` is the second run and the honest one: the same node drives into a doorway
and the odometry starts disagreeing with the hall, which is where the re-aiming shows up unasked.

The two arguments worth an hour are `aim_tolerance` and `gain_turn`. The first is the accuracy, and it is
computable before the robot moves: 0.08 rad at 10 m is 80 cm of sideways error this controller will accept and
still call reached. The second is the shape of the turn, and `turn_limit` is what stops it asking for rates the
wheels cannot deliver. Change one, run the same `go`, compare where it stopped — that is the whole lecture.

RViz is **on** here, unlike the other three demos, and it is the point of the file: the view is this node's own
overlay (`view:=true`, the default) — the line to the place, the line the nose points along, the command as an
arrow and the phase with its two numbers as text (`view_markers.py`). The other demos are legible from the hall
window because their subject is the path; this one's subject is the decision two ticks before the path, and a
straight line and a turn-in-place look the same from outside until you draw what the controller sees.
`rviz_config:=` takes another config. The default here is **`drive.rviz`** rather than `explore.rviz`, and the
difference between the two is the camera and nothing else: text in a marker is metres tall, so the phase and the
heading error written over the robot are readable at 8 m and gone at the 20 m the frontier run needs, and the two
cannot be the same shot. `explore.rviz` is that same thirteen displays pulled back to see the hall.

`markers:=false` leaves the overlay unpublished altogether.

Like the rest of the family, no tf tree argument and no lidar no-echo setting: no mapper is in this graph, so
there is nobody to collide with over `map → <robot>/odom`, and no lidar is read at all — this node steers from
the odometry alone, which is precisely why it can end up wide.
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
from launch_ros.parameter_descriptions import ParameterValue

#: where the simulator lives when it is not installed as a ROS package of its own
FALLBACK_SIM = os.path.expanduser("~/git/mecanum-lab")

#: the strings a launch argument has to be to count as yes, and the line to print when `rviz2` is missing
TRUE = ("true", "True", "1", "yes", "on")
RVIZ_INSTALL_HINT = "sudo apt install ros-$ROS_DISTRO-rviz2"


def sim_dir() -> str:
    """The simulator's share directory, or its checkout when it is not installed.

    The same arrangement as in `reactive_wall_follow.launch.py`: `ros2 launch <path>/launch/lab.launch.py`
    takes a path as readily as a package name, so a simulator that was never built into this workspace still
    starts.
    """
    try:
        return get_package_share_directory("mecanum_lab")
    except PackageNotFoundError:
        return FALLBACK_SIM


def missing(template: str) -> str:
    """Why this file cannot be the view, or "" when it can.

    An exception raised inside an `OpaqueFunction` leaves **nothing** in a launch log: no traceback, no
    `process has died`, just a screen with no RViz on it and no line to read. That is how the first version of
    this launch file failed for a whole session — the config path was resolved against a workspace built
    without its ROS underlay sourced, `open()` raised `FileNotFoundError`, and the launch said nothing at all.
    So the absence is looked for and named here, because a silent absence is worth more than a good error
    message nobody sees.
    """
    if not os.path.isfile(template):
        return (f"[move_to_point] no view config at {template} — this is the path "
                f"`rviz_config` resolved to; `rviz_config:=<file>` picks another, and a workspace whose "
                f"`install/` was built without a sourced ROS underlay is how it goes missing")
    return ""


def written(template: str, robot: str) -> str:
    """This file with the robot's name written into it, as a path rviz2 can be given.

    The one view for the whole package (`config/explore.rviz`, installed under `share/ohm_frontier/rviz/`),
    and the same substitution `explore.launch.py` performs on it and on the two parameter files: an `.rviz`
    file names topics absolutely inside display properties, where a launch substitution cannot reach, so
    `<robot>/odom` has to be replaced on the way to disk. Rewritten on every start, which keeps the copy in
    this checkout the one that is true.
    """
    text = open(template).read().replace("<robot>", robot)
    out = os.path.join(tempfile.gettempdir(), f"ohm_frontier_{robot}_{os.path.basename(template)}")
    open(out, "w").write(text)
    return out


def generate_launch_description():
    robot, sim_time = LaunchConfiguration("robot"), LaunchConfiguration("use_sim_time")
    arguments = [
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("world", default_value="open",
                              description="hall to drive across; open is empty, which is the point"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="without the simulator window (sets SDL_VIDEODRIVER=dummy)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="the view of what the controller is deciding; rviz_config:= for another"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("ohm_frontier"), "rviz", "drive.rviz"),
            description="the frontier view on a camera close enough to read the phase over the robot; "
                        "`explore.rviz` is the same thirteen displays at hall distance"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("goal_topic", default_value="/frontier_goal",
                              description="where a place to drive to comes from; empty for typed commands only"),
        DeclareLaunchArgument("markers", default_value="true",
                              description="publish the overlay markers this node is drawn from; not named "
                                          "`view` because the simulator declares that name for its window"),
        DeclareLaunchArgument("speed", default_value="0.3", description="m/s while driving"),
        DeclareLaunchArgument("aim_tolerance", default_value="0.08",
                              description="rad; outside this cone the robot turns and does not drive"),
        DeclareLaunchArgument("gain_turn", default_value="1.8",
                              description="rad/s of turn per radian of heading error"),
        DeclareLaunchArgument("turn_limit", default_value="1.2", description="rad/s ceiling on the turn"),
        DeclareLaunchArgument("arrive_distance", default_value="0.12",
                              description="m; nearer than this to the place and it counts as reached"),
    ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim_dir(), "launch", "lab.launch.py"])),
        launch_arguments={"robot": robot, "world": LaunchConfiguration("world"),
                          "headless": LaunchConfiguration("headless"), "use_sim_time": sim_time,
                          "rviz": "false"}.items(),          # our view, not the simulator's: one fixed frame
    )

    node = Node(
        package="ohm_frontier", executable="move_to_point", name="move_to_point", output="screen",
        parameters=[{
            "robot": robot,
            "use_sim_time": ParameterValue(sim_time, value_type=bool),
            "goal_topic": LaunchConfiguration("goal_topic"),
            "view": ParameterValue(LaunchConfiguration("markers"), value_type=bool),
            "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
            "aim_tolerance": ParameterValue(LaunchConfiguration("aim_tolerance"), value_type=float),
            "gain_turn": ParameterValue(LaunchConfiguration("gain_turn"), value_type=float),
            "turn_limit": ParameterValue(LaunchConfiguration("turn_limit"), value_type=float),
            "arrive_distance": ParameterValue(LaunchConfiguration("arrive_distance"), value_type=float),
        }],
    )

    # The view is the FIRST entity, and that position is load-bearing rather than tidiness. An included launch
    # file writes its own arguments into *this* file's configuration space — measured, not inferred: a file
    # that declares `rviz` with default `true`, includes `lab.launch.py` with `rviz:=false`, and reads `rviz`
    # again afterwards reads back `false`, and reads back `robots` as empty although nobody mentioned it. The
    # simulator declares `world, robot, robots, controller, task, grade, seconds, headless, use_sim_time,
    # config, view, layers, rviz, truth, log, json, seed, tf_tree, lidar_no_echo` (`BASICS` in
    # `mecanum-lab/launch/lab.launch.py`), so it overwrites the one argument this decision is made of. Read
    # before the include and the answer is ours; read after it and the answer is `false`, which is how the
    # default-on view stopped being on. `test_launch_files.py` keeps this in front of the include.
    view = OpaqueFunction(function=started_by_view, args=[LaunchConfiguration("rviz"),
                                                          LaunchConfiguration("rviz_config"), robot, sim_time])
    return LaunchDescription(arguments + [view, simulator, node])


def started_by_view(context, *args, **kwargs):
    """rviz2 on this package's view, or the one line that says why there is none.

    The same three judgements `explore.launch.py` makes, for the same reasons: `rviz:=false` was asked for and
    needs no comment; a machine without `rviz2` gets the apt line rather than a screen ending in `process has
    died`; and the viewer is told `use_sim_time`, because the simulator stamps its transforms in seconds since
    it started and a viewer on the wall clock thinks the whole run happened 1.7 billion seconds ago and draws
    nothing.
    """
    read = lambda given: perform_substitutions(                      # noqa: E731
        context, normalize_to_list_of_substitutions(given))
    if read(args[0]) not in TRUE:
        return []
    complaint = missing(read(args[1]))
    if complaint:
        return [LogInfo(msg=complaint)]
    if shutil.which("rviz2") is None:
        return [LogInfo(msg=f"[move_to_point] no rviz2 on this machine — the simulator's window is the only "
                            f"view, `{RVIZ_INSTALL_HINT}` adds this one, `rviz:=false` stops asking for it")]
    command = ["rviz2", "--display-config", written(read(args[1]), read(args[2]))]
    if read(args[3]) in TRUE:
        command += ["--ros-args", "-p", "use_sim_time:=true"]
    return [ExecuteProcess(cmd=command, name="move_to_point_view", output="screen")]
