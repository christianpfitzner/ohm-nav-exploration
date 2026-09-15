"""Turn, drive a distance, drive to a place: the three primitives, typed at a topic.

    ros2 launch ohm_frontier reactive_turn_and_move.launch.py
    ros2 launch ohm_frontier reactive_turn_and_move.launch.py world:=open robot:=carlo

The third reactive demo of the family, and the only one of the three that a lecture drives by hand. The node
takes one line of text per command (`turn_and_move.py`), so the effect of a number is visible without
anybody editing a file:

    ros2 topic pub --once /muster/reactive_command std_msgs/msg/String  "data: 'turn 90'"
    ros2 topic pub --once /muster/reactive_command std_msgs/msg/String  "data: 'drive 1.0'"
    ros2 topic pub --once /muster/reactive_command std_msgs/msg/String  "data: 'stop'"

A place to drive to is not typed here — it arrives on `/frontier_goal`, which is what the frontier node
publishes, and `drive_to` is the controller that closes on it. `parse_command` knows three commands and
answers anything else with the reason, because a lecturer mistyping at a whiteboard should see why nothing
moved rather than watch a robot ignore them.

`open` is the default hall, for the reason that it contains nothing: 60 × 40 cells of the simulator's 0.5 m
cells — 30 × 20 m — and four walls, so a `drive 5.0` measures what the odometry reports rather than what the
first obstacle allowed. In `rooms` the same command ends at a doorway and the lesson becomes a different one.

Three things to do with it while it is on the screen, the first two of which take under a minute:

* `turn 90` with the default gains, then `turn_and_move` started again with `turn_tolerance:=0.2`. The robot
  stops 11 degrees early and nobody touched the controller. The tolerance is what declares a turn finished,
  not the robot arriving — a P controller gets asymptotically closer and never does.
* `drive 3.0` while watching the hall window. The simulator's `robot.slip` is 1.0 by default
  (`mecanum_lab/physics.py`), so driving a mecanum base into a wall keeps the wheels integrating metres the
  robot never drove. `drive` counts metres of wheels and finishes "successfully" with the robot where it
  started. That is the reason the frontier node in this package reads a mapper's map instead of trusting
  this odometry, and this is the cheapest place in the whole repository to show it.

`strafe:=true` is the third thing worth doing, and it is off by default: the node then commands `vx` and `wz`
only. The number behind that default is one 6.0 m goal in this very hall with the lateral term live — **49 % of
the samples carried a sideways command of up to 0.15 m/s, and 1.36 m of the 8.51 m travelled was sideways travel
for 5.90 m of net displacement** (`tools/try_demo.sh reactive_turn_and_move.launch.py <robot> <domain> 28`, the
`sideways command` line of its report). Both settings arrive; only one of them looks to a hall like a robot
going sideways for no visible reason, and `measure_drive`'s `path/net` was 1.4 against 1.2 for the demo with no
lateral term at all. Turn the switch on to show what a mecanum base is for, and off again to show what that
looks like from the back of the room.

Nothing here asks for the tf tree or the lidar's no-echo dialect that `explore.launch.py` is careful about.
There is no mapper and no navigation stack in this graph, so there is no second publisher of
`map → <robot>/odom` to collide with, and the node has no tf on purpose: a goal that arrives on
`/frontier_goal` is taken as odometry coordinates and said out loud once, which is the difference between the
map's frame and the odometry's — centimetres in a good run, metres in the exercises that ruin the odometry on
purpose.

The hall window is the display for this demo, so RViz is off by default; `rviz:=true` adds it for anyone who
wants to see the commanded pose next to the map.
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

    The same arrangement as in `reactive_wall_follow.launch.py`: `ros2 launch <path>/launch/lab.launch.py`
    takes a path as readily as a package name, so a simulator that was never built into this workspace
    still starts.
    """
    try:
        return get_package_share_directory("mecanum_lab")
    except PackageNotFoundError:
        return FALLBACK_SIM


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("world", default_value="open",
                              description="hall to drive in; open is empty, which is the point"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="without the simulator window — of little use for this demo"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="the simulator's RViz too; the hall window is the display here"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("speed", default_value="0.3", description="m/s for `drive` and for `go`"),
        DeclareLaunchArgument("turn_limit", default_value="1.2", description="rad/s ceiling on `turn`"),
        DeclareLaunchArgument("arrive_distance", default_value="0.12",
                              description="m; a `go` goal inside this counts as reached"),
        DeclareLaunchArgument("turn_tolerance", default_value="0.05",
                              description="rad; the P controller never arrives, this declares it finished"),
        DeclareLaunchArgument("strafe", default_value="false",
                              description="command vy as well as vx and wz. Off: the default run is car-like, "
                                          "because 49 % of the samples of a 6 m goal otherwise carried a sideways "
                                          "command and 1.36 m of the path was sideways travel. On: the mecanum "
                                          "sum holds a line and walks diagonally at the place"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([sim_dir(), "launch", "lab.launch.py"])),
            launch_arguments={"robot": robot, "world": LaunchConfiguration("world"),
                              "headless": LaunchConfiguration("headless"),
                              "use_sim_time": LaunchConfiguration("use_sim_time"),
                              "rviz": LaunchConfiguration("rviz")}.items(),
        ),
        Node(
            package="ohm_frontier", executable="turn_and_move", name="turn_and_move", output="screen",
            parameters=[{
                "robot": robot,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # a launch argument arrives as text and these four are numbers in the node
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "turn_limit": ParameterValue(LaunchConfiguration("turn_limit"), value_type=float),
                "arrive_distance": ParameterValue(LaunchConfiguration("arrive_distance"), value_type=float),
                "turn_tolerance": ParameterValue(LaunchConfiguration("turn_tolerance"), value_type=float),
                # a launch argument arrives as text and this one is a bool in the node
                "strafe": ParameterValue(LaunchConfiguration("strafe"), value_type=bool),
            }],
        ),
    ])
