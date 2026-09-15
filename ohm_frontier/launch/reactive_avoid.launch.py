"""Drive one way and get round whatever the lidar sees: the hall, 360 beams, no map and no planner.

    ros2 launch ohm_frontier reactive_avoid.launch.py
    ros2 launch ohm_frontier reactive_avoid.launch.py world:=rooms drive_heading:=90
    ros2 launch ohm_frontier reactive_avoid.launch.py strafe:=true      # the mecanum comparison

The obstacle-avoidance demo of the reactive family: `obstacle_avoidance.py` against a real hall. One
argument is worth playing with live — `drive_heading`, the direction the rule wants, in degrees from the
robot's own nose. Set it to 0 and the robot walks out of the hall; set it to 90 and the same robot in the
same hall does something completely different with the same walls, which is the point: this node has no
goal, so the only thing that makes a direction "forward" is the number a person typed.

What it does with that direction is worth knowing before the run, because the overlay on the screen is that
and not a wish. Every heading 5° around the robot gets two numbers of metres — how far it may travel that
way before an echo enters the `clearance` ring, and how much floor the lidar sees down that ray — and a
heading is only drivable when it offers `free_travel` of the first **and** `open_floor` of the second. The
second test is why a dead end is told from a corridor: the near map alone says a heading into a dead end has
half a metre of travel, which is true and useless. Of the headings left, the one with the widest opening is
driven, the side an obstacle is being gone round is committed to rather than re-chosen every cycle, the speed
comes from the travel the driven direction has, and `wz` goes to the chosen heading even when the speed is
zero — a stopped robot that is turning towards the widest gap has decided something, and a stopped robot that
is not has a minute left on the projector and nothing to show.

`rooms` is the default, and it is the hall where the rule has something to say: seven cells (3.5 m) of open
floor from the spawn, a doorway out of the box it is standing in, and a corridor beyond that narrows. The run
worth watching is the last one — the robot lines up on the doorway, takes about a third of it before the gap
fails the clearance test, and then says which way it decided rather than pushing.

**`production` is the refusal, and it is the honest demonstration of the two.** Measured on the aisle floor
with the simulator's own geometry, the free aisle between the shelf ends and the hall wall is **0.63 m**; the
robot is 0.44 m long and 0.46 m wide, so its own diagonal is **0.66 m**, and the `clearance` ring the rule
keeps off things has to sit inside the difference — it cannot both fit the aisle and stay off its walls. The
robot therefore stops at the mouth of the aisle, says so, and after `stuck_timeout` says so with a number:
measured over 120 s, **118.99 s of it in `STOPPED`, 1.20 m from where it started, the nearest echo 0.70 m**.
That is the aisle speaking and not the rule failing, and the two are only told apart by a node that refuses
out loud instead of scraping through — which is the version of this demo that used to end in a robot wedged
between two shelves printing `clear way`.

`open` (30 × 20 m, nothing in it) is the flat test: the rule should cross it at full speed and never brake.
Put shelves in `worlds/*.txt` and the two tests are the two `ways` numbers on the overlay, drawn as the
headings it refused to drive.

Nothing here asks for the tf tree or the lidar's no-echo dialect that `explore.launch.py` is careful about,
and that is deliberate: with no mapper and no planner in the graph there is no second publisher of
`map → <robot>/odom` to collide with, and the rule reads the range array directly, so it does not matter
whether a missing echo arrives as 8.0 m or as infinity. It handles both, and `nan` as well.

The simulator's own window is the display for this demo, so RViz stays off by default; `rviz:=true` adds it
for anyone who wants the scan and the clearance ring while the robot curves.
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
        DeclareLaunchArgument("world", default_value="rooms",
                              description="hall to drive the rule through; `production` is the hall this "
                                          "robot does not fit, and says so"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="without the simulator window — of little use for this demo"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="the simulator's RViz too; the hall window is the display here"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        DeclareLaunchArgument("drive_heading", default_value="0.0",
                              description="deg from the robot's nose: the direction the rule wants to go"),
        DeclareLaunchArgument("speed", default_value="0.35", description="m/s on a heading with room"),
        DeclareLaunchArgument("clearance", default_value="0.40",
                              description="m of ring around the centre that no echo may enter. This robot's "
                                          "own body reaches 0.25 m from the centre and its collision circle "
                                          "is 0.21 m, so anything under about 0.25 m asks the rule to drive "
                                          "with its flank inside the wall"),
        DeclareLaunchArgument("free_travel", default_value="0.25",
                              description="m of travel a heading must offer to be worth driving at all; "
                                          "below it the heading is drawn red and refused"),
        DeclareLaunchArgument("open_floor", default_value="3.0",
                              description="m of floor the lidar must see down a ray for that heading to be "
                                          "worth driving — the test that tells a corridor from a dead end"),
        DeclareLaunchArgument("stuck_timeout", default_value="20.0",
                              description="s of turning with nothing offering `free_travel` before the node "
                                          "says `nowhere to go` and stops trying; 'back out and try the other "
                                          "side of the hall' is a program with a map in it"),
        DeclareLaunchArgument("strafe", default_value="false",
                              description="drive the chosen heading sideways instead of turning to it. Off, "
                                          "because a robot that slides while facing elsewhere reads to an "
                                          "audience as a robot that misbehaves; on, it is the mecanum "
                                          "comparison this hardware is there for"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([sim_dir(), "launch", "lab.launch.py"])),
            launch_arguments={"robot": robot, "world": LaunchConfiguration("world"),
                              "headless": LaunchConfiguration("headless"),
                              "use_sim_time": LaunchConfiguration("use_sim_time"),
                              "rviz": LaunchConfiguration("rviz")}.items(),
        ),
        Node(
            package="ohm_frontier", executable="obstacle_avoidance", name="obstacle_avoidance",
            output="screen",
            parameters=[{
                "robot": robot,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # a launch argument arrives as text and these are numbers and a switch in the node
                "drive_heading": ParameterValue(LaunchConfiguration("drive_heading"), value_type=float),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "clearance": ParameterValue(LaunchConfiguration("clearance"), value_type=float),
                "free_travel": ParameterValue(LaunchConfiguration("free_travel"), value_type=float),
                "open_floor": ParameterValue(LaunchConfiguration("open_floor"), value_type=float),
                "stuck_timeout": ParameterValue(LaunchConfiguration("stuck_timeout"), value_type=float),
                "strafe": ParameterValue(LaunchConfiguration("strafe"), value_type=bool),
            }],
        ),
    ])
