"""Drive one way and get around whatever the lidar sees: the hall plus the vector field, no map, no planner.

    ros2 launch ohm_frontier reactive_avoid.launch.py
    ros2 launch ohm_frontier reactive_avoid.launch.py world:=rooms drive_heading:=90

The obstacle-avoidance demo of the reactive family, `obstacle_avoidance.py` against a real hall. One
argument is worth playing with live: `drive_heading`, the direction the field is pulled towards, in
degrees from the robot's own nose. Set it to 0 and the robot walks out of the hall; set it to 90 and the
same robot in the same hall does something completely different with the same walls, which is the point —
this node has no goal, so the only thing that makes a direction "forward" is the number a person typed.

`production` is the default because it is the hall where the field has something to say. Measured in
`worlds/production.txt` (a 20 × 12 m hall, cells 0.5 m): the spawn faces east down the bottom aisle with
the south wall 2.5 m to its right and the first shelf end 29 cells — 14.5 m — ahead, which is beyond the
lidar's 8 m. So the run starts with the field having no opinion at all and the robot driving straight, and
somewhere at 8 m the shelf end walks into the scan and the robot begins to curve. Watch that first
correction: it is not a decision, it is the sum of the beams near the corner of a shelf.

What this node cannot see is the second half of the lesson and it takes about a minute to demonstrate:
drive the same robot into the inside corner between a shelf end and the hall wall. The two repulsions add
up along the bisector, which points straight into the corner, so the robot stops there and pushes — the
concave-corner trap of a vector field, and the reason `frontiers.py` and nav2 exist rather than a better
tuning of this one file.

`rooms` is the shorter version of the same demo, seven cells (3.5 m) of open floor from the spawn to a
wall: the field turns the corner of it in about four seconds, which fits on one slide.

Nothing here asks for the tf tree or the lidar's no-echo dialect that `explore.launch.py` is careful
about, and that is deliberate: with no mapper and no planner in the graph there is no second publisher of
`map → <robot>/odom` to collide with, and `field` reads the range array directly, so it does not matter
whether a missing echo arrives as 8.0 m or as infinity. It handles both.

The simulator's own window is the display for this demo, so RViz stays off by default; `rviz:=true` adds
it for anyone who wants to see the scan while the robot curves.
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
        DeclareLaunchArgument("world", default_value="production",
                              description="hall to drive the field through; production has shelf ends"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="without the simulator window — of little use for this demo"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="the simulator's RViz too; the hall window is the display here"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("drive_heading", default_value="0.0",
                              description="deg from the robot's nose: the direction the field wants to go"),
        DeclareLaunchArgument("speed", default_value="0.35", description="m/s"),
        DeclareLaunchArgument("repulsion_range", default_value="2.0",
                              description="m; a wall further than this gets no vote at all"),
        DeclareLaunchArgument("stop_gap", default_value="0.55",
                              description="m ahead at which the robot stands still instead of braking"),

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
                # a launch argument arrives as text and these three are numbers in the node
                "drive_heading": ParameterValue(LaunchConfiguration("drive_heading"), value_type=float),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "repulsion_range": ParameterValue(LaunchConfiguration("repulsion_range"),
                                                  value_type=float),
                "stop_gap": ParameterValue(LaunchConfiguration("stop_gap"), value_type=float),
            }],
        ),
    ])
