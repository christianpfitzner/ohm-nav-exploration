"""The velocity command has to reach the simulator as a message type it is listening for.

nav2 hands the robot's velocity through four hands, and every hand is a topic of its own:

    controller_server ─┐                      /cmd_vel_nav
    behavior_server  ──┤  (cmd_vel → cmd_vel_nav by nav2's own launch file)
    velocity_smoother ─┼→ /cmd_vel_nav → /cmd_vel_smoothed        (its input, by the same remap)
    collision_monitor ─→ /cmd_vel_smoothed → /<robot>/cmd_vel → the simulator

Two of those hops are chosen by nav2's `navigation_launch.py` and two are written in
`config/nav2_rooms.yaml`, and the last one is the only place where nav2 stops talking to itself and starts
talking to something that was not written by nav2. That is where it went silent.

Kilted's nav2 defaults `enable_stamped_cmd_vel` — which picks the *message type* of those topics, not their
rate — to **true**, meaning `geometry_msgs/msg/TwistStamped`. The simulator drives from a
`geometry_msgs/msg/Twist` on `/<robot>/cmd_vel` (`mecanum-lab` CONTRACT §6.9, and one Twist is what a
student's own node publishes when they hold `w`). DDS refuses to match a publisher and a subscription of
different types and **says nothing about it**: measured on the `rooms` spawn with a goal 1.5 m away, all
three topics carried 20 Hz, all three were `TwistStamped`, all three carried `vx=+0.350` for the whole
goal, the collision monitor passed every command through, and the odometry did not move 0.01 m. `ros2 topic
echo` and `--hz` answer "no messages" for a topic whose published type differs from the one they guessed, so
the whole thing looks like a controller that never commands anything — which is what this repo's README said
about it until a probe that subscribed both types side by side was written.

So the four nodes are one setting, not four, and this file is where it is kept.

Nothing here needs ROS, a simulator or a running stack: the thing under test is a parameter file, and
`python3 -m pytest test` reads it as the YAML nav2 will read it.

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
"""
import pathlib

import yaml

HERE = pathlib.Path(__file__).resolve().parents[1]
PARAMS = HERE / "config" / "nav2_rooms.yaml"

#: Every node that writes or reads a velocity command, in the order a command travels. A node missing from
#: this list keeps nav2's own default for the type switch, which is the stamped one — so the list and the
#: parameter file have to be kept in step by the first test below rather than by memory.
CHAIN = ("controller_server",      # → /cmd_vel_nav, the drive a follow_path goal produces
         "behavior_server",        # → /cmd_vel_nav, the recovery drives (spin, backup, drive_on_heading)
         "velocity_smoother",      # /cmd_vel_nav in, /cmd_vel_smoothed out
         "collision_monitor")      # /cmd_vel_smoothed in, the simulator's topic out

#: What the simulator listens to. `mecanum-lab` gives a student's node a `geometry_msgs/msg/Twist` on
#: `/<robot>/cmd_vel` and nothing else, so the chain has to end in a Twist: stamped would be True.
SIMULATOR_TAKES_A_TWIST = False

#: the topic names of the two hops nav2's own launch file decides, repeated here so a rename on either
#: side of the include is caught instead of discovered by a robot that stands still
SMOOTHER_OUTPUT = "cmd_vel_smoothed"


def simulator_input(robot="muster"):
    """The one topic the simulator drives from — its name is in it, like every other topic of a robot."""
    return f"/{robot}/cmd_vel"


def loaded(robot="muster"):
    """The parameter file as the launch file hands it to nav2: this robot's name written in."""
    return yaml.safe_load(PARAMS.read_text().replace("<robot>", robot))


def flag(section):
    return section["ros__parameters"]["enable_stamped_cmd_vel"]


def test_every_node_that_hands_the_command_on_names_the_message_type_it_uses():
    """One missing and that node keeps nav2's default, which is the other type."""
    given = loaded()
    missing = [n for n in CHAIN if "enable_stamped_cmd_vel" not in given[n]["ros__parameters"]]
    assert not missing, f"{', '.join(missing)}: nav2's default for enable_stamped_cmd_vel decides this hop"


def test_the_four_hands_of_the_chain_agree_on_one_message_type():
    """The chain breaks at the first hop where two neighbours disagree, and breaks silently."""
    decided = {node: flag(loaded()[node]) for node in CHAIN}
    assert len(set(decided.values())) == 1, \
        "the type switch is one setting for the whole chain: " + \
        ", ".join(f"{n}={v}" for n, v in decided.items())


def test_that_one_type_is_the_one_the_simulator_listens_to():
    """"All four agree" is not enough: four nodes agreeing on TwistStamped move nothing, and agree with
    each other while they do it. The far end of the chain is a Twist, so the chain is a Twist."""
    decided = {node: flag(loaded()[node]) for node in CHAIN}
    wrong = {n: v for n, v in decided.items() if v != SIMULATOR_TAKES_A_TWIST}
    assert not wrong, (f"{wrong} publishes/reads TwistStamped, but the simulator drives from a "
                       f"geometry_msgs/msg/Twist on {simulator_input()} (mecanum-lab CONTRACT §6.9) and "
                       f"DDS drops a type mismatch between publisher and subscription without a word")


def test_the_last_hop_is_the_topic_the_simulator_drives_from():
    """nav2's default output is `cmd_vel`; this robot's topic carries its name, and the monitor is the only
    node that writes it."""
    monitor = loaded()["collision_monitor"]["ros__parameters"]
    assert monitor["cmd_vel_in_topic"] == SMOOTHER_OUTPUT, \
        f"the monitor reads {monitor['cmd_vel_in_topic']!r}, the velocity smoother writes {SMOOTHER_OUTPUT!r}"
    assert monitor["cmd_vel_out_topic"] == simulator_input(), \
        f"the simulator listens on {simulator_input()!r}, the monitor writes {monitor['cmd_vel_out_topic']!r}"


def test_writing_another_robots_name_in_leaves_the_chain_alone():
    """`robot:=carlo` rewrites this file as text before nav2 reads it. It must not be able to move a type
    switch or miss the command topic on its way through."""
    given = loaded("carlo")
    assert {node: flag(given[node]) for node in CHAIN} == \
        {node: SIMULATOR_TAKES_A_TWIST for node in CHAIN}
    assert given["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] == simulator_input("carlo")
    assert "<robot>" not in PARAMS.read_text().replace("<robot>", ""), "one placeholder per name, no nesting"
