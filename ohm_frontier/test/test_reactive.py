"""The reactive family's maths, tested without ROS and without a hall.

    python3 -m pytest test          # needs nothing but numpy

`wall_following.py`, `obstacle_avoidance.py` and `turn_and_move.py` each keep their arithmetic in a plain
function — `steer`, `field`, `turn_towards`, `drive_straight`, `drive_to` — for exactly this file: a rule
like these is worth believing before someone watches a robot for ten minutes in front of a lecture hall,
and a simulation is the slowest possible way to find out whether a sign is right. A scan is therefore drawn
here as a list of 360 numbers, the way `sensors.py` builds one, and the answer is a number.

The three tests at the end are about the split itself: the modules must still import when ROS is not there,
because that is what makes the functions above testable at all, and `main` is what has to complain instead.
"""
from math import inf, nan, pi, radians
from pathlib import Path
import re
import sys

import pytest

sys.path.insert(0, ".")
from ohm_frontier import move_to_point, obstacle_avoidance, turn_and_move, view_markers, wall_following   # noqa: E402
from ohm_frontier.obstacle_avoidance import CLEAR, STOPPED, TURNING, field, nearest_ahead, wrap  # noqa: E402
from ohm_frontier.turn_and_move import drive_straight, drive_to, parse_command, turn_towards  # noqa: E402
from ohm_frontier.wall_following import BLOCKED, FOLLOWING, SEARCHING, beam, steer    # noqa: E402

BEAMS, INCREMENT, RANGE_MAX = 360, 2 * pi / 360, 8.0        # what `sensors.py` actually publishes


def a_scan(walls=(), no_echo="inf"):
    """A laser scan drawn by hand: `walls` is ((degrees, metres), ...), everything else no echo.

    Degrees, because that is how the beam index is thought about: beam 0 is the nose and the index grows
    counterclockwise, so the right hand is at negative angles and the right-wall beam of the wall follower
    is index 270. A wall covers five beams rather than one, which is what a flat wall in front of a robot
    with 1° beams really does.

    `no_echo` picks which dialect of "nothing came back" to write, because both exist in this ecosystem —
    the simulator's own `range_max` unless it was started with `lidar_no_echo:=inf`, and infinity when it
    was — and the rules are supposed to answer them the same way.
    """
    blank = {"inf": inf, "range": RANGE_MAX, "nan": nan}[no_echo]
    ranges = [blank] * BEAMS
    for angle, distance in walls:
        middle = round(radians(angle) / INCREMENT) % BEAMS
        for offset in range(-2, 3):
            ranges[(middle + offset) % BEAMS] = distance
    return ranges


PACKAGE = Path(__file__).resolve().parents[1]      # where setup.py and launch/ are, whoever ran pytest from

# ------------------------------------------------------------------------------- wall following


def test_a_wall_closer_than_the_wanted_gap_is_answered_by_turning_away_from_it():
    """The proportional term of `steer`, in the direction a person standing beside the robot expects."""
    decided = steer(a_scan([(-90, 0.25)]), INCREMENT, RANGE_MAX)
    assert decided.state == FOLLOWING and decided.wall == pytest.approx(0.25)
    assert decided.turn > 0, "a wall at 25 cm where 40 cm was wanted must turn left, away from it"
    assert decided.speed > 0, "the rule follows a wall it has, it does not stop for it"


def test_a_wall_farther_than_the_wanted_gap_is_answered_by_turning_towards_it():
    decided = steer(a_scan([(-90, 0.90)]), INCREMENT, RANGE_MAX)
    assert decided.state == FOLLOWING
    assert decided.turn < 0, "90 cm of gap where 40 cm was wanted must turn right, back towards the wall"


def test_nothing_on_the_right_stops_the_robot_and_turns_until_a_wall_comes_back():
    """The state the lecture waits for. Driving forward without a reference is how this demo crashes."""
    decided = steer(a_scan(), INCREMENT, RANGE_MAX)
    assert decided.state == SEARCHING and decided.wall is None
    assert decided.speed == 0.0 and decided.turn < 0, "spin on the spot to the right, do not drive"


def test_the_right_hand_is_the_half_of_the_scan_with_the_negative_angles():
    """A wall on the left must not be mistaken for a wall on the right — the classic off-by-half-circle."""
    assert steer(a_scan([(90, 0.30)]), INCREMENT, RANGE_MAX).state == SEARCHING
    assert steer(a_scan([(-90, 0.30)]), INCREMENT, RANGE_MAX).state == FOLLOWING


def test_a_missing_echo_is_not_a_wall_at_the_far_end_of_the_lidar():
    """All three dialects of "nothing reflected" mean the same thing; "no echo" as 8.0 m is the trap."""
    for dialect in ("inf", "range", "nan"):
        decided = steer(a_scan(no_echo=dialect), INCREMENT, RANGE_MAX)
        assert decided.state == SEARCHING, f"{dialect} arrived as a wall"


def test_a_wall_is_read_with_the_beams_beside_it_and_an_empty_direction_still_answers_nothing():
    """`beam` with a window: one echo in the middle of the window counts, an empty window answers None."""
    ranges = a_scan()
    ranges[round(radians(-90) / INCREMENT)] = 0.40        # one single echoed beam, nothing beside it
    assert beam(ranges, INCREMENT, -pi / 2, window=2) == pytest.approx(0.40)
    assert beam(ranges, INCREMENT, pi / 2, window=2) is None


def test_a_wall_straight_ahead_brakes_without_losing_the_wall_on_the_right():
    decided = steer(a_scan([(0, 0.30), (-90, 0.40)]), INCREMENT, RANGE_MAX)
    assert decided.state == BLOCKED and decided.speed == 0.0
    assert decided.wall == pytest.approx(0.40), "the reference on the right is still what it is following"


# ------------------------------------------------------------------------------- the vector field


def test_the_field_points_away_from_a_wall_and_towards_the_open_side():
    """Both halves in one test, because a field that only ever turns one way is a sign error."""
    wall_on_the_right = field(a_scan([(-60, 0.80)]), INCREMENT, RANGE_MAX)
    assert wall_on_the_right.state == TURNING and wall_on_the_right.direction > 0
    assert wall_on_the_right.turn > 0, "the open side is to the left, so the turn goes left"

    wall_on_the_left = field(a_scan([(60, 0.80)]), INCREMENT, RANGE_MAX)
    assert wall_on_the_left.direction < 0, "the same wall mirrored must give the same answer mirrored"


def test_a_wall_beyond_the_repulsion_range_gets_no_vote_at_all():
    """The `- r` in `(repulsion_range - r) / r`. Without it the far half of a 360-beam scan outvotes the
    near half and the robot steers away from the room instead of away from the thing in front of it."""
    decided = field(a_scan([(90, 3.0)]), INCREMENT, RANGE_MAX, aim=0.0, repulsion_range=2.0)
    assert decided.state == CLEAR and decided.turn == 0.0
    assert decided.direction == pytest.approx(0.0, abs=1e-9), "a wall 3 m away may not bend the field"


def test_a_nearer_wall_ahead_means_a_smaller_forward_command():
    """The braking term on its own: full speed from twice `stop_gap` out, nothing at `stop_gap`.

    Measured with one wall only, this compared two different *directions* rather than two speeds — a wall
    dead ahead votes in the field as well as in the brake, so 1.5 m and 0.85 m differ in which way the robot
    is pointed and the assertion was measuring the field. Here the wall that steers sits to the right in
    both halves, and the wall in the nose cone is outside `repulsion_range` in both, so the only thing left
    to change between the two calls is how hard the approach may be driven.
    """
    steer_by = (-60, 0.55)
    far = field(a_scan([(0, 3.00), steer_by]), INCREMENT, RANGE_MAX, aim=0.0, repulsion_range=0.6)
    near = field(a_scan([(0, 0.70), steer_by]), INCREMENT, RANGE_MAX, aim=0.0, repulsion_range=0.6)
    assert far.direction == pytest.approx(near.direction, abs=1e-9), "the way chosen must not be what changed"
    assert far.forward > near.forward, "the same wall, nearer, must not ask for more speed"


def test_a_corridor_that_closes_brings_the_robot_to_a_standing_stop():
    decided = field(a_scan([(0, 0.40)]), INCREMENT, RANGE_MAX, aim=0.0, stop_gap=0.55)
    assert decided.state == STOPPED
    assert (decided.forward, decided.sideways, decided.turn) == (0.0, 0.0, 0.0), \
        "a stopped field that still turns is a robot that pivots into the wall"
    assert "0.40" in decided.why, decided.why


def test_straight_ahead_means_twenty_degrees_and_not_the_whole_front_half():
    """±0.35 rad of `nearest_ahead`. Wider, and the wall of a corridor 60 cm to the side counts as "ahead"
    and stops the robot dead in a corridor it could have driven down."""
    assert nearest_ahead(a_scan([(60, 0.40)]), INCREMENT, RANGE_MAX) is None
    assert nearest_ahead(a_scan([(10, 0.40)]), INCREMENT, RANGE_MAX) == pytest.approx(0.40)


def test_the_turn_to_a_wanted_heading_goes_the_short_way_round():
    """170° to -170° is 20°, not 340° — the most common first bug in a heading controller.

    Both files carry their own `wrap` on purpose, so both are checked rather than one being assumed.
    """
    assert wrap(radians(340)) == pytest.approx(-radians(20))
    assert obstacle_avoidance.wrap(pi * 3) == turn_and_move.wrap(pi * 3)
    off_a_small_angle = turn_towards(radians(170), radians(-170), gain=1.8, limit=1.2)
    assert off_a_small_angle.turn > 0, "the short way from 170° to -170° is through 180°"
    assert off_a_small_angle.turn == pytest.approx(1.8 * radians(20))


# ------------------------------------------------------------------------------- the three primitives


def test_turn_ninety_means_ninety_degrees_and_not_ninety_radians():
    assert parse_command("turn 90") == ("turn", pytest.approx(pi / 2))
    assert parse_command("rotate 45")[1] == pytest.approx(pi / 4)
    assert parse_command("drive 1.5") == ("drive", 1.5)
    assert parse_command("stop") == ("stop", 0.0)


def test_a_command_that_is_not_a_command_comes_back_with_the_reason():
    """The lecturer types this at a whiteboard, so the answer has to be readable in the log line."""
    for typed in ("", "turn", "dance 3"):
        what, why = parse_command(typed)
        assert what is None, f"{typed!r} was taken as a command"
        assert "turn" in why or "stop" in why, f"{typed!r} answered with {why!r}"


def test_a_turn_is_proportional_and_capped_at_what_the_wheels_can_do():
    half_a_radian = turn_towards(0.0, 0.2, gain=1.8, tolerance=0.05)
    assert half_a_radian.running and half_a_radian.turn == pytest.approx(0.36)
    assert half_a_radian.forward == 0.0, "a turn is a turn, not a curve"
    # Which way round at exactly 180 degrees is not asserted and cannot be: `wrap` answers -pi there, and
    # both ways are the same distance. What this test is about is the ceiling, and the ceiling is a magnitude.
    assert abs(turn_towards(0.0, pi, gain=1.8, limit=1.2).turn) == pytest.approx(1.2), \
        "1.8 * pi rad/s is a command the wheels ignore; a limited one is the only one worth sending"


def test_a_turn_is_finished_by_its_tolerance_rather_than_by_arriving():
    """A P controller gets asymptotically closer and never arrives, so the tolerance declares it done."""
    assert not turn_towards(1.0, 1.02, tolerance=0.05).running
    assert turn_towards(1.0, 1.06, tolerance=0.05).running


def test_driving_straight_stops_at_its_distance():
    assert drive_straight(1.0, 1.0) == drive_straight(1.2, 1.0)                     # both done
    assert not drive_straight(0.99, 1.0).running, "inside the tolerance of the distance is arrived"
    assert drive_straight(0.0, 1.0).forward == pytest.approx(0.25)


def test_the_last_centimetres_of_a_drive_are_driven_slowly():
    """`remaining / 0.15` is the deceleration phase; without it the stop overshoots by one cycle."""
    nearly_there = drive_straight(0.0, 0.05, speed=1.0)
    assert nearly_there.running and nearly_there.forward == pytest.approx(0.05 / 0.15)
    assert nearly_there.forward < 1.0


def test_holding_the_line_pushes_sideways_without_touching_the_forward_speed():
    """Only a mecanum base can do this, which is the cheapest demonstration of what the wheels are for."""
    off_to_the_left = drive_straight(0.0, 1.0, cross_track=0.20)
    assert off_to_the_left.sideways < 0, "the line is 20 cm to the right, so the correction goes right"
    assert off_to_the_left.forward == pytest.approx(0.25)
    assert drive_straight(0.0, 1.0, cross_track=5.0).sideways == pytest.approx(-0.12), \
        "a correction beyond the wheels' reach is a command the wheels ignore"


def test_driving_to_a_place_walks_diagonally_at_it_and_turns_to_face_it():
    decided = drive_to((0.0, 0.0, 0.0), (1.0, 0.5))
    assert decided.running and decided.forward > 0 and decided.sideways > 0 and decided.turn > 0, \
        "the goal is ahead and to the left: forward, left, and turning left, all at once"


def test_a_goal_within_arrive_distance_is_arrived_rather_than_approached():
    arrived = drive_to((0.0, 0.0, 0.0), (0.05, 0.0), arrive_distance=0.12)
    assert not arrived.running
    assert (arrived.forward, arrived.sideways, arrived.turn) == (0.0, 0.0, 0.0)


def test_the_heading_integral_cannot_grow_while_a_robot_is_held_against_a_wall():
    """`integral_limit` is the anti-windup. Without it, 30 s of being stuck is a minute of spinning after."""
    error_sum = 0.0
    for _ in range(200):                                    # 10 s of a robot held from turning at 20 Hz
        decided = drive_to((0.0, 0.0, 0.0), (0.0, 1.0), error_sum, dt=0.05)
        error_sum = decided.error_sum
        assert abs(error_sum) <= 1.0, "the sum left the anti-windup limit"
    assert error_sum == pytest.approx(1.0), "held for 10 s, the term should have saturated, not drifted"


# ------------------------------------------------------------------------------- the split itself


def test_the_maths_of_every_reactive_module_imports_without_ros(monkeypatch):
    """`test_reactive.py` is the reason for the guarded imports, so the guard is checked, not assumed.

    With the ROS modules unavailable, the files must still import — that is what `field` and `steer` being
    plain functions is for — and it has to be `main` that refuses, in words that say what to source.

    `move_to_point.py` is in this list because it wants the same property, and the two message modules added
    below are in it because its overlay imports and `view_markers`' are the newest guarded ones in the
    package: a marker type imported outside a `try` would break the one thing this file is for.
    """
    from importlib import reload

    modules = (wall_following, obstacle_avoidance, turn_and_move, move_to_point)
    for name in ("rclpy", "rclpy.node", "rclpy.action", "geometry_msgs", "geometry_msgs.msg",
                 "sensor_msgs", "sensor_msgs.msg", "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg",
                 "visualization_msgs", "visualization_msgs.msg", "builtin_interfaces",
                 "builtin_interfaces.msg"):
        monkeypatch.setitem(sys.modules, name, None)            # `import x` on a None entry raises

    for module in modules:
        without_ros = reload(module)
        assert without_ros.rclpy is None, f"{module.__name__} imported rclpy anyway"
        with pytest.raises(SystemExit) as refused:
            without_ros.main()
        assert "rclpy" in str(refused.value), f"{module.__name__}.main() said {refused.value!r}"

    assert wall_following.steer(a_scan(), INCREMENT, RANGE_MAX).state == SEARCHING, \
        "the rule itself is supposed to work with no ROS in the process at all"

    monkeypatch.undo()                                          # the real ROS, for whichever test is next
    for module in modules:
        reload(module)


def test_with_ros_present_the_guard_imports_every_message_name_it_uses():
    """The mirror image of the test above, and the one that has earned its keep once already.

    A guarded import that puts a whole block under one `except ImportError:` cannot tell "this machine has no
    ROS" from "this file asked for a message that is not where it said" — and answers both with the same hint
    about sourcing a setup file. `move_to_point.py` did exactly that for a day: it wanted `Odometry` from
    `nav_msgs` and asked `sensor_msgs` for it, and the guard told the author to install ROS on a machine that
    had it. So on a machine that does have ROS, every name behind every guard in the package must be a class.
    """
    names = {wall_following: ("Twist", "LaserScan", "MarkerArray"),
             obstacle_avoidance: ("Twist", "LaserScan", "MarkerArray"),
             turn_and_move: ("Twist", "PoseStamped", "Odometry", "String"),
             move_to_point: ("Twist", "PoseStamped", "Odometry", "String", "MarkerArray"),
             view_markers: ("Marker", "MarkerArray", "Point", "Duration")}
    for module, given in names.items():
        missing = [name for name in given if getattr(module, name, None) is None]
        assert not missing, f"{module.__name__} has {missing} as None: either this machine has no ROS, or the " \
                            "name is on the wrong module and the guard has hidden that"
    assert hasattr(turn_and_move, "Node") and turn_and_move.Node is not object, \
        "`Node = object` in a guard means the same lie about rclpy"
    assert view_markers.available(), "the guards are fine but the overlay says it is not there anyway"


def test_every_executable_a_launch_file_starts_is_installed_and_every_one_installed_is_launched():
    """`Node(package=..., executable=...)` is a string: a missing console_script is a launch-time error and
    the kind of typo that no import catches, so the two lists are compared here.

    Compared by walking `launch/*.py` rather than a list of file names written here, because the list was the
    weak part: a new launch file simply was not checked, and the one failure mode of this test is exactly a
    file nobody thought to add. Checked both ways for the same reason — an entry point no launch file starts
    is an executable that is installed, documented and never run, which is how a demo rots.
    """
    installed = set(re.findall(r"([\w-]+)\s*=\s*ohm_frontier\.\w+:main", (PACKAGE / "setup.py").read_text()))
    assert {"frontier_node", "wall_following", "obstacle_avoidance", "turn_and_move",
            "move_to_point"} <= installed, installed

    # `package=` and `executable=` are adjacent in every launch file of this package, which is what lets one
    # regex ask "which executables of *this* package" without dragging in the nav2 and slam_toolbox ones.
    starts = re.compile(r'package="ohm_frontier",\s*\n?\s*executable="([\w-]+)"')
    launched = set()
    for launch in sorted((PACKAGE / "launch").glob("*.py")):
        started = set(starts.findall(launch.read_text()))
        launched |= started
        assert started <= installed, f"{launch.name} starts {started - installed}, which setup.py does not build"
    assert launched == installed, f"installed but launched by nothing: {installed - launched}"
