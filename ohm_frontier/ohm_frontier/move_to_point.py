"""Drive to a place the slow way, on purpose: turn until it faces the place, then go.

    ros2 launch ohm_frontier move_to_point.launch.py
    ros2 topic pub --once /muster/move_command std_msgs/msg/String  "data: 'go 3.0 2.0'"
    ros2 topic pub --once /muster/move_command std_msgs/msg/String  "data: 'stop'"

The fourth control example in the reactive family, and the one that answers the question a student asks after
the first two: *why* does `turn_and_move.drive_to` turn and drive at the same time, instead of doing the
obvious thing — aim first, then go? This file does the obvious thing, so the two can be started in two
terminals on the same hall with the same goal and compared. `orient_then_drive` has two phases and one rule
each: turn while the place is outside `aim_tolerance` of the nose, drive while it is inside. Nothing is mixed,
so nothing here needs a stability argument — and that is exactly what makes the comparison worth an hour,
because the simultaneous controller is the one that has to be tuned.

What to watch, in the order it happens:

* **The turn costs distance.** Every second spent turning on the spot is a second not driving, and on a
  mecanum base it is a choice rather than a necessity — the same robot can strafe to the same place without
  rotating at all, which is what `turn_and_move.drive_straight`'s `cross_track` term does. Ask the room which
  of the two arrives first, then run both.
* **It re-aims.** Driving is dead straight: `turn` is zero, nothing corrects the line. So the moment the place
  drifts outside the tolerance cone — because the robot slid, because the odometry integrated metres it never
  drove (`robot.slip` = 1.0 in `mecanum_lab/physics.py`) — the phase falls back to `orienting`, and the robot
  stops to look at the goal again. Stop-and-go along a straight line is the signature of a sequential
  controller; the simultaneous one never stops, and never stops *for a reason* either.
* **The tolerance is the accuracy.** `aim_tolerance` 0.08 rad is 4.6°; at a goal 3 m away that allows 24 cm of
  sideways error before this controller notices, which you can compute before the robot moves. Then start it
  with `aim_tolerance:=0.25` and watch it arrive half a metre wide and still report `arrived`. No other
  parameter in this package is so cheap to demonstrate.
* **Whose odometry is this?** Nothing in this graph authenticates a topic. The first 58 s run of this file
  produced 32 phase changes and a heading error that jumped between two values a metre of robot motion apart —
  which is not a controller misbehaving, it is a second simulator from somebody else's session, still alive and
  publishing `/muster/odom` on the same domain, feeding one node two different halls. The tell is arithmetic:
  `remaining` went 0.9 m, 10.4 m, 0.87 m, 10.37 m between consecutive 50 ms ticks, and no mecanum robot covers
  9.5 m in one tick. `ros2 topic info /muster/odom --verbose` before a run, and `pkill -f mecanum_lab` after
  it, are two of the cheapest diagnostics in the course.
* **A goal that moves is a goal that never arrives.** Push the goal past the robot with `go` while it is
  driving and it turns, then drives, then turns again. With the frontier node's `goal_topic` wired in
  (`goal_topic:=/frontier_goal`) the goal moves by itself twice a second — which is the same failure the
  frontier node needed `keep_or_switch` for, seen from the driver's seat.

It is navigation in the weakest sense of the word: it takes a place, not a route, and has no notion of
anything in between. `obstacle_avoidance.py` is the same job with a sensor.

    ros2 launch ohm_frontier move_to_point.launch.py view:=true

draws what it is thinking — the line to the goal, the line the nose points along, the command as an arrow and
the two numbers as text (`view_markers.py`). The gap between the two lines *is* `aim_tolerance`, which is the
one quantity in this file that is invisible from the simulator's window.
"""
from collections import namedtuple
from math import atan2, cos, degrees, hypot, sin

try:                                    # the rule below is importable without ROS, and is tested that way
    import rclpy
except ImportError:                     # so the phase rule can be read, and tested, with no ROS present
    rclpy = Twist = Odometry = String = PoseStamped = MarkerArray = None
    Node = object                       # not None: `class MoveToPoint(Node)` has to remain constructible
else:
    # Split from the import above on purpose, because one `except ImportError:` for both writes the wrong
    # diagnosis: this file asked for `Odometry` from `sensor_msgs` for a whole afternoon, and the single guard
    # reported that as "no rclpy here — source a ROS 2 installation" on a machine that had one. A machine
    # without ROS is a one-line hint in `main`; a message name this file got wrong is a bug in this file, and
    # has to say so, because the first is answered by the shell and the second only by reading the code.
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray

from . import view_markers as view
from .angles import wrap

#: What one cycle decided, carrying the two numbers a lecture is watching. `heading_error` and `remaining` go
#: out as a label too (`view.labels`), so the number and the behaviour appear on the same screen at the same
#: moment — which is the whole argument for writing the decision as a value instead of acting on it inline.
Motion = namedtuple("Motion", "phase forward turn heading_error remaining")

ORIENTING, DRIVING, ARRIVED, NO_GOAL = "orienting", "driving", "arrived", "no goal yet"

#: The view's namespaces: the place, the nose, the cone the nose has to stay inside, the command, the
#: arithmetic. RViz switches namespaces, so a lecturer can leave the numbers off and keep the geometry
#: (`view_markers` on why one namespace per kind) — which is also why the cone is drawn rather than written out:
#: two lines at ±`aim_tolerance` off the nose say what the number says, sit next to the goal line at the same
#: time, and are big enough for a hall to read.
GOAL_NAMESPACE, AIM_NAMESPACE, CONE_NAMESPACE = "goal", "aim", "cone"
COMMAND_NAMESPACE, NUMBERS_NAMESPACE = "command", "numbers"

#: metres of cone edge to draw, so the two lines reach past the width of a doorway and the comparison with the
#: goal line is made where the robot is rather than where the goal is.
CONE_REACH = 3.0

#: metres of arrow per metre per second. 0.3 m/s is 30 cm of arrow at 1:1 and invisible from the back of a
#: room at 1:10; drawn to a stated scale rather than to a flattering one, and the scale is in the label.
COMMAND_SCALE = 2.0


def orient_then_drive(pose, goal, gain_turn=1.8, turn_limit=1.2, speed=0.3, decelerate_over=0.25,
                      aim_tolerance=0.08, arrive_distance=0.12) -> Motion:
    """The two-phase controller: face the place, then drive at it. One rule is in charge per cycle.

    `pose` is `(x, y, heading)` as the odometry reports it, `goal` is `(x, y)`, both in the same frame — see
    `MoveToPoint.on_goal` for what this node does when the goal arrives in somebody else's.

    **Orienting** is proportional on the heading error, ceilinged at `turn_limit`, and no forward motion at
    all: a turn that also drives is a curve, and a curve is `turn_and_move.drive_to`, which this file exists
    to be compared against. Gain and ceiling are the same pair as there for the same reason — a gain of 1.8 on
    a half-turn asks for 5.7 rad/s in the first cycle, past what these wheels reach comfortably, and a command
    the wheels cannot reach is a command they ignore.

    **Driving** is straight: `turn` zero, speed scaled by `min(1, remaining / decelerate_over)` so the last
    `decelerate_over` metres are driven proportionally slower and the stop is a stop rather than an overshoot of
    one 50 ms tick. Nothing corrects the line, on purpose — the correction is that the phase test runs again
    next cycle, and if the place has left the cone the robot stops and faces it.

    Three numbers decide where it ends up — `speed`, `decelerate_over`, `arrive_distance` — and they interact,
    which is how the first version of this function got it wrong. Writing the cap as `remaining /
    decelerate_over` starts the ramp at `speed × decelerate_over` = 0.045 m, which is *inside* an
    `arrive_distance` of 0.12 m: the ramp could never be seen, and the robot stopped from full speed in one
    tick. Scaling by `remaining / decelerate_over` puts the ramp where the name says it is, and the rule is
    worth writing down: **`decelerate_over` must exceed `arrive_distance`**, or the last stretch is driven at
    full speed and reported as a stop.

    **One cone, and the second one was measured away.** The obvious next parameter is hysteresis — enter
    `driving` at `aim_tolerance` but leave it only at a wider one, which is what a switch with a dead band does
    and what any textbook would ask for here. It was implemented, and measured on identical runs across the
    empty hall with the band closed and open: 9 changes of phase per minute against 12, both with a couple of
    changes inside 100 ms of one another. That is no difference, on a controller that re-aims roughly every six
    seconds of driving and re-aims *because it should* — the odometry slips, the bearing to the place moves, and
    noticing that is the whole behaviour. So the file has one cone again: a parameter that changes nothing is a
    parameter a student has to be told about, and the honest answer would be that it does not matter here.

    **Arrived** is measured to the place, not along the path. A controller that finishes on distance driven
    reports success with the robot in a wall; that failure belongs to `turn_and_move.drive_straight` and is
    documented there.
    """
    x, y, heading = pose[0], pose[1], pose[2]
    remaining = hypot(goal[0] - x, goal[1] - y)
    if remaining <= arrive_distance:
        return Motion(ARRIVED, 0.0, 0.0, 0.0, remaining)

    heading_error = wrap(atan2(goal[1] - y, goal[0] - x) - heading)
    if abs(heading_error) > aim_tolerance:
        return Motion(ORIENTING, 0.0, max(-turn_limit, min(turn_limit, gain_turn * heading_error)),
                      heading_error, remaining)
    return Motion(DRIVING, speed * min(1.0, remaining / decelerate_over), 0.0, heading_error,
                remaining)


def parse_command(text: str):
    """What was typed, as `(what, goal)` — or `(None, the reason)`, because a whiteboard is not a debugger.

    `go 3.0 2.0` in the odometry's metres, which is what the simulator's window reports and what somebody
    standing next to the robot can point at. Headings are not asked for: this node steers by itself, and a
    typed heading would make it a third controller in a file whose subject is one.
    """
    words = str(text).split()
    if not words:
        return None, "empty command — try: go 3.0 2.0 | stop"
    if words[0].lower() == "stop":
        return "stop", None
    if words[0].lower() not in ("go", "to"):
        return None, f"unknown command '{words[0]}' — try: go 3.0 2.0 | stop"
    if len(words) < 3:
        return None, "go wants two numbers: go 3.0 2.0"
    try:
        return "go", (float(words[1]), float(words[2]))
    except ValueError:
        return None, f"go wants two numbers, got '{' '.join(words[1:])}'"


class MoveToPoint(Node):
    """The rule above, on a timer, with the simulator's topics and a view bolted on.

    One decision per tick and no state but the goal: what the controller does is a function of where the
    odometry says the robot is, which is what makes `orient_then_drive` testable without a graph and what
    leaves this node four lines of plumbing. Commands go out on every tick (`period` 0.05 s) because the
    simulator drops wheel commands after `cmd_timeout` = 0.35 s (`mecanum_lab/types.py`) — a node that answered
    only the 20 Hz scan would leave the robot standing still for a third of every second.
    """

    def __init__(self):
        super().__init__("move_to_point")
        for name, value in {
            "robot": "muster",
            "goal_topic": "/frontier_goal",      # so a run with no navigation stack has something to drive
            "speed": 0.3,                        # m/s — the wheels stop at 0.6 m/s (see turn_and_move.py)
            "gain_turn": 1.8,                    # rad/s of turn per radian of heading error
            "turn_limit": 1.2,                   # rad/s
            "aim_tolerance": 0.08,               # rad, 4.6°: outside this the robot turns and does not drive
            "arrive_distance": 0.12,             # m, from here the place counts as reached
            "decelerate_over": 0.25,             # m: the last stretch; must exceed arrive_distance
            "period": 0.05,
            "view": True,                        # the overlay; `view:=false` for the projector-poor
            "view_topic": "move_view",
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.settings = {name: float(self.get_parameter(name).value)
                         for name in ("speed", "gain_turn", "turn_limit", "aim_tolerance",
                                      "arrive_distance", "decelerate_over")}
        self.pose, self.goal, self.phase = None, None, NO_GOAL
        self.typed, self.said_no_tf = False, False      # who is driving, and what has been said already
        self.view_pub = None

        self.wheels = self.create_publisher(Twist, f"/{self.robot}/cmd_vel", 10)
        self.create_subscription(Odometry, f"/{self.robot}/odom", self.on_odom, 10)
        self.create_subscription(String, f"/{self.robot}/move_command", self.on_command, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.on_goal, 10)
        if bool(self.get_parameter("view").value) and view.available():
            self.view_pub = self.create_publisher(MarkerArray, str(self.get_parameter("view_topic").value), 10)
        elif view.available():
            self.get_logger().info("view:=false — the phases are logged and not drawn")
        else:
            self.get_logger().info("no visualization_msgs here — the phases are logged and not drawn")
        self.create_timer(float(self.get_parameter("period").value), self.on_timer)
        self.get_logger().info(f"driving to places typed on /{self.robot}/move_command (try: go 3.0 2.0) and "
                               f"to goals on {self.get_parameter('goal_topic').value}")

    # ------------------------------------------------------------------------------- what comes in

    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose, msg.pose.pose.orientation
        self.pose = (p.position.x, p.position.y, 2.0 * atan2(q.z, q.w))     # yaw of a quaternion, flat world

    def on_command(self, msg: String):
        """A place from a person, which outranks the frontier node's until somebody types `stop`.

        Two sources for one goal needs a rule. `frontier_node` republishes its choice twice a second, so
        first-write-wins would erase a typed command within a tick and last-write-wins would erase it within
        half one. Hands-on first, autonomous by default: `go` takes the wheel, `stop` hands it back.
        """
        what, goal = parse_command(msg.data)
        if what is None:
            self.get_logger().warn(goal)                            # `parse_command`'s other answer
            return
        if what == "stop":
            self.goal, self.phase, self.typed = None, NO_GOAL, False
            self.get_logger().info("stopped, goal forgotten, the goals on the goal topic are welcome again")
            return
        self.goal, self.typed = goal, True
        self.get_logger().info(f"driving to ({goal[0]:.2f}, {goal[1]:.2f}) — in the odometry's own metres")

    def on_goal(self, msg: PoseStamped):
        """A place from the frontier node, driven as odometry metres, and only while nobody typed one.

        The frame is the mapper's: `frontier_node.publish` stamps its goal with the frame the map message
        carried, because a mapper anchors `map` wherever its first scan found the robot. This node has no tf on
        purpose — the subject is a controller a student can hold in their head, not a transform stack — so the
        numbers are used as they arrive and the gap is reported once. A robot that drives 0.18 m wide and still
        says `arrived` teaches more than one that quietly compensates; `frontiers.py` and its `pose_on_map`
        carry the measurement of that offset.
        """
        if self.typed:
            return
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        if msg.header.frame_id not in ("odom", f"{self.robot}/odom") and not self.said_no_tf:
            self.said_no_tf = True
            self.get_logger().warn(f"goal in frame {msg.header.frame_id!r}, driven as if it were the odometry "
                                   "— no tf here on purpose; that gap is the mapper's correction")

    # ------------------------------------------------------------------------------- what goes out

    def on_timer(self):
        if self.pose is None:
            return
        decided = (orient_then_drive(self.pose, self.goal, **self.settings) if self.goal is not None
                   else Motion(NO_GOAL, 0.0, 0.0, 0.0, 0.0))
        command = Twist()
        command.linear.x, command.angular.z = decided.forward, decided.turn
        self.wheels.publish(command)
        self.draw(decided)
        if decided.phase != self.phase:                          # one line per change, not twenty a second
            self.get_logger().info(describe(decided, self.goal))
            self.phase = decided.phase

    def draw(self, decided: Motion):
        """The place, the nose, the cone, the command and the arithmetic — five namespaces, one message.

        The aim line is the part of a sequential controller that cannot be seen any other way: goal line and aim
        line coinciding is `driving`, a gap between them is `orienting`. And the cone the two have to stay inside
        *is* `aim_tolerance`, the parameter this file exists to make visible, so it is drawn — which is not a
        stylistic preference. The label used to name it, in words, with the tolerance, the speed and the turn
        rate all in one line; in a screenshot of a real run that rendered as letters half a hall wide, with two
        fragments of it visible at a time. What is written over the robot now is the phase and the two numbers
        that decide it; the geometry carries the tolerance, and the log line still has the whole sentence.

        The arrow is the command at `COMMAND_SCALE`, because what the wheels are told and what the robot does on
        one screen is what a control lecture is actually about.
        """
        if self.view_pub is None or self.pose is None:
            return
        frame = f"{self.robot}/odom"                             # every number here is the odometry's
        stamp = self.get_clock().now().to_msg()
        x, y, heading = self.pose
        markers = [view.lines(frame, stamp, AIM_NAMESPACE,
                              [((x, y), (x + cos(heading), y + sin(heading)))], 0.02, view.GREEN)]
        tolerance = self.settings["aim_tolerance"]
        markers.append(view.lines(
            frame, stamp, CONE_NAMESPACE,
            [((x, y), (x + CONE_REACH * cos(heading + tolerance), y + CONE_REACH * sin(heading + tolerance))),
             ((x, y), (x + CONE_REACH * cos(heading - tolerance), y + CONE_REACH * sin(heading - tolerance)))],
            0.015, view.WHITE))
        if self.goal is not None:
            markers.append(view.lines(frame, stamp, GOAL_NAMESPACE, [((x, y), self.goal)], 0.03, view.BLUE))
        if decided.forward or decided.turn:
            reach = COMMAND_SCALE * hypot(decided.forward, decided.turn)
            markers.append(view.arrows(frame, stamp, COMMAND_NAMESPACE,
                                       [((x, y), (x + reach * cos(heading), y + reach * sin(heading)))],
                                       colour=view.ORANGE))
        markers += view.labels(frame, stamp, NUMBERS_NAMESPACE,
                               [((x, y), f"{decided.phase} · {decided.heading_error:+.2f} rad · "
                                         f"{decided.remaining:.2f} m")])
        view.publish(self.view_pub, markers)


def describe(decided: Motion, goal) -> str:
    """The cycle's decision in the words a lecture wants beside the window."""
    if decided.phase == NO_GOAL:
        return "waiting for a place to drive to — try: go 3.0 2.0 on the command topic"
    if decided.phase == ARRIVED:
        return f"arrived at ({goal[0]:.2f}, {goal[1]:.2f}), {decided.remaining:.2f} m from the place"
    return (f"{decided.phase} towards ({goal[0]:.2f}, {goal[1]:.2f}): {decided.heading_error:+.2f} rad off "
            f"({degrees(decided.heading_error):+.0f}°), {decided.remaining:.2f} m left, driving "
            f"{decided.forward:.2f} m/s turning {decided.turn:+.2f} rad/s")


def main(args=None):
    if rclpy is None:
        raise SystemExit("no rclpy here. The phase rule in this file imports without ROS — running the node "
                         "does not. Source a ROS 2 installation.")
    rclpy.init(args=args)
    node = MoveToPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
