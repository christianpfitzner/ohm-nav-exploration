"""Turn to a heading, drive a distance, drive to a pose: the three primitives, on the odometry alone.

    ros2 launch ohm_frontier reactive_turn_and_move.launch.py

The third reactive example, and the one a lecture drives by hand. One command line per primitive, on a
`std_msgs/String` topic, so the effect of one number is visible without editing a file:

    ros2 topic pub --once /muster/reactive_command std_msgs/msg/String  "data: 'turn 90'"
    ros2 topic pub --once /muster/reactive_command std_msgs/msg/String  "data: 'drive 1.0'"
    ros2 topic pub --once /muster/reactive_command std_msgs/msg/String  "data: 'stop'"

A goal from the frontier node is taken on `/frontier_goal` as well, which is how this node doubles as the
no-nav2 driver of an exploration run: same decisions, no navigation stack, and the failure modes of both
sides visible at once.

The maths is three small controllers, deliberately the textbook ones, because the interesting part of the
demo is what they cannot do:

* **A P controller does not arrive.** `turn_towards` gets asymptotically closer and slower; the tolerance
  is what declares it finished, not the robot. Turn the gain down to 0.5 and watch the last degrees take
  seconds. Add the I term in `drive_to` and watch the overshoot appear — that trade is the lecture.
* **The odometry lies at a wall.** `robot.slip` is 1.0 by default in the simulator, so a robot that has
  driven into a shelf keeps commanding the speed the motor wants and keeps integrating metres it never
  drove (`mecanum_lab/physics.py`). `drive 3.0` into a wall therefore ends after 3 m of *wheels*, with the
  robot in the same place. That is not a bug to fix here: it is the reason a real robot needs something
  other than odometry to know where it is, and it is visible in ten seconds.
* **The map's frame is not the odometry's.** The frontier node publishes goals in the frame of the SLAM
  map; this node has no tf on purpose, so it treats the numbers it is given as odometry coordinates and
  says so once. The gap between the two frames is the mapper's correction for the odometry drifting —
  centimetres in a good run, metres in the exercises that ruin it deliberately, which is a fine demonstration
  to run *after* this one and is precisely why nav2 does not skip it.

The base is mecanum, so `drive_to` may use `vy` and hold its line sideways while it turns. On the steering
car of the same simulator the same code is wrong: the simulator drops `vy` with a warning, because a car
with one steering axle cannot strafe, and the turn has to come first.
"""
from collections import namedtuple
from math import atan2, cos, degrees, hypot, pi, radians, sin

try:                                    # the controllers below import without ROS, and are tested that way
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:                     # so `test_reactive.py` can read the maths alone; `main` says so
    rclpy = PoseStamped = Twist = Odometry = String = None
    Node = object

IDLE, TURNING, STRAIGHT, GOING, ARRIVED = "waiting for a command", "turning", "driving straight", \
    "driving to the goal", "arrived"

#: One controller's output: the two velocity components (x forward, y left), the turn rate, whether the
#: command is still running, and — for the PI one — the heading-error sum to carry into the next cycle.
#: A controller with an integral has state; keeping it in the return value is what lets all three be pure
#: functions, and a test can call them as numbers instead of as a node.
Motion = namedtuple("Motion", "forward sideways turn running error_sum")


def wrap(angle: float) -> float:
    """An angle in radians into -π … +π, so a turn goes the short way round.

    Without it, "from 170° to -170°" is a 340° turn instead of a 20° one — the single most common first bug
    in a heading controller, and the reason this line is worth reading twice. It appears twice in the
    package, here and in `obstacle_avoidance.py`, because each reactive example is meant to be read alone.
    """
    return (angle + pi) % (2 * pi) - pi


def parse_command(text: str):
    """What the lecturer typed, as (what to do, how much) — or (None, why that is not a command).

    Headings are degrees because that is what people speak; `turn 90` means 90° from the heading the robot
    had when the command arrived, not 90° in the odometry frame, because nobody standing at a whiteboard
    knows where the odometry frame is.
    """
    words = str(text).split()
    if not words:
        return None, "empty command — try: turn 90 | drive 1.0 | stop"
    what, amount = words[0].lower(), (float(words[1]) if len(words) > 1 else None)
    if what == "stop":
        return "stop", 0.0
    if amount is None:
        return None, f"{what} wants a number: turn 90 | drive 1.0"
    if what in ("turn", "rotate"):
        return "turn", radians(amount)
    if what in ("drive", "move", "forward"):
        return "drive", amount
    return None, f"unknown command '{what}' — try: turn 90 | drive 1.0 | stop"


def turn_towards(heading: float, wanted: float, gain=1.8, limit=1.2, tolerance=0.05):
    """Turn to an absolute heading, the short way round, and stop within `tolerance` of it.

    Pure proportion: half the distance left is half the turn rate. The `limit` is not decoration — with a
    gain of 1.8 the first cycle of a 180° turn would ask for 5.7 rad/s, which is more than this robot's
    wheels do comfortably, and a command beyond the wheels' reach is a command the wheels ignore.
    """
    off = wrap(wanted - heading)
    if abs(off) <= tolerance:
        return Motion(0.0, 0.0, 0.0, False, 0.0)
    return Motion(0.0, 0.0, max(-limit, min(limit, gain * off)), True, 0.0)


def drive_straight(driven: float, distance: float, cross_track: float = 0.0,
                   speed=0.25, hold_line_gain=0.8, side_limit=0.12, tolerance=0.03):
    """Drive `distance` metres along the line the command started on, staying on it.

    Two things to watch in this one:

    * the speed is capped by `remaining / 0.15` as well as by `speed`, so the last 15 cm are driven slowly
      and the stop is not a crash into the tolerance. Remove the division and the robot overshoots by
      however far it travelled in one cycle — at 20 Hz and 0.25 m/s that is only 1.2 cm, but at 1 m/s it is
      5 cm, and that is the whole story of why a stop needs a deceleration phase;
    * `cross_track` is corrected with `vy`, sideways, while the robot keeps facing down the line. Only a
      mecanum base can do that; it is the cheapest demonstration in the whole set of what the wheels are for.
    """
    remaining = distance - driven
    if remaining <= tolerance:
        return Motion(0.0, 0.0, 0.0, False, 0.0)
    forward = min(speed, remaining / 0.15)
    sideways = max(-side_limit, min(side_limit, -hold_line_gain * cross_track))
    return Motion(forward, sideways, 0.0, True, 0.0)


def drive_to(pose, goal, error_sum=0.0, gain_forward=0.7, gain_side=0.7, gain_turn=1.8,
             gain_integral=0.25, integral_limit=1.0, speed=0.3, side_limit=0.15,
             turn_limit=1.2, arrive_distance=0.12, dt=0.05):
    """Drive to a place, facing it when you get there: P on the distance, PI on the heading.

    `pose` is (x, y, heading) as the odometry reports it, `goal` is (x, y). Both in the same frame — see
    the module docstring for the one sentence that has to be said about that.

    The forward and sideways terms are proportional to the two body-frame errors, so the robot walks
    diagonally at the goal and needs no separate "align first" phase. The turn term carries an integral:
    a P-only heading controller aims exactly where it is told to aim, which at a constant small heading
    error is a robot that drives a permanent arc and never points anywhere. `error_sum` comes in and goes
    out again rather than living in the function, so this stays a pure function; `integral_limit` is the
    anti-windup — without it a robot held against a wall for 30 s accumulates an error sum that takes a
    minute of turning to spend afterwards.
    """
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    distance = hypot(dx, dy)
    if distance <= arrive_distance:
        return Motion(0.0, 0.0, 0.0, False, 0.0)

    heading = pose[2]
    along = dx * cos(heading) + dy * sin(heading)           # both errors in the robot's own frame
    side = -dx * sin(heading) + dy * cos(heading)           # positive: the goal is to the left
    off = wrap(atan2(dy, dx) - heading)
    total = max(-integral_limit, min(integral_limit, error_sum + off * dt))

    return Motion(max(0.0, min(speed, gain_forward * along)),
                  max(-side_limit, min(side_limit, gain_side * side)),
                  max(-turn_limit, min(turn_limit, gain_turn * off + gain_integral * total)),
                  True, total)


class TurnAndMove(Node):
    """The three controllers, one command topic, one goal topic, and the odometry to compare them with."""

    def __init__(self):
        super().__init__("turn_and_move")
        for name, value in {
            "robot": "muster",
            "goal_topic": "/frontier_goal",
            "speed": 0.3,                   # m/s — the wheels stop at 0.6 m/s (12 rad/s at r = 0.05 m)
            "turn_limit": 1.2,              # rad/s
            "arrive_distance": 0.12,        # m, from here a goal counts as reached
            "turn_tolerance": 0.05,         # rad, from here a turn counts as finished
            "period": 0.05,
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.settings = dict(speed=float(self.get_parameter("speed").value),
                             turn_limit=float(self.get_parameter("turn_limit").value),
                             arrive_distance=float(self.get_parameter("arrive_distance").value),
                             tolerance=float(self.get_parameter("turn_tolerance").value))
        self.pose = None                    # (x, y, heading) as the odometry reports it
        self.mode = IDLE
        self.wanted_heading = 0.0           # for TURNING, absolute in the odometry frame
        self.start = (0.0, 0.0, 0.0)        # for STRAIGHT: where the line began, and its direction
        self.distance = 0.0
        self.period = float(self.get_parameter("period").value)
        self.goal = None                    # for GOING, (x, y)
        self.error_sum = 0.0                # the I term of `drive_to`, the only state this node keeps
        self.said_frames = False

        self.wheels = self.create_publisher(Twist, f"/{self.robot}/cmd_vel", 10)
        self.create_subscription(Odometry, f"/{self.robot}/odom", self.on_odom, 10)
        self.create_subscription(String, f"/{self.robot}/reactive_command", self.on_command, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.on_goal, 10)
        self.create_timer(self.period, self.on_timer)
        self.get_logger().info(f"{IDLE}: turn 90 | drive 1.0 | stop on /{self.robot}/reactive_command, "
                               f"a pose on {self.get_parameter('goal_topic').value}")

    # ------------------------------------------------------------------------------- what comes in

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, 2.0 * atan2(q.z, q.w))

    def on_command(self, msg: String):
        what, amount = parse_command(msg.data)
        if what is None:
            self.get_logger().warn(amount)                    # `amount` carries the reason
            return
        if what == "stop":
            self.mode, self.goal, self.error_sum = IDLE, None, 0.0
            self.get_logger().info(f"{IDLE} — commanded")
            return
        if self.pose is None:
            self.get_logger().warn(f"'{msg.data}' has to wait: no odometry on /{self.robot}/odom yet, "
                                   "and every controller here compares itself with it")
            return
        if what == "turn":
            # Relative to the heading the robot has now, because that is what the person typing means.
            self.wanted_heading = wrap(self.pose[2] + amount)
            self.mode, self.goal, self.error_sum = TURNING, None, 0.0
        else:
            self.start, self.distance = self.pose, amount
            self.mode, self.goal, self.error_sum = STRAIGHT, None, 0.0
        self.get_logger().info(f"{msg.data!r} → {self.mode}")

    def on_goal(self, msg: PoseStamped):
        if self.mode in (STRAIGHT, TURNING):
            return                                            # a command typed last wins over a goal
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.error_sum, self.mode = 0.0, GOING
        if msg.header.frame_id and not self.said_frames:
            self.get_logger().warn(f"goal arrives in frame '{msg.header.frame_id}' and this node has no tf "
                                   "— taking its numbers as odometry coordinates. See the module docstring.")
            self.said_frames = True
        self.get_logger().info(f"{GOING}: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    # ------------------------------------------------------------------------------- what goes out

    def on_timer(self):
        if self.pose is None:
            return
        decided = self.decide()
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = decided.forward, decided.sideways, \
            decided.turn
        self.wheels.publish(command)
        if not decided.running and self.mode != ARRIVED and self.mode != IDLE:
            self.get_logger().info(f"{ARRIVED}: {self.mode} done at "
                                   f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, "
                                   f"{degrees(self.pose[2]):.0f}°)")
            self.mode, self.goal = ARRIVED, None

    def decide(self):
        """The one controller the current mode names, on the numbers the odometry has."""
        if self.mode == TURNING:
            return turn_towards(self.pose[2], self.wanted_heading, limit=self.settings["turn_limit"],
                                tolerance=self.settings["tolerance"])
        if self.mode == STRAIGHT:
            along, cross = self.along_the_line()
            return drive_straight(along, self.distance, cross, speed=self.settings["speed"])
        if self.mode == GOING:
            decided = drive_to(self.pose, self.goal, self.error_sum, speed=self.settings["speed"],
                               turn_limit=self.settings["turn_limit"],
                               arrive_distance=self.settings["arrive_distance"], dt=self.period)
            self.error_sum = decided.error_sum
            return decided
        return Motion(0.0, 0.0, 0.0, False, 0.0)

    def along_the_line(self):
        """How far along the commanded line the robot is, and how far off it: the two parts of the
        displacement from where the command arrived, rotated into the line's own axes."""
        dx, dy = self.pose[0] - self.start[0], self.pose[1] - self.start[1]
        return (dx * cos(self.start[2]) + dy * sin(self.start[2]),
                -dx * sin(self.start[2]) + dy * cos(self.start[2]))


def main(args=None):
    if rclpy is None:
        raise SystemExit("no rclpy here. The three controllers in this file import without ROS — running "
                         "the node does not. Source a ROS 2 installation.")
    rclpy.init(args=args)
    node = TurnAndMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
