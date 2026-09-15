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

The base is mecanum, so `drive_to` and `drive_straight` *could* use `vy` and hold a line sideways while they
turn. They do not, any more, unless they are asked to: `strafe` is false by default and the command is
`vx` and `wz`. What decided that is a measurement, not a preference — one 6.0 m goal run in `open` with the
lateral term live: **49 % of the samples carried a sideways command of up to 0.15 m/s, and 1.36 m of the 8.51 m
the robot travelled was sideways travel** for 5.90 m of net displacement (path/net 1.4, against 1.2 for the
same hall driven by `move_to_point.py`, which has no lateral term at all). The robot got there. It also read,
to anyone standing in the hall, as a robot going sideways for no visible reason, which is what an audience
calls misbehaving. `strafe:=true` puts the mecanum sum back for the comparison, and on the steering car of the
same simulator the crab version is simply a bug — the simulator drops `vy` with a warning, because a car with
one steering axle cannot strafe (`mecanum_lab/steering.py`), and there the turn has to come first.

**Which term saturates, and what it saturates on.** The wall follower's bug was a metres error multiplied by a
gain tuned in centimetres, and this file has the same shape in two places:

* the lateral term asks for `gain_side × cross-track` = 0.7 × 5.0 = **3.5 m/s of strafe** for a goal 5 m to the
  side, and `side_limit` gives it 0.15 m/s — 4 % of what the proportional rule wants. It stops being
  proportional at 0.15 / 0.7 = **0.21 m** of cross-track error, so every error worth correcting is in the clamp;
  that is the term the `strafe` switch takes out of the default, and the reason the default still arrives is
  that with the switch off, cross-track error is not multiplied by anything: it becomes part of the heading
  error, which is an angle, is bounded by π, and is closed by turning.
* the turn term `gain_turn × off + gain_integral × error_sum` reaches its `turn_limit` of 1.2 rad/s at
  1.2 / 1.8 = **0.67 rad = 38°** off the bearing. That one is an honest ceiling — 1.8 × π = 5.7 rad/s is a
  command these wheels ignore — and it is the reason a 90° start costs 1.3 s of turning before any forward
  motion, which is the price of a car-like approach and worth seeing.
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

from .angles import wrap            # the one place; see `angles.py` on why not three

IDLE, TURNING, STRAIGHT, GOING, ARRIVED = "waiting for a command", "turning", "driving straight", \
    "driving to the goal", "arrived"

#: One controller's output: the two velocity components (x forward, y left), the turn rate, whether the
#: command is still running, and — for the PI one — the heading-error sum to carry into the next cycle.
#: A controller with an integral has state; keeping it in the return value is what lets all three be pure
#: functions, and a test can call them as numbers instead of as a node.
Motion = namedtuple("Motion", "forward sideways turn running error_sum")


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
                   speed=0.25, hold_line_gain=0.8, side_limit=0.12, tolerance=0.03, strafe=False):
    """Drive `distance` metres along the line the command started on, staying on it.

    Two things to watch in this one:

    * the speed is capped by `remaining / 0.15` as well as by `speed`, so the last 15 cm are driven slowly
      and the stop is not a crash into the tolerance. Remove the division and the robot overshoots by
      however far it travelled in one cycle — at 20 Hz and 0.25 m/s that is only 1.2 cm, but at 1 m/s it is
      5 cm, and that is the whole story of why a stop needs a deceleration phase;
    * `cross_track` is corrected with `vy`, sideways, while the robot keeps facing down the line — but only
      when `strafe` is on, because only a mecanum base can do it at all. With the switch off the drift is
      measured and ignored: a car cannot hold a line with its bumper, and the alternative — leaning the nose
      by `atan(error / lead)`, the wall follower's trick — would turn `drive 3.0` into a curve, which is not
      what that command says. Ask for `strafe:=true` and watch the same command walk the line sideways.
    """
    remaining = distance - driven
    if remaining <= tolerance:
        return Motion(0.0, 0.0, 0.0, False, 0.0)
    forward = min(speed, remaining / 0.15)
    sideways = max(-side_limit, min(side_limit, -hold_line_gain * cross_track)) if strafe else 0.0
    return Motion(forward, sideways, 0.0, True, 0.0)


def drive_to(pose, goal, error_sum=0.0, gain_forward=0.7, gain_side=0.7, gain_turn=1.8,
             gain_integral=0.25, integral_limit=1.0, speed=0.3, side_limit=0.15,
             turn_limit=1.2, arrive_distance=0.12, dt=0.05, strafe=False):
    """Drive to a place, facing it when you get there: P on the distance, PI on the heading.

    `pose` is (x, y, heading) as the odometry reports it, `goal` is (x, y). Both in the same frame — see
    the module docstring for the one sentence that has to be said about that.

    With `strafe` off — the default — the two terms are `vx` and `wz` only: forward is proportional to the part
    of the distance that lies along the nose, and the turn closes the angle that part is missing. That is what
    makes it a car, and it is still not `move_to_point.orient_then_drive`, because nothing here stops driving
    to aim: forward only falls to zero beyond 90° off the bearing, where driving would be driving backwards.
    Measured on the closed loop at the node's own gains, 6.0 m from a spawn facing 90° away: 21.0 s, 6.02 m of
    path, 0.119 m of residue, and one stop in the whole run — the arrival. With `strafe` on the same controller
    walks diagonally, which is the comparison the pair is for.

    The turn term carries an integral: a P-only heading controller aims exactly where it is told to aim, which
    at a constant small heading error is a robot that drives a permanent arc and never points anywhere.
    `error_sum` comes in and goes out again rather than living in the function, so this stays a pure function.
    It is integrated only while the proportional term alone is *inside* `turn_limit` — see below — and
    `integral_limit` remains as the second bound: without it a robot held against a wall for 30 s accumulates
    an error sum that takes a minute of turning to spend afterwards.

    **The lateral term is the one that saturates like the wall follower's did.** `gain_side × side` is a gain in
    seconds-per-metre on an error in metres, and `side_limit` cuts it at 0.15 / 0.7 = **0.21 m** of cross-track
    error: at 5 m off the line the rule wants 3.5 m/s of strafe and is told 0.15 m/s, 4 % of the answer. That
    is why it is behind `strafe` and off by default. With the switch off there is no metre gain left in the
    controller at all: the only error the default path multiplies is an angle, and the only ceiling on it is a
    rate the wheels can reach.

    **The integral does not integrate a turn the wheels are already refusing.** `error_sum` used to grow on the
    heading error whatever the command looked like, so a robot pressed against a shelf with its place 90° off
    — where the turn sits at its 1.2 rad/s clamp for as long as the shelf lasts — wound the sum to its whole
    1.0 rad·s clamp in 1.0 / (1.571 × 0.05) = **13 cycles, 0.65 s**, and spent it afterwards swinging the nose
    past the bearing. Now nothing is added while `gain_turn × off` is itself past `turn_limit`, which is the
    same 0.67 rad = 38° where the proportional term hits its ceiling: outside it the wheels are already turning
    as fast as they can towards the bearing, so integrating more of the error cannot act, and only has to be
    undone later. Inside 38° the integral still integrates, which is where a heading bias — `sensors.odom
    .bias_omega`, 0.0 in this simulator by default, which is why the integral changes nothing measurable about
    arrival here — is what a P-only controller cannot cancel.
    """
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    distance = hypot(dx, dy)
    if distance <= arrive_distance:
        return Motion(0.0, 0.0, 0.0, False, 0.0)

    heading = pose[2]
    along = dx * cos(heading) + dy * sin(heading)           # both errors in the robot's own frame
    side = -dx * sin(heading) + dy * cos(heading)           # positive: the goal is to the left
    off = wrap(atan2(dy, dx) - heading)
    carrying = abs(gain_turn * off) < turn_limit            # past that the wheels are refusing the command
    total = error_sum if not carrying else \
        max(-integral_limit, min(integral_limit, error_sum + off * dt))

    return Motion(max(0.0, min(speed, gain_forward * along)),
                  max(-side_limit, min(side_limit, gain_side * side)) if strafe else 0.0,
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
            "strafe": False,                # vy at all. Off, because 49 % of the samples of a 6 m run carried a
                                            # sideways command of up to 0.15 m/s and 1.36 m of its 8.51 m of path
                                            # was sideways travel for 5.90 m of net — a robot that arrives while
                                            # going sideways reads to a hall as a robot that misbehaves. On, the
                                            # mecanum sum holds a line and walks diagonally at a place, which is
                                            # the thing this base can do and the car cannot.
            "period": 0.05,
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.settings = dict(speed=float(self.get_parameter("speed").value),
                             turn_limit=float(self.get_parameter("turn_limit").value),
                             arrive_distance=float(self.get_parameter("arrive_distance").value),
                             tolerance=float(self.get_parameter("turn_tolerance").value))
        self.strafe = bool(self.get_parameter("strafe").value)
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
                               f"a pose on {self.get_parameter('goal_topic').value}, "
                               f"{'strafing' if self.strafe else 'car-like: vx and wz only'}")

    # ------------------------------------------------------------------------------- what comes in

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, 2.0 * atan2(q.z, q.w))

    def on_command(self, msg: String):
        what, amount = parse_command(msg.data)
        if what is None:
            self.get_logger().warning(amount)                    # `amount` carries the reason
            return
        if what == "stop":
            self.mode, self.goal, self.error_sum = IDLE, None, 0.0
            self.get_logger().info(f"{IDLE} — commanded")
            return
        if self.pose is None:
            self.get_logger().warning(f"'{msg.data}' has to wait: no odometry on /{self.robot}/odom yet, "
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
            # A command typed last outranks a goal, and the frontier node republishes its choice twice a
            # second — so this branch would be taken thousands of times a minute and is said once per goal
            # in the log rather than never. Principle: a decision the node makes is a decision it says.
            self.get_logger().info(f"goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) left alone: "
                                   f"the node is {self.mode} on a typed command, which outranks it until 'stop'")
            return
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.error_sum, self.mode = 0.0, GOING
        if msg.header.frame_id and not self.said_frames:
            self.get_logger().warning(f"goal arrives in frame '{msg.header.frame_id}' and this node has no tf "
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
            self.get_logger().info(f"{ARRIVED}: {self.finished_in_words()}")
            self.mode, self.goal = ARRIVED, None

    def finished_in_words(self) -> str:
        """What an arrival means in the one unit this node can check: metres to the place it was sent to.

        `turn` and `drive` finish on their own numbers — a heading, a distance of wheels — and both of those
        are the odometry's word, which is the thing this file exists to doubt. A goal is different: the
        residue is a fact about the place, so the line prints it, and a stop that reports 0.12 m to go is
        visibly a tolerance being met rather than a robot claiming success.
        """
        if self.mode == GOING:
            left = hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
            return (f"the goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}) is {left:.2f} m from here, "
                    f"{arrive_words(left, self.settings['arrive_distance'])}, at "
                    f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, {degrees(self.pose[2]):.0f}°)")
        return (f"{self.mode} done at "
                f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, {degrees(self.pose[2]):.0f}°)")

    def decide(self):
        """The one controller the current mode names, on the numbers the odometry has."""
        if self.mode == TURNING:
            return turn_towards(self.pose[2], self.wanted_heading, limit=self.settings["turn_limit"],
                                tolerance=self.settings["tolerance"])
        if self.mode == STRAIGHT:
            along, cross = self.along_the_line()
            return drive_straight(along, self.distance, cross, speed=self.settings["speed"],
                                  strafe=self.strafe)
        if self.mode == GOING:
            decided = drive_to(self.pose, self.goal, self.error_sum, speed=self.settings["speed"],
                               turn_limit=self.settings["turn_limit"], strafe=self.strafe,
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


def arrive_words(left: float, tolerance: float) -> str:
    """The one judgement the arrival line closes with, in words rather than in a second number.

    `drive_to` stops on `arrive_distance`, not on the place, so 0.11 m reads as "inside the tolerance this node
    was started with" and 0.30 m would read as a bug — the two look the same as a number on a screen.
    """
    return (f"inside the {tolerance:.2f} m tolerance this node was started with"
            if left <= tolerance else f"outside its own {tolerance:.2f} m tolerance")


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
