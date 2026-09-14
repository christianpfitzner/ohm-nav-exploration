"""Drive in one direction and get around whatever the lidar sees — the vector-field idea, in one function.

    ros2 launch ohm_frontier reactive_avoid.launch.py
    ros2 launch ohm_frontier reactive_avoid.launch.py drive_heading:=90

The second reactive example of the family, answering "how does a robot get around an obstacle it has no
map of?". Every echoed beam nearer than `repulsion_range` throws the robot away from itself, harder the
shorter the beam, the direction the robot would like to go pulls the other way, and the sum is the
direction it drives. Two lines of arithmetic and a corner gets turned, a pillar gets walked around and a
corridor gets followed — and, just as reliably, the robot walks into a concave corner and stays there,
because the repulsions of its two walls add up along the bisector, which is exactly where the wall is.
Both halves of that are the lesson, and `frontiers.py` plus nav2 exist because the second half is not a
defect that can be tuned out of a field like this one.

Why this file commands `vy` as well as `vx`: the base is mecanum, so the sum of the field can be driven
directly instead of being projected onto the driving direction, and a heading to the side is a direction
the robot can actually take. On the steering car of the same simulator this would be a bug — the
simulator answers „steering robot cannot strafe, vy=0.25 dropped" and keeps driving straight
(`mecanum_lab/physics.py`) — which is the comparison worth making in the lecture.

What a lidar is not told is the other half of the honest picture: a table whose top is 30 cm above the
floor reflects its beams over its own edge, so this field has an opinion about the floor and none about
the tabletop.

The heading the field is pulled towards is a parameter, not a plan — this file has no goal and no memory.
`turn_and_move.py` is the one of the three that drives to a place; `aim` is an argument of `field` so that
node can hand it a bearing instead of a constant without the maths being copied.
"""
from collections import namedtuple
from math import atan2, cos, hypot, pi, sin

try:                                    # the field below is importable without ROS, and is tested that way
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
except ImportError:                     # so `test_reactive.py` can read the field alone; `main` says so
    rclpy = Twist = LaserScan = None
    Node = object

CLEAR, TURNING, STOPPED = "clear way", "steering around an obstacle", "stopped"

#: The field's answer, all of it in the robot's own frame: the two velocity components it drives (x
#: forward, y left), the turn, the direction the sum points at — plotting that over a scan is the picture
#: from the slide — which of the three states this is, and the sentence that says why when it stopped.
Decision = namedtuple("Decision", "forward sideways turn direction state why")


def wrap(angle: float) -> float:
    """An angle in radians into -π … +π, so that a turn goes the short way round.

    Needed because `atan2` answers in -π … +π while the wanted heading is anywhere: with `drive_heading`
    at 170° and the field pointing at -170° the difference is 340°, and without this line the robot turns
    through 340° instead of 20°. This one-liner appears twice in the package, here and in
    `turn_and_move.py`, on purpose: each of the three reactive examples is meant to be read alone.
    """
    return (angle + pi) % (2 * pi) - pi


def echoed(r) -> bool:
    """Did this beam come back? A missing echo is either the laser's own range, or infinity, or nan; all
    three mean "no wall here", not "a wall at 8 m". See `wall_following.py` for the two dialects."""
    return r == r and r != float("inf") and r > 0.0


def nearest_ahead(ranges, angle_increment, range_max, half_view=0.35):
    """The nearest echo within ±`half_view` radians of the nose, or None where nothing reflected there.

    ±0.35 rad is ±20°, about a hand at arm's length. Wider than that and the wall of a corridor 60 cm to
    the side counts as "ahead", which stops the robot dead in a corridor it could have driven down.
    """
    count = len(ranges)
    if count == 0:
        return None
    beams = max(1, min(count // 2 - 1, int(half_view / angle_increment)))
    indices = list(range(0, beams + 1)) + list(range(count - beams, count))
    seen = [float(ranges[i]) for i in indices if echoed(ranges[i]) and float(ranges[i]) < range_max]
    return min(seen) if seen else None


def field(ranges, angle_increment, range_max, aim=0.0, repulsion_range=2.0,
          speed=0.35, turn_limit=1.1, turn_gain=1.6, stop_gap=0.55, half_view=0.35):
    """One scan plus a direction the robot would like to go in, out comes the way it drives.

    The attraction is a unit vector at `aim`. The repulsion of one beam is `(repulsion_range - r) / r`
    pointing back along that beam, and both halves of that fraction matter:

    * the `- r` stops a wall's vote at `repulsion_range`. Without it every wall in the hall votes, the far
      half of a 360-beam scan outvotes the near half, and the robot steers away from the room instead of
      away from the thing in front of it;
    * the `/ r` makes it urgent. A wall at 2 m is information, a wall at 30 cm is an argument.

    The speed is then braked by what the nose sees: full speed from twice `stop_gap` out, nothing at
    `stop_gap`, so a corridor that closes is walked into slowly and then not at all.
    """
    pull_x, pull_y = cos(aim), sin(aim)             # the wanted direction, in the robot's own frame
    for index, r in enumerate(ranges):
        distance = float(r)
        if not echoed(r) or distance >= range_max or distance >= repulsion_range:
            continue
        beam_angle = index * angle_increment        # index 0 is the nose, the index grows counterclockwise
        push = (repulsion_range - distance) / distance
        pull_x -= push * cos(beam_angle)
        pull_y -= push * sin(beam_angle)

    direction = atan2(pull_y, pull_x)
    ahead = nearest_ahead(ranges, angle_increment, range_max, half_view)
    if ahead is not None and ahead < stop_gap:
        return Decision(0.0, 0.0, 0.0, direction, STOPPED,
                        f"{ahead:.2f} m of wall straight ahead — this node stops at {stop_gap:.2f} m")

    brake = 1.0 if ahead is None else min(1.0, max(0.0, (ahead - stop_gap) / stop_gap))
    off = wrap(direction - aim)                     # the turn goes the short way, whichever way aim points
    if abs(off) < 0.12:                             # the field agrees with the wish: nothing to report
        return Decision(speed * brake * cos(direction), speed * brake * sin(direction), 0.0,
                        direction, CLEAR, "")
    return Decision(speed * brake * cos(direction), speed * brake * sin(direction),
                    max(-turn_limit, min(turn_limit, turn_gain * off)), direction, TURNING, "")


class ObstacleAvoidance(Node):
    """The field on a timer, on the simulator's wheels.

    Commands go out at 20 Hz even though the scan arrives at 20 Hz too, because the simulator forgets
    wheel commands after `cmd_timeout` = 0.35 s (`mecanum_lab/types.py`). A node that only ever answers a
    scan stops driving the moment the scans stop arriving, which happens to be the right behaviour here
    and for entirely the wrong reason — so the timer says what it is for.
    """

    def __init__(self):
        super().__init__("obstacle_avoidance")
        for name, value in {
            "robot": "muster",
            "drive_heading": 0.0,         # deg, the direction the field is pulled towards; 0 = the nose
            "repulsion_range": 2.0,       # m, beyond this a wall gets no vote
            "speed": 0.35,                # m/s — the wheels stop at 0.6 m/s (12 rad/s at r = 0.05 m)
            "turn_limit": 1.1,            # rad/s
            "turn_gain": 1.6,
            "stop_gap": 0.55,             # m, where "ahead is blocked" has become "stand still"
            "half_view": 0.35,            # rad, how wide "straight ahead" is
            "period": 0.05,
        }.items():
            self.declare_parameter(name, value)

        robot = str(self.get_parameter("robot").value)
        self.aim = float(self.get_parameter("drive_heading").value) * pi / 180.0
        self.settings = dict(repulsion_range=float(self.get_parameter("repulsion_range").value),
                             speed=float(self.get_parameter("speed").value),
                             turn_limit=float(self.get_parameter("turn_limit").value),
                             turn_gain=float(self.get_parameter("turn_gain").value),
                             stop_gap=float(self.get_parameter("stop_gap").value),
                             half_view=float(self.get_parameter("half_view").value))
        self.scan = None
        self.range_max = 8.0
        self.increment = 2 * pi / 360
        self.reported = ""

        self.wheels = self.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
        self.create_subscription(LaserScan, f"/{robot}/scan", self.on_scan, 10)
        self.create_timer(float(self.get_parameter("period").value), self.on_timer)
        self.get_logger().info(
            f"pulling the field towards {float(self.get_parameter('drive_heading').value):.0f}° and away "
            f"from anything nearer than {self.settings['repulsion_range']:.1f} m")

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.range_max = float(msg.range_max)
        self.increment = float(msg.angle_increment)

    def on_timer(self):
        if self.scan is None:
            return
        decided = field(list(self.scan.ranges), self.increment, self.range_max, self.aim, **self.settings)
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = decided.forward, decided.sideways, \
            decided.turn
        self.wheels.publish(command)
        if decided.state != self.reported:                      # one line per change of mind
            self.get_logger().info(
                decided.state + (f": {decided.why}" if decided.why else
                                 f" — the field points {decided.direction:+.2f} rad, "
                                 f"{hypot(decided.forward, decided.sideways):.2f} m/s"))
            self.reported = decided.state


def main(args=None):
    if rclpy is None:
        raise SystemExit("no rclpy here. The vector field in this file imports without ROS — running the "
                         "node does not. Source a ROS 2 installation.")
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
