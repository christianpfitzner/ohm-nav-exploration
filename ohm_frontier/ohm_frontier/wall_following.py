"""Follow the wall on the right hand with nothing but the lidar — no map, no planner, no tf.

    ros2 launch ohm_frontier reactive_wall_follow.launch.py
    ros2 launch ohm_frontier reactive_wall_follow.launch.py world:=maze

The reactive family in this package is three examples of navigation without a map, one per question a
lecture asks. This one is "what does a rule that only looks at the last measurement do?": three numbers
per cycle — how far the wall is to the right, how far it is to the right-ahead and to the right-behind —
and one rule, keep the first one at `wall_distance` and let the difference of the other two tell you
which way the wall is turning. It follows a corridor, a room's outline and a shelf in the production
hall; it cannot cross open floor, it cannot find a door it has to turn towards, and it gets nothing out
of a wall it has already followed. That is the lesson: reactive means fast, cheap and hopeless the moment
the geometry stops being a corridor.

Which side is which, because this is where a lidar program goes wrong: the simulator's `LaserScan`
starts at the robot's own nose (`angle_min = 0`) and its beam index grows **counterclockwise**, so the
right hand is at negative angles, which is the same beam as index 3/4 of the circle. See
`mecanum_lab/sensors.py`, where the beam directions are built.

The two dialects of a missing echo are both handled. The simulator reports a beam that came back
nowhere as its own `range_max` unless it was started with `lidar_no_echo:=inf`, which writes infinity,
and numpy-style `nan` shows up in recordings. All three mean the same thing to this file: no wall there.
"""
from collections import namedtuple
from math import pi

try:                                    # the rule below is importable without ROS, and is tested that way
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
except ImportError:                     # so `test_reactive.py` can read the steering alone; `main` says so
    rclpy = Twist = LaserScan = None
    Node = object

AHEAD, RIGHT, RIGHT_AHEAD, RIGHT_BEHIND = 0.0, -pi / 2, -pi / 4, -3 * pi / 4
FOLLOWING, SEARCHING, BLOCKED = "following", "wall lost — turning until one comes back", "blocked ahead"

#: what one cycle decided: the two wheel commands, which of the two states the robot is in, and how far
#: the wall it is following is. A namedtuple rather than a tuple because a test wants to name these.
Steering = namedtuple("Steering", "speed turn state wall")


def beam(ranges, angle_increment, angle, window=1):
    """How far the wall is in this direction, or None where nothing reflected.

    `window` beams either side are searched for the nearest echo: one beam of a 360-beam scan is 1° wide
    and 1 cm of noise on a wall at 40 cm is a whole degree, so a single beam answers "nothing" far too
    often for a control loop to trust it.
    """
    count = len(ranges)
    if count == 0:
        return None
    middle = round(angle / angle_increment) % count
    seen = [ranges[(middle + offset) % count] for offset in range(-abs(window), abs(window) + 1)]
    echoed = [float(r) for r in seen if echoed(r)]
    return min(echoed) if echoed else None


def echoed(r) -> bool:
    """Did this beam come back? Both dialects of "no" are the same answer here — see the module docstring."""
    return r == r and r != float("inf") and r > 0.0


def steer(ranges, angle_increment, range_max, want=0.40, speed=0.35,
          distance_gain=0.9, swivel_gain=0.5, turn_limit=1.1, search_turn=0.7, free_ahead=0.5):
    """The right-hand rule, as metres per second and radians per second.

    Two proportional terms, both signed the same way, which is the whole algorithm:

    * **keep the gap** — `side - want`. Further away than `want` says turn right (negative, towards the
      wall); nearer says turn left (positive, away from it).
    * **follow the wall's own turn** — `right_ahead - right_behind`. Where that is positive the wall is
      falling away in front of us, which is a corridor bending right or an opening; steering right before
      the gap is in the middle of the scan is what keeps the robot in the corridor instead of in the wall
      at its end.

    With nothing on the right at all the robot stops and turns on the spot until something comes back,
    because driving forward without a reference is how this demo ends against a shelf.
    """
    side = beam(ranges, angle_increment, RIGHT, 2)
    if side is None or side >= range_max:
        return Steering(0.0, -search_turn, SEARCHING, None)

    turn = -distance_gain * (side - want)           # too far → right, too near → left
    ahead, behind = (beam(ranges, angle_increment, a, 2) for a in (RIGHT_AHEAD, RIGHT_BEHIND))
    if ahead is not None and behind is not None:
        turn -= swivel_gain * (ahead - behind)
    turn = max(-turn_limit, min(turn_limit, turn))

    forward = beam(ranges, angle_increment, AHEAD, 3)
    if forward is not None and forward < free_ahead:
        return Steering(0.0, turn, BLOCKED, side)   # the wall to the right is still the reference
    return Steering(speed, turn, FOLLOWING, side)


class WallFollower(Node):
    """The rule above, on a timer, with the simulator's topics bolted on.

    The scan is 20 Hz and the commands go out at 20 Hz too: the simulator drops wheel commands after
    `cmd_timeout` = 0.35 s (`mecanum_lab/types.py`), so a node that only answers each scan would have the
    robot stand still for a third of every second.
    """

    def __init__(self):
        super().__init__("wall_follower")
        for name, value in {
            "robot": "muster",
            "wall_distance": 0.40,        # m, the gap this node tries to keep on its right hand
            "speed": 0.35,                # m/s — the wheels stop at 0.6 m/s (12 rad/s at r = 0.05 m)
            "distance_gain": 0.9,         # 1/s of turn per metre of gap error
            "swivel_gain": 0.5,           # 1/s of turn per metre of ahead-minus-behind difference
            "turn_limit": 1.1,            # rad/s
            "search_turn": 0.7,           # rad/s while no wall is in sight
            "free_ahead": 0.5,            # m, below this the space ahead counts as blocked
            "period": 0.05,
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.want = float(self.get_parameter("wall_distance").value)
        self.settings = dict(speed=float(self.get_parameter("speed").value),
                             distance_gain=float(self.get_parameter("distance_gain").value),
                             swivel_gain=float(self.get_parameter("swivel_gain").value),
                             turn_limit=float(self.get_parameter("turn_limit").value),
                             search_turn=float(self.get_parameter("search_turn").value),
                             free_ahead=float(self.get_parameter("free_ahead").value))
        self.scan = None
        self.range_max = 8.0
        self.increment = 2 * pi / 360
        self.state = ""

        self.wheels = self.create_publisher(Twist, f"/{self.robot}/cmd_vel", 10)
        self.create_subscription(LaserScan, f"/{self.robot}/scan", self.on_scan, 10)
        self.create_timer(float(self.get_parameter("period").value), self.on_timer)
        self.get_logger().info(f"following the wall on the right at {self.want:.2f} m "
                               f"on /{self.robot}/scan")

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.range_max = float(msg.range_max)
        self.increment = float(msg.angle_increment)

    def on_timer(self):
        if self.scan is None:
            return
        decided = steer(list(self.scan.ranges), self.increment, self.range_max, self.want, **self.settings)
        command = Twist()
        command.linear.x, command.angular.z = decided.speed, decided.turn
        self.wheels.publish(command)
        if decided.state != self.state:                     # one line per state change, not 20 per second
            self.get_logger().info(state_line(decided))
            self.state = decided.state


def state_line(decided: Steering) -> str:
    """What the last cycle decided, in the words a lecture wants beside the window."""
    if decided.wall is None:
        return decided.state
    if decided.state == FOLLOWING:
        return f"{FOLLOWING}: wall {decided.wall:.2f} m, turning {decided.turn:+.2f} rad/s"
    return f"{decided.state} — wall still {decided.wall:.2f} m to the right"


def main(args=None):
    if rclpy is None:
        raise SystemExit("no rclpy here. The steering rule in this file imports without ROS — running "
                         "the node does not. Source a ROS 2 installation.")
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
