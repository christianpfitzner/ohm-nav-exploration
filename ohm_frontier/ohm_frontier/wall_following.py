"""Follow the wall on the right hand with nothing but the lidar — no map, no planner, no tf.

    ros2 launch ohm_frontier reactive_wall_follow.launch.py
    ros2 launch ohm_frontier reactive_wall_follow.launch.py world:=maze

The reactive family in this package is three examples of navigation without a map, one per question a
lecture asks. This one is "what does a rule that only looks at the last measurement do?": the lidar is read
in three bearings on the right — how far the wall is to the right, to the right-ahead and to the
right-behind — and from those three numbers come one heading and one speed. It follows a corridor, a room's
outline and a shelf in the production hall; it cannot find a door it has to turn towards, and it gets
nothing out of a wall it has already followed. That is the lesson: reactive means fast, cheap and hopeless
the moment the geometry stops being a corridor.

Three phases, because a wall has to be found before it can be kept:

| the phase | what the right hand sees | what the robot does |
| --- | --- | --- |
| `SEARCHING` | nothing at all | turns on the spot with no forward drive, and says so out loud before `search_timeout` |
| `APPROACHING` | a wall, but farther than `max_wall_range` | closes on it at `find_speed`, squaring itself up to its angle |
| `FOLLOWING` | a wall inside `max_wall_range` | drives the gap to `wall_distance` at `find_speed`, then holds it at `speed` |

**Why the gap term is an angle and not a metres error.** This file used to multiply `side - want` by a gain,
which is a rule that only works while `side` is already near `want`. Measured over 45 s in `rooms` at the
spawn, with the defaults of the version that did that: **17.15 m of path and 0.45 m of net displacement**,
`wz` pinned at its −1.10 rad/s clamp the whole time and the state line printing `following: wall 3.93 m`. A
wall 3.93 m away is 3.43 m of error against the 0.40 m gap that was wanted, times a gain tuned in
centimetres: the proportional term saturated, the robot spun on the spot, the beam swept past the wall and
back, and it printed `following` while having no wall and never stopping to look for one. So metres of gap
error now become degrees of heading — *lean the nose towards the gap you want, `aim_lead` metres ahead* —
which is bounded by construction, and a robot far from a wall drives towards it at an angle instead of
spinning beside it. `aim_lead` is the clamp that keeps 3.4 m of error from asking for more than 45°.

**What the gap is measured from: the kinematic centre, not the sensor.** The scan arrives in the lidar's own
frame, and the wanted gap means metres from `base_link` — the point the wheels turn about, and the point a
metre rule laid on the hall floor would be measured from. For the beam straight to the right, that beam's
own range *is* the perpendicular distance from the sensor whatever the wall's angle, because a contact point
90° off the nose has no forward component: `distance = range − laser_offset_y`, the one correction being the
sensor's own offset from the centre. The simulator mounts the lidar at `[0, 0, 0]` (`robot_frames` → `mount`
in `mecanum_lab/types.py`), so today the two agree by accident of that mount and `laser_offset_y` is 0.0;
move the sensor 7 cm to the left of the centre and this file still means the same metre. The two diagonal
beams are at 45°, so a metre of their slant range is 0.707 m sideways and 0.707 m along the nose; read as
two points of one line they give the wall's **angle**, which is also the sentence a student needs: a single
beam's projection is the perpendicular distance only while the wall is parallel to the heading, and the
angle term is what makes the robot turn with a wall that is not, instead of reading its corner as a nearer
wall.

Which side is which, because this is where a lidar program goes wrong: the simulator's `LaserScan` starts at
the robot's own nose (`angle_min = 0`) and its beam index grows **counterclockwise**, so the right hand is at
negative angles, which is the same beam as index 3/4 of the circle. See `mecanum_lab/sensors.py`, where the
beam directions are built.

The three dialects of a missing echo are all handled. The simulator reports a beam that came back nowhere as
its own `range_max` unless it was started with `lidar_no_echo:=inf`, which writes infinity, and numpy-style
`nan` shows up in recordings. All of them mean the same thing to this file: no wall there — not a wall at
8 m, which is the difference between looking for a wall and chasing the horizon.
"""
from collections import namedtuple
from math import atan, atan2, cos, degrees, pi, sin

try:                                    # the rule below is importable without ROS, and is tested that way
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import MarkerArray
except ImportError:                     # so `test_reactive.py` can read the steering alone; `main` says so
    rclpy = Twist = LaserScan = MarkerArray = None
    Node = object

from . import view_markers as view      # the overlay; `view.available()` answers for the import above

AHEAD, RIGHT, RIGHT_AHEAD, RIGHT_BEHIND = 0.0, -pi / 2, -pi / 4, -3 * pi / 4
DIAGONAL = sin(pi / 4)          # the two diagonal beams are at 45°, so the lateral and the longitudinal
                                # component of one of their slant ranges are the same number

#: the three phases, in the order a wall arrives, plus the two answers that are not phases
SEARCHING = "looking for a wall"
APPROACHING = "wall in sight, closing on it"
FOLLOWING = "following"
BLOCKED = "blocked ahead"
NO_WALL = "no wall found — standing still"

#: what the right hand measured: metres from the kinematic centre, and the wall's own angle relative to the
#: nose (negative means the wall runs away to the right ahead, so following it means turning right).
Side = namedtuple("Side", "distance angle")

#: what one cycle decided. `wall` is the measured gap and is None while there is no reference at all; `aim`
#: is the heading the rule leaned to; `at_gap` says whether the wanted gap has been reached, which is what
#: separates `converging` from `following` on the screen; and `looking` carries the seconds spent without a
#: reference into the next cycle so `search_timeout` can fire. A namedtuple rather than a tuple because a
#: test wants to name these.
Steering = namedtuple("Steering", "speed turn state wall aim at_gap looking")


def clamp(value: float, low: float, high: float) -> float:
    """Inside the range, unchanged; outside it, the nearest edge. Three terms here need it."""
    return max(low, min(high, value))


def beam(ranges, angle_increment, angle, window=1, range_max=None):
    """How far the wall is in this direction, or None where nothing reflected.

    `window` beams either side are searched for the nearest echo: one beam of a 360-beam scan is 1° wide
    and 1 cm of noise on a wall at 40 cm is a whole degree, so a single beam answers "nothing" far too
    often for a control loop to trust it.

    `range_max` is the third dialect of "nothing came back". The simulator writes a missing echo as its own
    maximum range unless it was started with `lidar_no_echo:=inf`, and an 8.0 m reading is the absence of a
    wall, not a wall 8 m away. Left as None this function reports it as a distance — which is what the
    overlay wants when it draws a beam, and what no rule may read.
    """
    count = len(ranges)
    if count == 0:
        return None
    middle = round(angle / angle_increment) % count
    seen = [ranges[(middle + offset) % count] for offset in range(-abs(window), abs(window) + 1)]
    # Named differently from `echoed` on purpose: binding that name here makes it a local of this function,
    # and the filter of the line below then reads it before it has a value. `test_reactive.py` found it by
    # calling the rule with an ordinary scan — UnboundLocalError in the beam every wall is found with.
    returned = [float(r) for r in seen if echoed(r, range_max)]
    return min(returned) if returned else None


def echoed(r, range_max=None) -> bool:
    """Did this beam come back? All three dialects of "no" give the same answer — see the module docstring."""
    return r == r and r != float("inf") and r > 0.0 and (range_max is None or float(r) < range_max)


def side_of(ranges, angle_increment, range_max, laser_offset_y=0.0, window=2):
    """The two numbers this whole rule lives on: the gap at the centre, and the wall's angle.

    Read from three bearings. The beam at −90° off the nose *is* the lateral distance; the two at 45° off
    the right shoulder are two points of the same wall, each 0.707 of its slant range to the side and the
    same amount forward or back, and what differs between them is the wall's angle.

    With the beam to the side blind but both diagonals seeing the wall, the gap is interpolated between them
    — the wall is still there, it is only hidden from the robot's own flank by a post. With one diagonal
    blind there is one point and no line, so the answer is None: look for a wall rather than drive on a
    guess about which way it runs.
    """
    straight = beam(ranges, angle_increment, RIGHT, window, range_max)
    ahead = beam(ranges, angle_increment, RIGHT_AHEAD, window, range_max)
    behind = beam(ranges, angle_increment, RIGHT_BEHIND, window, range_max)
    if straight is None and (ahead is None or behind is None):
        return None

    if ahead is not None and behind is not None:
        # metres to the right, at the longitudinal place each diagonal beam touches the wall: `ahead *
        # DIAGONAL` forward of the centre, `behind * DIAGONAL` behind it. A wall whose right-hand distance
        # grows towards the nose therefore has a positive angle — it runs away to the right, and following it
        # means turning right, which is negative: hence the minus sign.
        across, along = (ahead - behind) * DIAGONAL, (ahead + behind) * DIAGONAL
        angle = -atan2(across, along)
    else:
        angle = 0.0                                     # one point is not a line: assume it is parallel
    metres = straight if straight is not None else (ahead + behind) * DIAGONAL / 2
    return Side(distance=metres - laser_offset_y, angle=angle)


def steer(ranges, angle_increment, range_max, want=0.50, speed=0.35, find_speed=0.25,
          max_wall_range=2.0, follow_tolerance=0.10, aim_lead=1.5, turn_gain=1.6, turn_limit=1.1,
          search_turn=0.7, search_timeout=20.0, free_ahead=0.5, laser_offset_y=0.0,
          looking=0.0, dt=0.05):
    """The right-hand rule, as metres per second and radians per second.

    One heading per cycle, from the two things the right hand knows:

        aim  =  the wall's own angle           (turn with the wall, not at it)
                - atan(gap error / aim_lead)   (and lean towards the gap you want)
        turn =  clamp(turn_gain * aim, ±turn_limit)

    The gap term is a *heading*, bounded by `aim_lead`: however far away the wall is, the most this can ask
    for is 45° off parallel. That clamp is the whole fix described in the module docstring — the same error
    in metres, multiplied by a gain in seconds per metre, is what held this robot's `wz` at its limit for
    45 s while it travelled 45 cm. Leaning 45° at a wall closes the gap at `sin(45°)` of the driving speed,
    which is the honest pace for a wall whose end the lidar cannot see round, and the lean flattens to 0° —
    parallel, at full `speed` — as the error closes.

    `max_wall_range` is the line between a reference and a direction: inside it the measured gap is what the
    rule regulates (`FOLLOWING`), outside it a wall is only somewhere off the right shoulder and the robot
    closes on it (`APPROACHING`). `follow_tolerance` is what counts as being at the gap: inside it the phase
    is `following` at `speed`, outside it the screen says `converging` and the speed stays `find_speed`,
    because a robot still 60 cm off its line has no business driving full pace along it.

    With nothing on the right at all the robot turns on the spot without driving, because driving forward
    without a reference is how this demo ends against a shelf. After `search_timeout` seconds of that it
    stops and says so, because a hall with nothing on its right is the one thing this algorithm cannot fix
    and silent spinning is how it used to pretend otherwise.
    """
    side = side_of(ranges, angle_increment, range_max, laser_offset_y)
    if side is None:
        spent = looking + dt
        if spent >= search_timeout:
            return Steering(0.0, 0.0, NO_WALL, None, 0.0, False, spent)
        return Steering(0.0, -search_turn, SEARCHING, None, -pi / 2, False, spent)

    error = side.distance - want
    at_gap = abs(error) <= follow_tolerance
    aim = side.angle - atan(clamp(error, -aim_lead, aim_lead) / aim_lead)
    turn = clamp(turn_gain * aim, -turn_limit, turn_limit)

    forward = beam(ranges, angle_increment, AHEAD, 3, range_max)
    if forward is not None and forward < free_ahead:
        # the wall on the right is still the reference and the turn still goes to the aim: refusing this
        # cycle's forward drive is the whole answer to a wall in the way
        return Steering(0.0, turn, BLOCKED, side.distance, aim, at_gap, 0.0)
    state = FOLLOWING if side.distance <= max_wall_range else APPROACHING
    return Steering(speed if at_gap else find_speed, turn, state, side.distance, aim, at_gap, 0.0)


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
            "wall_distance": 0.50,        # m from the KINEMATIC CENTRE to the wall it keeps on its right
            "speed": 0.35,                # m/s once at that gap — the wheels stop at 0.6 m/s (12 rad/s at r = 0.05 m)
            "find_speed": 0.25,           # m/s while the gap is not the wanted one yet. A wall leaned on at
                                          # 45° closes at 0.71 of this, so 0.25 m/s takes the 3.4 m the `rooms`
                                          # spawn is from its nearest wall in about 20 s — walking pace, which
                                          # is the right pace for a wall you cannot see the end of
            "max_wall_range": 2.0,        # m: beyond this an echo on the right is a direction to drive to, not
                                          # a gap to regulate. 3.93 m of gap fed into the gap term as if it
                                          # were a reference is the number that spun this robot for 45 s
            "follow_tolerance": 0.10,     # m of gap error that still counts as being at the gap rather than
                                          # converging on it; also the width of the band drawn blue in RViz
            "aim_lead": 1.5,              # m of lookahead the gap error is leaned over, and so the steepest
                                          # approach angle this rule will ever ask for (atan(1) = 45° here)
            "turn_gain": 1.6,             # 1/s of turn per radian of aim error
            "turn_limit": 1.1,            # rad/s — what the demo asks of wheels that could do about 2.6
            "search_turn": 0.7,           # rad/s on the spot while the right hand sees nothing at all
            "search_timeout": 20.0,       # s of seeing nothing before it says so and stops. This rule has no
                                          # map, so "drive somewhere else and look" is a different program
            "free_ahead": 0.5,            # m, below this the space ahead counts as blocked
            "laser_offset_y": 0.0,        # m the sensor sits LEFT of the kinematic centre. The simulator
                                          # mounts the lidar at [0,0,0] (`robot_frames`/`mount`,
                                          # mecanum_lab/types.py), so 0.0 is what this hall needs; the gap is
                                          # still measured from the centre, not from wherever the sensor is
            "period": 0.05,
            "view": True,                 # the overlay in RViz; `view:=false` stops publishing it
            "view_topic": "wall_view",    # → /wall_view, the topic the RViz config lists
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.want = float(self.get_parameter("wall_distance").value)
        self.period = float(self.get_parameter("period").value)
        self.laser_offset_y = float(self.get_parameter("laser_offset_y").value)
        self.settings = dict(speed=float(self.get_parameter("speed").value),
                             find_speed=float(self.get_parameter("find_speed").value),
                             max_wall_range=float(self.get_parameter("max_wall_range").value),
                             follow_tolerance=float(self.get_parameter("follow_tolerance").value),
                             aim_lead=float(self.get_parameter("aim_lead").value),
                             turn_gain=float(self.get_parameter("turn_gain").value),
                             turn_limit=float(self.get_parameter("turn_limit").value),
                             search_turn=float(self.get_parameter("search_turn").value),
                             search_timeout=float(self.get_parameter("search_timeout").value),
                             free_ahead=float(self.get_parameter("free_ahead").value))
        self.scan = None
        self.range_max = 8.0
        self.increment = 2 * pi / 360
        self.state = ()                                   # (phase, at_gap) of what was last printed
        self.looking = 0.0                                # seconds without a reference; every one resets it
        self.scan_frame = f"{self.robot}/base_link"       # whatever the scan says it is, once one arrives

        self.view_pub = None
        if bool(self.get_parameter("view").value) and view.available():
            self.view_pub = self.create_publisher(MarkerArray, str(self.get_parameter("view_topic").value), 10)
        self.wheels = self.create_publisher(Twist, f"/{self.robot}/cmd_vel", 10)
        self.create_subscription(LaserScan, f"/{self.robot}/scan", self.on_scan, 10)
        self.create_timer(self.period, self.on_timer)
        self.get_logger().info(
            f"keeping the wall {self.want:.2f} m off the right of the centre on /{self.robot}/scan; a wall "
            f"beyond {self.settings['max_wall_range']:.1f} m is only a direction to drive towards")

    def draw(self, decided: Steering):
        """What this rule reads, what it measured, and the heading it leaned to — the argument, on screen.

        `steer` looks in exactly four bearings — `AHEAD`, `RIGHT`, `RIGHT_AHEAD`, `RIGHT_BEHIND` — and a
        student who does not know that spends the demo guessing why the robot reacts to a shelf it never
        seems to look at. Each yellow line is as long as the echo that bearing returned, in the frame the
        scan arrived in. The blue line is `wall_distance` straight right from the centre and the green one
        the gap that was measured at the centre, so those two side by side *are* the error term, in metres,
        without anybody having to read a number. The short orange arrow is the heading the rule leaned to
        (`decided.aim`) and the red one the speed it asked for: those two are what a lecture should argue
        about, which is why the arithmetic is also written over the robot in words.

        The frame comes out of the message rather than from a name written here: the simulator's lidar is a
        child of `base_link` (`robot_frames` in `mecanum_lab/types.py`), and a marker in a frame nobody
        publishes is a marker that silently never appears — the least debuggable class of RViz bug there is.
        """
        if self.view_pub is None or self.scan is None:
            return
        stamp, frame = self.get_clock().now().to_msg(), self.scan_frame
        seen = [(index * self.increment, float(r)) for index, r in enumerate(self.scan.ranges)
                if echoed(r, self.range_max)]
        markers = [view.dots(frame, stamp, "echoes", [(r * cos(a), r * sin(a)) for a, r in seen],
                             0.03, view.CYAN)]
        for name, bearing in (("reads_ahead", AHEAD), ("reads_right", RIGHT),
                              ("reads_right_ahead", RIGHT_AHEAD), ("reads_right_behind", RIGHT_BEHIND)):
            reach = beam(self.scan.ranges, self.increment, bearing, 2, self.range_max)
            if reach is not None:
                markers.append(view.lines(frame, stamp, name,
                                          [((0.0, 0.0), (reach * cos(bearing), reach * sin(bearing)))],
                                          0.015, view.WHITE if bearing == AHEAD else view.YELLOW))
        markers.append(view.lines(frame, stamp, "wanted",
                                  [((0.0, 0.0), (self.want * cos(RIGHT), self.want * sin(RIGHT)))],
                                  0.03, view.BLUE))
        if decided.wall is not None:
            markers.append(view.lines(frame, stamp, "measured",
                                      [((0.0, 0.0), (decided.wall * cos(RIGHT), decided.wall * sin(RIGHT)))],
                                      0.03, view.GREEN))
        markers.append(view.arrows(frame, stamp, "aim",
                                   [((0.0, 0.0), (1.2 * cos(decided.aim), 1.2 * sin(decided.aim)))],
                                   0.03, view.ORANGE))
        markers.append(view.arrows(frame, stamp, "command", [((0.0, 0.0), (2.0 * decided.speed, 0.0))],
                                   0.05, view.RED))
        markers += view.labels(frame, stamp, "numbers", [((0.0, 0.0), state_line(decided, self.want))])
        view.publish(self.view_pub, markers)

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.range_max = float(msg.range_max)
        self.increment = float(msg.angle_increment)
        self.scan_frame = msg.header.frame_id or self.scan_frame

    def on_timer(self):
        if self.scan is None:
            return
        decided = steer(list(self.scan.ranges), self.increment, self.range_max, self.want,
                        laser_offset_y=self.laser_offset_y, looking=self.looking, dt=self.period,
                        **self.settings)
        self.looking = decided.looking                    # the clock the give-up phase runs on
        command = Twist()
        command.linear.x, command.angular.z = decided.speed, decided.turn
        self.wheels.publish(command)
        self.draw(decided)
        phase = (decided.state, decided.at_gap)           # one line per phase, not twenty per second
        if phase != self.state:
            self.get_logger().info(state_line(decided, self.want))
            self.state = phase


def state_line(decided: Steering, want: float = 0.50) -> str:
    """What the last cycle decided, in the words a lecture wants beside the window.

    The three phases and the two answers that are not phases: `looking for a wall` when the right hand sees
    nothing, `closing on it` when it sees a wall too far off to be a reference, then `converging on the gap`
    and `following` as that gap is met, with `blocked ahead` over any of them and `no wall found` when
    looking stopped being worth it.
    """
    if decided.state == NO_WALL:
        return (f"{NO_WALL} after {decided.looking:.0f} s: nothing at all reflected on the right. This rule "
                "has no map, so 'drive somewhere else and look again' is another program — try world:=maze, "
                "or raise max_wall_range if the hall really does have a wall out there")
    if decided.state == SEARCHING:
        return (f"{SEARCHING} — turning right on the spot, nothing reflected after {decided.looking:.0f} s "
                f"of {decided.speed:.2f} m/s of patience")
    aim = f"{degrees(decided.aim):+.0f}°"
    if decided.state == APPROACHING:
        return (f"{APPROACHING}: {decided.wall:.2f} m right of the centre, leaning {aim}, driving "
                f"{decided.speed:.2f} m/s")
    if decided.state == BLOCKED:
        return f"{BLOCKED} — gap still {decided.wall:.2f} m to the right, turning where it aims"
    if not decided.at_gap:
        return (f"converging on the {want:.2f} m gap — it is {decided.wall:.2f} m, leaning {aim}, driving "
                f"{decided.speed:.2f} m/s")
    return f"{FOLLOWING} · gap {decided.wall:.2f} m (want {want:.2f}) · aim {aim} · {decided.speed:.2f} m/s"


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
