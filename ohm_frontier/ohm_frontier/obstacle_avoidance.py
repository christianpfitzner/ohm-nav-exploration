"""Drive in one direction and get around whatever the lidar sees — the whole scan read as a clearance map.

    ros2 launch ohm_frontier reactive_avoid.launch.py
    ros2 launch ohm_frontier reactive_avoid.launch.py world:=rooms

(to be finished once the hall numbers are in — the rule's own docstrings below are complete.)
"""
from collections import namedtuple
from math import cos, hypot, inf, pi, radians, sin, sqrt

try:                                    # the rule below is importable without ROS, and is tested that way
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import MarkerArray
except ImportError:                     # so `test_reactive.py` can read the rule alone; `main` says so
    rclpy = Twist = LaserScan = MarkerArray = None
    Node = object

from . import view_markers as view      # the overlay; `view.available()` answers for the import above

from .angles import wrap            # the one place; see `angles.py` on why not three

#: the three answers the rule can give, in the order they get worse
CLEAR = "clear way"
TURNING = "steering around an obstacle"
BLOCKED = "blocked, turning to the widest gap"
#: and the one that is not an answer but a resignation, with a number attached
NOWHERE = "nowhere to go — standing still"

#: One candidate heading per `CANDIDATE_BEAMS` beams. The turn limit is 1.1 rad/s and a cycle is 0.05 s, so
#: one cycle moves the nose 0.055 rad = 3.2°: a grid finer than that is a question the wheels cannot answer
#: before it is asked again, and 5 beams (5°) is the coarsest grid that cannot hide a gap this robot does
#: not fit through. 72 candidates instead of 360, which is what keeps the whole scan affordable at 20 Hz.
CANDIDATE_BEAMS = 5

#: What one candidate heading is. Two numbers of metres and a bearing, because the scan answers two different
#: questions and only one of them is about the next half second:
#:
#:   `runway` — the metres the robot may travel that way before one of its echoes comes inside the clearance
#:     ring. Read from the beams inside `planning_reach`, which is the gap, and it is what the speed is taken
#:     from.
#:   `floor` — the metres of floor the lidar can see down that ray, all 8 m of it, `inf` where nothing
#:     reflects. That is the far half of the scan speaking as a direction rather than as a gap to regulate,
#:     and it is what decides whether a heading is worth driving at all.
Way = namedtuple("Way", "angle runway floor")

#: The rule's answer, all of it in the robot's own frame: the two velocity components it drives (x forward,
#: y left — y is zero unless `strafe` was asked for), the turn, the heading it chose, which of the four
#: states this is, the sentence that says which way it decided and why, the heading it is committed to for
#: the next cycle, the metres of travel the direction it drives has, and the seconds spent with nowhere to
#: go. `held` and `spent` go back in as arguments: this rule has no map, but it must not re-choose which
#: way round an obstacle it is passing every 50 ms either, and those two numbers are the whole of its memory.
Decision = namedtuple("Decision", "forward sideways turn direction state why held runway spent")


def echoed(r, range_max=None) -> bool:
    """Did this beam come back? All three dialects of "no" give the same answer — see `wall_following.py`.

    The simulator writes a beam that came back nowhere as its own `range_max` unless it was started with
    `lidar_no_echo:=inf`, and `nan` shows up in recordings. None of the three is a distance, and reading 8.0
    m as "a wall at 8 m" is how a reactive demo ends up steering away from the middle of an empty hall.
    """
    return r == r and r != float("inf") and r > 0.0 and (range_max is None or float(r) < range_max)


def clamp(value: float, low: float, high: float) -> float:
    """Inside the range, unchanged; outside it, the nearest edge."""
    return max(low, min(high, value))


def echoes_in_reach(ranges, angle_increment, range_max, reach):
    """The echoes this rule can act on, as points from the centre, in the robot's own frame.

    `reach` is not a dial: see `planning_reach`. An echo outside it is a fact about the room and cannot
    change this cycle's answer, which is the first principle of this file — an echo that far away is a
    direction, not a gap to regulate — arriving as arithmetic instead of as a tuned range.
    """
    points = []
    for index, r in enumerate(ranges):
        distance = float(r)
        if not echoed(r, range_max) or distance > reach:
            continue
        angle = index * angle_increment         # index 0 is the nose, the index grows counterclockwise
        points.append((distance * cos(angle), distance * sin(angle)))
    return points


def planning_reach(clearance, brake_lead) -> float:
    """The one range a beam has to be inside to matter at all, derived rather than chosen.

    A beam at (`x`, `y`) can only hold the robot back in a direction if that direction drives *towards* it:
    the closest the centre ever comes to the beam is its perpendicular distance from the ray, so a beam that
    is to be worth anything has to be inside `clearance` of the ray at worst, and the ray has to reach it.
    The tightest case is a beam exactly on the ray, which is `brake_lead + clearance` away, and the widest
    case is a beam at the edge of the ring beside the robot, `clearance` away sideways. The hypotenuse of
    those two is the radius beyond which no beam can either brake the drive or close a direction, and with
    the shipped defaults it is 1.26 m of the lidar's 8 m: three quarters of the scan is read as empty space
    and the rule is cheaper for knowing it.
    """
    return hypot(brake_lead + clearance, clearance)


def runway(points, clearance, angle) -> float:
    """How far the centre may travel along `angle` before one of `points` comes inside `clearance` of it.

    The robot is a point for this rule — the `clearance` ring around it is the footprint. Driving `s` metres
    along the unit vector `u` puts the centre at `s·u` and the beam at distance² = (`along` − s)² + `across`²,
    where `across` is the beam's perpendicular distance from the ray. Solving that for the distance equalling
    `clearance` gives the travel at which the ring first touches the beam:

        room = along − sqrt(clearance² − across²)

    Three cases answer "nothing to say here" and they are the whole content of this function:

    * `across >= clearance`: this beam is outside the ring whatever the heading, so driving this way can
      never bring it in. A wall 2 m to the flank is not a reason to stop, and — this is the half that the
      version this file replaces got wrong — it is not nothing either: it is a fact about the gap, and the
      gap it reports is what `way_round` reads to decide which headings are on the menu at all.
    * `along <= 0.0`: the beam is beside or behind the centre, and driving away from it only ever increases
      its distance. It is the corner the robot is standing in, not the wall in front of it.
    * otherwise `room` may still be negative, which means the beam is *already* inside the ring: the honest
      answer to that is no speed and a turn, not a smaller speed.
    """
    ux, uy = cos(angle), sin(angle)
    room = inf
    for x, y in points:
        along, across = x * ux + y * uy, abs(x * uy - y * ux)
        if across >= clearance or along <= 0.0:
            continue
        room = min(room, along - sqrt(clearance * clearance - across * across))
    return room


def floor_down(ranges, angle_increment, angle, range_max, window=3):
    """How much floor the lidar sees straight down this ray: the near map's opposite number.

    The scan *is* this measurement — the beam at this bearing already left the sensor along this ray and came
    back with the distance to the first thing in it — so the answer is the nearest echo within `window` beams
    of the bearing and nothing more, which is why reading the far half of the scan costs nothing. The window
    is there because one beam of a 360-beam scan is 1° wide and 1.5 cm of lidar noise on a wall at 40 cm is
    several degrees: without it the edge of a shelf answers "8 m of open floor" and the rule drives at the
    shelf. No echo is `inf`, which is the most open answer a ray can give.
    """
    count = len(ranges)
    if count == 0:
        return None
    middle = round(angle / angle_increment) % count
    seen = [ranges[(middle + offset) % count] for offset in range(-abs(window), abs(window) + 1)]
    back = [float(r) for r in seen if echoed(r, range_max)]
    return min(back) if back else inf


def ways(ranges, angle_increment, range_max, clearance, brake_lead):
    """The scan as a polar map: one `Way` per candidate heading, every 5° of the 360° the lidar returns.

    This is the whole-scan half of the rewrite. The version before this one voted on the beams inside
    ±0.35 rad of the nose, summed the votes into a vector, and drove that vector — which produced a sideways
    wish out of a flank wall, because the flank beams were outside the window and the ones inside it were
    the ones pointing away from the wall. Here every beam of the scan is asked both questions: how much run
    it leaves (`runway`, from the beams near enough to be reacted to at all) and how much floor lies behind
    it (`floor`, from every beam the sensor returns).
    """
    points = echoes_in_reach(ranges, angle_increment, range_max, planning_reach(clearance, brake_lead))
    # `index` is a beam number and the bearing of a beam is `index × angle_increment`; the step between the
    # candidates is already folded into the range below. Multiplying by that step instead — which is what this
    # line did first, and its own docstring said 5° while it did it — asks for one candidate every 25°, and a
    # rule with 14 headings around the robot instead of 72 calls a 30° gap a wall.
    return [Way(wrap(index * angle_increment), runway(points, clearance, index * angle_increment),
                floor_down(ranges, angle_increment, index * angle_increment, range_max))
            for index in range(0, len(ranges), CANDIDATE_BEAMS)]


def menu(available, free_travel, open_floor):
    """Which candidate headings are worth driving, as one flag per heading.

    A heading is on the menu when it offers `free_travel` of run and shows `open_floor` of floor down itself.
    Both halves are needed: the run is the near map, the metres the robot can actually travel before an echo
    enters its ring; the floor is the far map, which is how a heading that leads into a dead end is told apart
    from one that leads down a corridor. Neither one alone works — measured with the run test only, the same
    rule in `rooms` follows the wall it is braking for instead of the door it could have gone through.
    """
    return [w.runway >= free_travel and w.floor >= open_floor for w in available]


def opening(available, cone) -> list:
    """How wide each candidate heading's opening is: the mean floor over the headings within ±`cone` of it.

    One ray answers "how far is the first wall down *this* line", which is the wrong thing to break a tie
    with. Two headings 10° either side of the nose can have the same runway and the same floor down
    themselves — one leads to a corridor, the other to a wall 40° further out — and the version of this rule
    that ranked on the single floor fell back on the order the beams arrive in, which is how a robot ends up
    taking the left round every obstacle in a hall: the positive bearings come first in the list. Averaging
    over a cone is also what the wheels ask for, because a heading is driven over the next half second and
    not sampled at an instant.

    Far echoes are capped at the lidar's own 8 m before averaging: `inf` means "as open as this sensor can
    tell", and three `inf`s out of seventy-two headings must not outvote a measured corridor.
    """
    count = len(available)
    if count == 0:
        return []
    # `count` is a count of *candidates*, so 2π/count is already the 5° between them: dividing the cone by
    # that gives the number of candidates either side to average over (±20° is 4). Multiplying by
    # `CANDIDATE_BEAMS` again — which is what this line did first — asks for 25° of cone per unit and settles
    # on a cone so wide it averages the whole hall, which is the same as not ranking by width at all.
    reach = max(1, round(cone / (2 * pi / count)))
    # modulo, not a slice: the headings either side of due nose are the last entries of the list and the
    # first, and `available[i - reach:i + reach + 1]` reads off the end of it and averages nothing for
    # exactly the headings this rule is deciding between.
    return [sum(min(available[(i + o) % count].floor, 8.0) for o in range(-reach, reach + 1)) / (2 * reach + 1)
            for i in range(count)]


def way_round(available, on_menu, wide, aim, held, free_travel, hold_floor):
    """Which heading to drive: the least turn that is on the menu, and the same side it chose last time.

    Two metres thresholds and a cone decide what is on the menu (`menu`); of what is on it this takes the
    heading nearest `aim`; and the side of `aim` committed to on an earlier cycle is kept while the wanted
    heading is off the menu. That last clause is the difference between this rule and the one it replaces:

        measured on 45 s in `rooms` from the spawn, the old field: 13.97 m of path, 0.03 m of net, 100 % of
        samples with a sideways command, `wz` pinned at ±1.1 rad/s flipping sign every cycle. It re-decided
        which way to go from a fresh scan every 50 ms, and two walls of comparable weight answer that question
        two different ways on two neighbouring cycles.

    While a side stands, the candidates are the menu headings on that side and that side only — leaving the
    wanted heading in the list is how the robot ends up going back into the thing it was turning round. Where
    nothing is committed yet the turn is the one nearest `aim`, and where the two sides offer the same turn
    the one with more floor wins: a tie broken by the scan's own index order is a tie that flips when the
    lidar jitters by its own 1.5 cm. A hall in which no heading shows `open_floor` is still a hall the robot
    has to drive out of, so there the run test rules alone; and None, when no heading even has `free_travel`,
    is the caller's full-stop-with-a-reason case.
    """
    if not available:
        return None
    wish_index = min(range(len(available)), key=lambda i: abs(wrap(available[i].angle - aim)))
    wish = available[wish_index]
    if wish.runway >= free_travel and wish.floor >= hold_floor:
        return wish                                     # the want is open far enough ahead: drive it
    side = 0.0 if not held else (1.0 if held > 0.0 else -1.0)
    open_ways = [i for i, ok in enumerate(on_menu) if ok]
    on_side = [i for i in open_ways if side == 0.0 or side * wrap(available[i].angle - aim) > 0.0]
    near = [i for i, w in enumerate(available) if w.runway >= free_travel]
    candidates = [i for i in (on_side or open_ways or near) if available[i].runway >= free_travel]
    if not candidates:
        return None
    # Widest opening first, then the heading with more travel in the near map, then the least turn: the far
    # map chooses where to go, the near map breaks the ties the far map cannot see, and only then does the
    # robot ask which way costs the least turning. The
    # order of the beams is not a fact about the hall and a tie broken by it is a rule with a favourite hand;
    # so is a tie broken by floating point. Measured on the mirrored pair in `test_reactive.py` — a dead end
    # dead ahead, a wall on one shoulder, and the same wall on the other — the two headings 15° either side of
    # the nose came out with the same opening and the same turn, and the difference that decided them was the
    # ninth decimal place of `abs(wrap(angle - aim))`: 0.26179938779914913 for the left and ...945 for the
    # right, which is how the same rule took the same side of the obstacle in both halls. The runway of the
    # two is 0.53 m against 0.49 m, which is a fact about the hall and flips when the hall flips.
    return available[min(candidates, key=lambda i: (-wide[i], -min(available[i].runway, 8.0),
                                                     abs(wrap(available[i].angle - aim))))]


def room_to_drive(available, target, strafe) -> float:
    """The metres the *driven* direction has, which is what the speed is taken from.

    With the wheels pointed at the heading it slides, so the heading it slides along is the only one that
    can be short. Without strafe — which is the default, and the car-like answer — the velocity goes out of
    the nose while the turn sweeps that nose round to the chosen heading, and the arc in between is driven
    as well: a 1.1 rad/s turn at 0.35 m/s is a 0.32 m radius, so the wall the nose is passing on its way to
    the gap is not behind the robot yet. The tightest of the arc is the brake.
    """
    if strafe:
        return target.runway
    low, high = sorted((0.0, wrap(target.angle)))
    return min([w.runway for w in available if low <= wrap(w.angle) <= high] + [target.runway])


def avoid(ranges, angle_increment, range_max, aim=0.0, clearance=0.40, brake_lead=0.80,
          free_travel=0.25, open_floor=3.00, hold_floor=4.00, open_cone=radians(20), speed=0.35,
          turn_gain=1.6, turn_limit=1.1, strafe=False, stuck_timeout=20.0, held=0.0, spent=0.0, dt=0.05):
    """One scan, one wanted heading, out come the metres per second and the radians per second.

    The rule, in the five sentences it is worth memorising:

    1. Every heading 5° of the scan gets two numbers of metres: how far the robot may travel that way before
       an echo enters the `clearance` ring (`runway`, the near map), and how much floor the lidar sees down
       that ray (`floor`, the far map). Every beam of the scan is asked, not the ones near the nose.
    2. A heading is worth driving when it offers `free_travel` of run and shows `open_floor` of floor.
    3. Of those, the one nearest the wanted heading is driven; while the wanted one is not on the menu, the
       side committed to last cycle is kept, and only re-chosen when that side closes.
    4. The speed is `speed` scaled by the run of the direction actually driven, reaching nothing at 0 m, so
       `clearance` is what stops the robot and not a threshold distance in front of the nose.
    5. `wz` goes to the chosen heading always, including when the speed is 0: a stopped robot is turning its
       nose to the widest gap and saying which gap it picked, never standing still because it had no idea.

    `held` and `spent` are the rule's memory and they are arguments rather than globals for the same reason
    `steer` takes `looking`: a pure function is a testable one, and two numbers are the whole of what this
    rule needs to remember between cycles.
    """
    available = ways(ranges, angle_increment, range_max, clearance, brake_lead)
    wide = opening(available, open_cone)
    if not available:                                   # a scan with no beams in it at all
        return Decision(0.0, 0.0, 0.0, aim, NOWHERE, "the scan is empty: no beam to decide from",
                        0.0, 0.0, spent + dt)

    chosen = way_round(available, menu(available, free_travel, open_floor), wide, aim, held, free_travel,
                       max(open_floor, hold_floor))
    if chosen is None:                                  # nowhere has free_travel of runway: turn, don't push
        widest = max(available, key=lambda w: (wide[available.index(w)], w.runway,
                                               -abs(wrap(w.angle - (held if held else aim)))))
        spent += dt
        turn = clamp(turn_gain * wrap(widest.angle), -turn_limit, turn_limit)
        if spent >= stuck_timeout:
            return Decision(0.0, 0.0, 0.0, widest.angle, NOWHERE,
                            f"nothing in this scan offers {free_travel:.2f} m of travel and it has had "
                            f"{spent:.0f} s turning to find some: this rule has no map, so 'back out and try "
                            "the other side of the hall' is another program", held, 0.0, spent)
        return Decision(0.0, 0.0, turn, widest.angle, BLOCKED,
                        f"no heading has {free_travel:.2f} m of travel: the widest gap is "
                        f"{widest.angle:+.2f} rad, where the scan sees {_cm(widest.floor)} — turning there, "
                        "wheels still", held, widest.runway, spent)

    room = room_to_drive(available, chosen, strafe)
    brake = clamp(room / brake_lead, 0.0, 1.0)
    drive = speed * brake
    # The turn goes to the heading that was chosen *from the nose*, not to the want: a car-like base can only
    # ever drive where its nose points, so the nose is what has to be brought round to the chosen heading.
    # The 0.12 rad band around it exists so that a robot already going straight is not made to wiggle at the
    # ±5° resolution of the candidate grid, and it is applied only while there is room to be wiggle-free in:
    # a cycle whose drive is braked to nothing is a cycle that has to turn whether or not the heading it wants
    # is only a few degrees off, because a few degrees is what takes it out of the wall it is standing in.
    off = wrap(chosen.angle)
    if room >= free_travel and abs(off) < 0.12:
        off = 0.0
    turn = clamp(turn_gain * off, -turn_limit, turn_limit)
    # The same decision, projected onto what the wheels can actually do. A mecanum base drives the chosen
    # heading directly; a car-like one takes the forward component of it and spends the rest on the turn —
    # which is why a car at 90° off its own gap stands still and pivots instead of driving into it sideways.
    if strafe:
        forward, sideways = drive * cos(chosen.angle), drive * sin(chosen.angle)
    else:
        forward, sideways = drive * max(0.0, cos(off)), 0.0
    where = f"{wrap(chosen.angle):+.2f} rad ({'left' if chosen.angle > 0 else 'right'})"
    if turn == 0.0:
        return Decision(forward, sideways, 0.0, chosen.angle, CLEAR, "", 0.0, room, 0.0)
    if forward < 0.02:
        return Decision(forward, sideways, turn, chosen.angle, BLOCKED,
                        f"{_cm(room)} of travel in the direction it drives, so the wheels stay still while "
                        f"the nose turns {where}, where the scan has {chosen.runway:.2f} m",
                        chosen.angle, room, 0.0)
    return Decision(forward, sideways, turn, chosen.angle, TURNING,
                    f"{_cm(room)} ahead of the nose, {where} has {chosen.runway:.2f} m — turning "
                    f"{'left' if chosen.angle > 0 else 'right'} at {forward:.2f} m/s",
                    chosen.angle, room, 0.0)


def _cm(metres: float) -> str:
    """Metres as this file speaks them, because every one of them is a wall it did not touch."""
    return "more than 1.3 m" if metres == inf else f"{metres:.2f} m"


class ObstacleAvoidance(Node):
    """The rule on a timer, on the simulator's wheels.

    Commands go out at 20 Hz even though the scan arrives at 20 Hz too, because the simulator forgets wheel
    commands after `cmd_timeout` = 0.35 s (`mecanum_lab/types.py`). A node that only ever answers a scan
    stops driving the moment the scans stop arriving, which happens to be the right behaviour here and for
    entirely the wrong reason — so the timer says what it is for.
    """

    def __init__(self):
        super().__init__("obstacle_avoidance")
        for name, value in {
            "robot": "muster",
            "drive_heading": 0.0,         # deg, the direction the rule wants; 0 = the nose. On a car-like
                                          # base this tilts the want inside the scan, it is not a bearing:
                                          # the node has no compass, so 90° means "90° left of wherever I end
                                          # up facing", which is a circle and not a route
            "clearance": 0.40,            # m: the ring around the centre no echo may enter. The robot's own
                                          # collision circle is 0.21 m (`physics.Geometry.footprint_r`), the
                                          # lidar's own noise is 1.5 cm (`lidar.sigma`), and the brake can only
                                          # stop the cycle *after* the one that saw the wall, so 0.30 m of
                                          # margin is what the measured nearest echo cost: at 0.30 m of
                                          # clearance the closest beam of a 60 s run in `rooms` came in under
                                          # the 0.30 m the measurement calls a near miss, at 0.40 m it stays
                                          # outside it and the robot still fits every passage of both halls
            "brake_lead": 0.80,           # m of travel at which full `speed` is back, and nothing below 0 m.
                                          # The line the hysteresis hangs on as well: a wanted heading counts as
                                          # open again — the commitment released — only at this much runway, so
                                          # the robot does not decide to go straight again at the very moment it
                                          # is deciding not to
            "free_travel": 0.25,          # m of travel that puts a heading on the menu at all: less than this
                                          # and the rule would drive a heading that arrives at the ring before
                                          # the turn across it has finished
            "open_floor": 3.00,           # m of floor the lidar must see down a ray for that heading to be
                                          # driven at all. This is the parameter that reads the far half of the
                                          # scan, and it reads it as a direction: a wall 4 m away cannot be
                                          # regulated as a gap and is perfectly good at ruling out a heading.
                                          # Measured in `rooms` without it — the same rule, every heading on
                                          # the menu: **path 11.15 m, net 3.68 m, path/net 3.03**, the robot
                                          # circling one 6.5 m room for the whole minute — and with it at
                                          # 3.00 m: see the numbers at the top of this file. The door of a hall
                                          # is 1.5 m wide and the corridor behind it is open, so it is the one
                                          # heading in a room this wide that shows 3 m of floor.
            "speed": 0.35,                # m/s — the wheels stop at 0.6 m/s (12 rad/s at r = 0.05 m)
            "turn_gain": 1.6,             # 1/s of turn per radian of heading error
            "turn_limit": 1.1,            # rad/s — what the demo asks of wheels that could do about 2.2
            "strafe": False,              # drive the chosen heading sideways instead of turning to it. Off:
                                          # a robot that travels sideways while facing elsewhere reads to an
                                          # audience as a robot that misbehaves, even when the mecanum wheels
                                          # make it the geometrically shortest answer
            "stuck_timeout": 20.0,        # s of turning with no heading offering `free_travel` before it says
                                          # so and stops both. Turning is the answer to a blocked robot and
                                          # this is the honest end of it: the rule has no map, so "back out and
                                          # try the other side of the hall" is a different program
            "period": 0.05,
            "view": True,                 # the overlay in RViz; `view:=false` stops publishing it
            "view_topic": "field_view",   # → /field_view, the topic the RViz config lists
        }.items():
            self.declare_parameter(name, value)

        robot = str(self.get_parameter("robot").value)
        self.aim = float(self.get_parameter("drive_heading").value) * pi / 180.0
        self.clearance = float(self.get_parameter("clearance").value)
        self.settings = dict(clearance=self.clearance,
                             brake_lead=float(self.get_parameter("brake_lead").value),
                             free_travel=float(self.get_parameter("free_travel").value),
                             open_floor=float(self.get_parameter("open_floor").value),
                             speed=float(self.get_parameter("speed").value),
                             turn_gain=float(self.get_parameter("turn_gain").value),
                             turn_limit=float(self.get_parameter("turn_limit").value),
                             strafe=bool(self.get_parameter("strafe").value),
                             stuck_timeout=float(self.get_parameter("stuck_timeout").value))
        self.period = float(self.get_parameter("period").value)
        self.scan = None
        self.range_max = 8.0
        self.increment = 2 * pi / 360
        self.state = ()                                   # (phase, side committed to) of what was printed
        self.held = 0.0                                   # the heading committed to last cycle, rad off nose
        self.spent = 0.0                                  # seconds with no drivable heading; else 0.0
        self.scan_frame = f"{robot}/base_link"            # whatever the scan says, once one arrives

        self.view_pub = None
        if bool(self.get_parameter("view").value) and view.available():
            self.view_pub = self.create_publisher(MarkerArray, str(self.get_parameter("view_topic").value), 10)
        self.wheels = self.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
        self.create_subscription(LaserScan, f"/{robot}/scan", self.on_scan, 10)
        self.create_timer(self.period, self.on_timer)
        self.get_logger().info(
            f"driving wherever the scan has room, keeping every echo {self.clearance:.2f} m off the centre "
            f"on /{robot}/scan; nothing nearer than "
            f"{planning_reach(self.clearance, self.settings['brake_lead']):.2f} m can be reacted to at all")

    def draw(self, decided: Decision):
        """What the rule read, what it decided, and what went to the wheels — the argument, on screen.

        The yellow spokes are the beams it acts on, drawn to their own range: everything outside
        `planning_reach` is deliberately absent, because that absence is the first principle of the file and
        a lecture should be able to see it. The green circle is `clearance`, the ring no echo may enter, drawn
        round the centre rather than round the sensor because the two are the same point only by accident of
        the simulator's lidar mount. Each grey spoke is one candidate heading and as long as the travel that
        heading has, capped at `brake_lead`; the red ones are the candidates under `free_travel`, which is
        the menu with the refused items still marked on it. The orange arrow is the heading the rule picked
        and the blue one the velocity that goes out, drawn in the robot's own frame — and a blue arrow that
        is not along the nose is a mecanum robot driving sideways, which is what `strafe:=true` is there to
        demonstrate and the default is there to avoid.
        """
        if self.view_pub is None or self.scan is None:
            return
        stamp, frame = self.get_clock().now().to_msg(), self.scan_frame
        near = planning_reach(self.clearance, self.settings["brake_lead"])
        seen = [(index * self.increment, float(r)) for index, r in enumerate(self.scan.ranges)
                if echoed(r, self.range_max) and float(r) <= near]
        markers = [view.lines(frame, stamp, "beams",
                              [((0.0, 0.0), (r * cos(a), r * sin(a))) for a, r in seen], 0.008, view.YELLOW),
                   view.ring(frame, stamp, "clearance", (0.0, 0.0), self.clearance, view.GREEN)]
        available = ways(list(self.scan.ranges), self.increment, self.range_max,
                         self.clearance, self.settings["brake_lead"])
        lead = self.settings["brake_lead"]
        markers.append(view.lines(frame, stamp, "travel",
                                  [((0.0, 0.0), (min(w.runway, lead) * cos(w.angle),
                                                 min(w.runway, lead) * sin(w.angle))) for w in available],
                                  0.006, view.WHITE))
        closed = [w for w in available if w.runway < self.settings["free_travel"]
                  or w.floor < self.settings["open_floor"]]
        markers.append(view.lines(frame, stamp, "closed",
                                  [((0.9 * lead * cos(w.angle), 0.9 * lead * sin(w.angle)),
                                    (lead * cos(w.angle), lead * sin(w.angle))) for w in closed],
                                  0.02, view.RED))
        markers.append(view.arrows(frame, stamp, "aim", [((0.0, 0.0), (cos(self.aim), sin(self.aim)))],
                                   0.03, view.GREEN))
        markers.append(view.arrows(frame, stamp, "gap",
                                   [((0.0, 0.0), (1.2 * cos(decided.direction), 1.2 * sin(decided.direction)))],
                                   0.04, view.ORANGE))
        markers.append(view.arrows(frame, stamp, "command",
                                   [((0.0, 0.0), (2.0 * decided.forward, 2.0 * decided.sideways))],
                                   0.05, view.BLUE))
        markers += view.labels(frame, stamp, "numbers", [((0.0, 0.0), state_line(decided))])
        view.publish(self.view_pub, markers)

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.range_max = float(msg.range_max)
        self.increment = float(msg.angle_increment)
        self.scan_frame = msg.header.frame_id or self.scan_frame

    def on_timer(self):
        if self.scan is None:
            return
        decided = avoid(list(self.scan.ranges), self.increment, self.range_max, self.aim,
                        held=self.held, spent=self.spent, dt=self.period, **self.settings)
        self.held, self.spent = decided.held, decided.spent
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = decided.forward, decided.sideways, \
            decided.turn
        self.wheels.publish(command)
        self.draw(decided)
        phase = (decided.state, decided.turn > 0.0, decided.turn < 0.0)     # one line per change of mind
        if phase != self.state:
            self.get_logger().info(state_line(decided))
            self.state = phase


def state_line(decided: Decision) -> str:
    """What the last cycle decided, in the words a lecture wants beside the window.

    Which way it turned and how much room it turned into is the sentence this file exists to produce; the
    version before it printed `steering around an obstacle` for 45 s while the robot stood on one spot, which
    is a state line that describes the intent rather than the motion. Every line therefore carries the metres
    the decision was taken on, and the two stopped states carry the reason as well.
    """
    if decided.state == NOWHERE:
        return f"{NOWHERE}: {decided.why}"
    if decided.state == BLOCKED:
        return f"{BLOCKED} — {decided.why}"
    if decided.state == TURNING:
        return (f"{TURNING} — {decided.why}, driving {decided.forward:.2f} m/s"
                + (f" and {decided.sideways:.2f} m/s sideways" if decided.sideways else ""))
    return f"{CLEAR} · {_cm(decided.runway)} of travel · {decided.forward:.2f} m/s"


def main(args=None):
    if rclpy is None:
        raise SystemExit("no rclpy here. The rule in this file imports without ROS — running the node does "
                         "not. Source a ROS 2 installation.")
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
