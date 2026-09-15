"""The three `draw()` methods, called for real, with the publisher faked.

Every other test in this package is about a number: what the field sums to, which clump wins, how long a search
takes. The overlay is the part of the package that has never had that, and it is the part a lecture looks at — and
it is where the bugs that no arithmetic test can see live. Two were found by writing this file:
`wall_following.py` called `cos` and `sin` with only `pi` imported, which is a `NameError` twenty times a second
inside the timer callback and nothing at all in a test run; and a `TEXT_VIEW_FACING` marker is placed by its
`pose`, not by the `points` that every other kind here uses, so every label in the package was being written at the
origin of the frame, hundreds of cells from the robot it describes, with a perfectly good message on the wire.

Neither shows up without calling `draw()` itself. So it is called, on a stub that is not a node: the three methods
touch `view_pub`, `pose` or `scan`, `scan_frame`, `settings`, `want`/`aim`, `state` and `get_clock()`, and a
`SimpleNamespace` carrying exactly those is enough to run the real code — which is the argument for keeping the
drawing in a method that receives a decision instead of in a callback that publishes one.

What is asserted is what RViz reads. That the namespaces are the ones the view lists a display for. That a text
marker has text, a pose where the robot is, a size in metres and a non-zero alpha — each of those four being a way
for a marker to be published, listed by name in the display panel, and never appear on screen. And that nothing is
stamped at the epoch, which `view.publish` drops on purpose.

What is *not* asserted, because no test can assert it: that RViz paints the result. The geometry has been seen
rendering, in screenshots of real runs; the text has been seen on the wire and not yet on this machine's screen,
and that distinction is in the README rather than papered over here.
"""
from types import SimpleNamespace

from builtin_interfaces.msg import Duration, Time
from sensor_msgs.msg import LaserScan

from ohm_frontier import move_to_point, obstacle_avoidance, view_markers, wall_following
from ohm_frontier.move_to_point import ARRIVED, DRIVING, NO_GOAL, Motion
from ohm_frontier.obstacle_avoidance import CLEAR, Decision
from ohm_frontier.wall_following import FOLLOWING, Steering

FRAME = "muster/odom"
TWO_PI = 2 * 3.141592653589793


class Recorder:
    """A publisher that remembers what it was handed, which is all a publisher does."""

    def __init__(self):
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


class Clock:
    """A clock that says 1 000 seconds, because a marker stamped at zero is dropped (`view_markers`)."""

    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=1000, nanosec=0))


def a_scan(distance=3.0, beams=360):
    scan = LaserScan()
    scan.header.frame_id = "muster/base_link/laser"
    scan.angle_increment = TWO_PI / beams
    scan.range_max = 8.0
    scan.ranges = [distance] * beams
    return scan


def drawn_by(method, node, decided):
    """One call, one message, its markers keyed by namespace — the shape each display in the view reads."""
    method(node, decided)
    assert node.view_pub.sent, "the draw method published nothing at all"
    return {marker.ns: marker for marker in node.view_pub.sent[-1].markers}


def the_point_controller():
    return SimpleNamespace(view_pub=Recorder(), robot="muster", pose=(2.0, 1.0, 0.3),
                           goal=(5.0, 4.0), scan_frame=FRAME,
                           settings=dict(aim_tolerance=0.08, speed=0.3, gain_turn=1.8, turn_limit=1.2,
                                         decelerate_over=0.25, arrive_distance=0.12),
                           get_clock=Clock)


def test_the_point_controller_draws_the_five_things_its_view_lists():
    node = the_point_controller()
    by_namespace = drawn_by(move_to_point.MoveToPoint.draw, node, Motion(DRIVING, 0.3, 0.0, 0.05, 4.24))

    assert {"aim", "cone", "goal", "command", "numbers"} <= set(by_namespace), \
        f"the view lists a display per namespace; the node drew {sorted(by_namespace)}"
    assert all(marker.header.frame_id == FRAME for marker in by_namespace.values()), \
        "everything this controller knows is in the odometry's frame, and every marker says so — a marker in a " \
        "frame nobody publishes does not fail, it just is not there"

    cone, goal = by_namespace["cone"], by_namespace["goal"]
    assert len(cone.points) == 4, "two lines at ±`aim_tolerance` off the nose: the parameter, drawn"
    assert len(goal.points) == 2 and goal.points[1].x == node.goal[0], "the line ends at the place it was sent to"

    label = by_namespace["numbers"]
    assert label.type == label.TEXT_VIEW_FACING
    assert label.text.startswith(DRIVING) and "rad" in label.text, f"the label said {label.text!r}"
    assert (label.pose.position.x, label.pose.position.y) == node.pose[:2], \
        "over the robot: RViz places a text marker by its `pose`, not by its `points`"
    assert label.scale.z > 0.05, "0.30 m of text is the size that survives the camera; 0 is no text at all"
    assert label.color.a > 0 and label.lifetime == Duration(sec=1), \
        "an invisible alpha and a sentence left hanging over a hall that moved on are the same nothing"

    assert all(marker.header.stamp.sec == 1000 for marker in by_namespace.values()), \
        "`publish` drops what is stamped at the epoch, so the stamp has to be a real time by the time it is drawn"


def test_the_point_controller_draws_nothing_before_the_odometry_arrives():
    """A marker drawn before the first pose is a marker at the origin of a hall nobody has driven in yet."""
    node = the_point_controller()
    node.pose, node.goal = None, None
    move_to_point.MoveToPoint.draw(node, Motion(NO_GOAL, 0.0, 0.0, 0.0, 0.0))
    assert not node.view_pub.sent


def test_the_wall_follower_draws_the_four_bearings_it_reads_and_the_gap_it_keeps():
    node = SimpleNamespace(view_pub=Recorder(), scan=a_scan(), scan_frame="muster/base_link/laser", want=0.40,
                           state=FOLLOWING, range_max=8.0, increment=TWO_PI / 360, get_clock=Clock)
    by_namespace = drawn_by(wall_following.WallFollower.draw, node, Steering(0.35, -0.2, FOLLOWING, 0.42))

    assert {"echoes", "reads_ahead", "reads_right", "reads_right_ahead", "reads_right_behind", "wanted",
            "command", "numbers"} <= set(by_namespace), f"it drew {sorted(by_namespace)}"
    echoes = by_namespace["echoes"]
    assert echoes.type == echoes.SPHERE_LIST and len(echoes.points) == 360, \
        "one dot per beam that came back, which is exactly the claim `echoed` makes"
    assert by_namespace["wanted"].points[1].y < 0, "the gap it keeps is on the right, so it is drawn on the right"
    assert by_namespace["numbers"].text.startswith(FOLLOWING), "the sentence over the robot is the log's own"


def test_the_field_draws_the_votes_the_wish_the_sum_and_the_wheels():
    node = SimpleNamespace(view_pub=Recorder(), scan=a_scan(1.2), scan_frame="muster/base_link/laser", aim=0.0,
                           settings=dict(repulsion_range=2.0, speed=0.35, turn_limit=1.1, turn_gain=1.6,
                                         stop_gap=0.55, half_view=0.35),
                           range_max=8.0, increment=TWO_PI / 360, get_clock=Clock)
    by_namespace = drawn_by(obstacle_avoidance.ObstacleAvoidance.draw, node,
                            Decision(0.3, 0.1, 0.4, 0.3, CLEAR, ""))

    assert {"votes", "aim", "field", "command", "numbers"} <= set(by_namespace)
    assert len(by_namespace["votes"].points) == 720, \
        "an arrow per voting beam — two points each, and every beam of this scan votes"
    assert by_namespace["aim"].points[1].x == 1.0, "the wish is a unit vector along the nose"
    assert by_namespace["numbers"].text.startswith(CLEAR)
    assert all(by_namespace[name].points for name in ("votes", "aim", "field", "command")), \
        "an empty marker is invisible in exactly the way a wrong frame is, and is far more often the reason"


def test_nothing_is_drawn_at_all_when_a_node_has_no_view_publisher():
    """`markers:=false`, and a machine without `visualization_msgs`, both end here — and neither may raise."""
    node = SimpleNamespace(view_pub=None, pose=(0.0, 0.0, 0.0), goal=(1.0, 0.0), settings={},
                           get_clock=Clock)
    move_to_point.MoveToPoint.draw(node, Motion(ARRIVED, 0.0, 0.0, 0.0, 0.1))

    follower = SimpleNamespace(view_pub=None, scan=a_scan(), scan_frame="f", want=0.4, state=FOLLOWING,
                               range_max=8.0, increment=TWO_PI / 360, get_clock=Clock)
    wall_following.WallFollower.draw(follower, Steering(0.3, 0.0, FOLLOWING, 0.4))

    field = SimpleNamespace(view_pub=None, scan=a_scan(), scan_frame="f", aim=0.0,
                            settings=dict(repulsion_range=2.0), range_max=8.0, increment=TWO_PI / 360,
                            get_clock=Clock)
    obstacle_avoidance.ObstacleAvoidance.draw(field, Decision(0.0, 0.0, 0.0, 0.0, CLEAR, ""))

    assert view_markers.publish(None, [view_markers.dots("f", Time(sec=1), "ns", [(0.0, 0.0)])]) is None
