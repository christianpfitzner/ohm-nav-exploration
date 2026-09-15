"""Whether a goal is kept, and what ends one — the state machine of `frontier_node.py`.

    python3 -m pytest test          # skipped here if ROS 2 is not installed

The frontier rules are tested without ROS in `test_frontiers.py`. These tests need ROS, because what is under
test is a node with a timer, a clock and an action client. Nothing is spun and nothing is waited for: the map
is handed to `on_map`, the robot to `on_odom`, the clock is a counter and the ticks are called by hand, which
is also how a ten-second timeout is tested in a millisecond.

The bug these are written against was measured on a real robot and is one line long: the timer that looked at
the map also chose the goal, so the re-decision period was the timer period — a new goal every half second, a
robot that drove a metre towards each of them and arrived at none. Every test below is one way to bring that
back, plus the two rules that replaced it: reached, or no nearer for `goal_timeout_s`.

Where a test is about what the node does with a ranking rather than about producing one, `candidates` is
replaced with a fixed list. Producing one is `test_frontiers.py`'s business.
"""
import math
from math import hypot
import sys

import pytest

pytest.importorskip("rclpy", reason="a goal is a node's business; the frontier rules are tested without ROS")
pytest.importorskip("nav_msgs", reason="the map and the odometry arrive as real messages")
import rclpy                                                # noqa: E402
from nav_msgs.msg import Odometry, OccupancyGrid            # noqa: E402
from rclpy.parameter import Parameter                       # noqa: E402

sys.path.insert(0, ".")
from ohm_frontier.frontier_node import APPROACH_NAMESPACE, CLOCK_NAMESPACE, FrontierNode, \
    FRONTIERS_NAMESPACE, SCORES_NAMESPACE, SELECTED_NAMESPACE   # noqa: E402
from ohm_frontier.frontiers import Frontier                 # noqa: E402

UNKNOWN_CELLS, FREE_CELLS, WALL_CELLS = -1, 0, 100          # slam_toolbox's own convention
RES = 0.5


@pytest.fixture(scope="module")
def ros():
    """One ROS context for the file: a node needs it, and initialising the default one twice is an error."""
    rclpy.init(args=[])
    yield
    rclpy.try_shutdown()


@pytest.fixture
def node(ros):
    """A frontier node with nothing running: the tests call its callbacks, the timers never do."""
    made = FrontierNode()
    yield made
    made.destroy_node()


class Clock:
    """The node's clock as a counter.

    `now` is the only place the node reads time from, so replacing it on the instance is the whole trick, and
    it is what lets 10 s of standing still cost a test nothing. Message stamps still come from the node's real
    clock, which is fine: nothing here reads a stamp.
    """

    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def advance(self, seconds: float):
        self.t += seconds


def a_map(painted, width=8.0, height=6.0, res=RES, frame="slam_map"):
    """The payload of an `OccupancyGrid`, with rectangles painted into it as `(x0, x1, y0, y1, state)`.

    A real message rather than a stand-in because the node's own `on_map` reads it — resolution, origin and
    frame id included — and because the frame is the one thing the markers must not invent.

    The row order comes from the message, not from a picture of the hall: `nav_msgs/msg/OccupancyGrid` documents
    the data as "row-major order, starting with (0,0)" with the cell above the origin's corner at index
    `info.width`, so the array's row index *is* the y index. Painting it the way the hall looks on paper —
    the far row first — mirrors the whole map over the x axis, and every y the node then reports is measured
    from the far wall: measured here, where the wide opening of `two_openings` came out of the node at
    y = 2.25, in a wall, instead of at y = 3.75 in the doorway.
    """
    rows, cols = round(height / res), round(width / res)
    cells = [[UNKNOWN_CELLS] * cols for _ in range(rows)]
    for x0, x1, y0, y1, state in painted:
        for row in range(int(y0 / res), int(y1 / res)):
            for col in range(int(x0 / res), int(x1 / res)):
                cells[row][col] = state                       # row 0 is the row the origin is in
    message = OccupancyGrid()
    message.header.frame_id = frame
    message.info.resolution, message.info.width, message.info.height = res, cols, rows
    message.info.origin.position.x, message.info.origin.position.y = 0.0, 0.0
    message.data = [value for row in cells for value in row]
    return message


def two_openings():
    """A mapped hall with its own walls, a wall strip at x = 4.0 … 4.5, and nothing mapped beyond it.

    The same fixture as `test_frontiers.two_doorways`: the strip has a 1.0 m opening at y = 1.0 … 2.0 and a
    1.5 m one at y = 3.5 … 5.0, and with the robot at (0.5, 1.5) the wide one is the further and the better
    of the two — which is what makes it the goal, and makes the narrow one the goal after it.
    """
    return a_map([
        (0.5, 4.0, 0.5, 5.5, FREE_CELLS),                   # the hall
        (0.0, 0.5, 0.0, 6.0, WALL_CELLS),                   # its walls, all of them mapped
        (0.0, 4.5, 0.0, 0.5, WALL_CELLS),
        (0.0, 4.5, 5.5, 6.0, WALL_CELLS),
        (4.0, 4.5, 0.0, 6.0, WALL_CELLS),                   # the wall with the two openings
        (4.0, 4.5, 1.0, 2.0, FREE_CELLS),                   # the narrow one, near the robot
        (4.0, 4.5, 3.5, 5.0, FREE_CELLS),                   # the wide one, further away
    ])


def an_odometry(x: float, y: float, yaw: float = 0.0) -> Odometry:
    """An `<robot>/odom` message: a position and a heading, which is all the node reads out of it."""
    message = Odometry()
    message.pose.pose.position.x, message.pose.pose.position.y = x, y
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return message


def start(node, painted=None, robot=(0.5, 1.5, 0.0)) -> Clock:
    """Give the node a map and a robot, and take its clock away. After this, `on_timer` will work."""
    node.on_map(painted if painted is not None else two_openings())
    node.on_odom(an_odometry(*robot))
    clock = Clock()
    node.now = clock.now
    node.set_parameters([Parameter("min_frontier_cells", value=2)])     # the openings are 2 and 3 cells
    return clock


def offered(node, *frontiers):
    """Replace the search with a fixed ranking: what follows is about the node's rules, not about the map."""
    node.candidates = lambda: list(frontiers)


def a_frontier(x: float, y: float, cells: int = 20, score: float = 1.0) -> Frontier:
    return Frontier(x=x, y=y, heading=0.0, cells=cells, distance=hypot(x - 0.5, y - 1.5), score=score)


# ---------------------------------------------------------------------------- a goal is kept, and for how long


def test_a_goal_survives_ticks_although_a_better_candidate_keeps_appearing(node):
    """The bug, as a unit test: two candidates, a ticking timer, and a goal that must not move.

    The second candidate is better — 2.6 against 2.0 — but not better by `reselect_margin`, and the whole
    point of the margin is that "better" is not a reason to stop driving somewhere.
    """
    clock = start(node)
    chosen, better = a_frontier(4.0, 4.0, cells=30, score=2.0), a_frontier(4.0, 1.0, cells=26, score=2.6)
    offered(node, chosen)
    node.on_timer()
    assert node.goal == chosen
    offered(node, better, chosen)
    for _ in range(6):
        clock.advance(0.5)
        node.on_timer()
    assert node.goal == chosen, f"the goal moved after {clock.t:.1f} s of ticking"
    assert node.avoid == [], "keeping a goal must not write anything off"


def test_only_a_much_better_candidate_takes_a_live_goal_away(node):
    """The other half of the rule: the escape hatch works, and what it drops is not blacklisted."""
    clock = start(node)
    small = a_frontier(4.0, 4.0, cells=12, score=1.0)
    offered(node, small)
    node.on_timer()
    offered(node, a_frontier(1.0, 1.0, cells=40, score=1.6))        # 1.6 > 1.5 × 1.0
    clock.advance(0.5)
    node.on_timer()
    assert node.goal.x == 1.0 and node.goal.y == 1.0, \
        f"a candidate 1.6 times as good was refused by a goal worth 1.0: {node.goal}"
    assert node.avoid == [], "switching away from a goal nobody failed to reach must not blacklist it"


def test_a_walk_out_goal_gives_way_to_the_first_real_frontier(node):
    """A walk-out goal is a filler — its job is to give the mapper a reason to integrate a scan — so it
    stands aside for the first frontier, however small that frontier is."""
    clock = start(node)
    stroll = Frontier(x=3.0, y=3.0, heading=0.0, cells=0, distance=3.5, score=0.0)
    offered(node, stroll)
    node.on_timer()
    assert node.goal == stroll
    offered(node, a_frontier(4.0, 4.0, cells=12, score=0.4))
    clock.advance(0.5)
    node.on_timer()
    assert node.goal is not stroll, "a frontier appeared and the robot is still walking out"
    assert node.avoid == []


def test_the_same_frontier_seen_again_is_not_a_new_candidate(node):
    """`frontiers` re-aims at a clump as the map grows around it. A goal whose best cell moved by a cell is
    the same doorway, and a node that treats it as news sends the robot after its own frontier."""
    clock = start(node)
    doorway = a_frontier(4.0, 4.0, cells=12, score=1.0)
    offered(node, doorway)
    node.on_timer()
    shifted = a_frontier(4.0 + node.grid.res, 4.0, cells=12, score=1.0)   # half a cell over, same clump
    offered(node, shifted, doorway)
    for _ in range(4):
        clock.advance(0.5)
        node.on_timer()
    assert node.goal == doorway


def test_a_goal_survives_while_the_robot_keeps_getting_nearer(node):
    """10 s is the patience with a robot that is not arriving, not a deadline for the drive. A frontier at
    the far end of a hall is reachable and slower than 10 s; writing those off blacklists a whole hall."""
    clock = start(node)
    node.on_timer()
    assert node.goal is not None
    for x in (1.5, 2.5, 3.2, 3.8):                              # 12 s of real driving, closer every time
        node.on_odom(an_odometry(x, 3.6))
        clock.advance(4.0)
        node.on_timer()
    assert node.goal is not None, f"a goal was written off while the robot was approaching it: {node.avoid}"
    assert node.avoid == []


def test_a_goal_is_written_off_when_the_robot_stops_getting_nearer(node):
    """The measured failure: the stack accepted the goal and published nothing, and the node asked the same
    frontier again for as long as anyone watched. Now the goal dies, the place is written off, and the next
    frontier gets its turn — all of it without a second of waiting."""
    clock = start(node)
    node.on_timer()
    wide = node.goal
    assert wide is not None and wide.y > 3.0, f"the wide opening should have been the goal, got {wide}"
    for _ in range(24):                                         # 12 s of the robot standing where it is
        clock.advance(0.5)
        node.on_timer()
    assert any(abs(x - wide.x) < 0.1 and abs(y - wide.y) < 0.1 for x, y in node.avoid), \
        f"the frontier the robot never reached is still offerable: {node.avoid}"
    assert node.goal is not None and abs(node.goal.y - 1.75) < 0.6, \
        f"the next frontier did not get its turn, the node has {node.goal}"


def test_a_reached_goal_is_not_asked_again(node):
    """Arriving has to end the goal *and* end it as that frontier: a goal forgotten without being written off
    is re-taken on the next tick, which from outside looks like a robot arriving at one doorway twice a
    second for ever."""
    clock = start(node)
    node.on_timer()
    reached = node.goal
    node.on_odom(an_odometry(reached.x, reached.y))              # the robot is standing on the goal
    clock.advance(0.5)
    node.on_timer()
    assert node.goal != reached, "the node arrived and started driving there again"
    assert any(abs(x - reached.x) < 0.1 and abs(y - reached.y) < 0.1 for x, y in node.avoid)
    assert node.goal is not None, "with the reached one gone the narrow opening is still unexplored"


# --------------------------------------------------------------------------- the stack, and which goal it means


class RefusedHandle:
    """A nav2 goal handle that only has to remember whether it was cancelled."""

    def __init__(self):
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


def test_a_goal_the_node_dropped_is_cancelled_at_the_stack(node):
    """Forgetting a goal is not dropping it: `bt_navigator` serves one goal at a time, so the stack would go
    on driving to the place the node has moved on from."""
    clock = start(node)
    first = a_frontier(4.0, 4.0, cells=12, score=1.0)
    offered(node, first)
    node.on_timer()
    node.goal_handle = RefusedHandle()
    offered(node, a_frontier(1.0, 1.0, cells=40, score=3.0))
    clock.advance(0.5)
    node.on_timer()
    assert node.goal_handle is None and True
    assert node.goal.x == 1.0


def test_a_dropped_goal_is_cancelled_and_the_cancel_reaches_the_stack(node):
    clock = start(node)
    first = a_frontier(4.0, 4.0, cells=12, score=1.0)
    offered(node, first)
    node.on_timer()
    handle = RefusedHandle()
    node.goal_handle = handle
    node.drop_goal()
    assert handle.cancelled, "the node forgot its goal and left the stack driving to it"
    assert node.goal is None and node.goal_handle is None


def test_a_late_word_from_the_stack_is_answered_to_the_goal_it_was_about(node):
    """Both callbacks used to ask the node what the current goal was and get the newer answer, so the failure
    of an abandoned goal wrote off the place the *next* goal had been chosen for."""
    start(node)
    offered(node, a_frontier(4.0, 4.0, cells=12, score=1.0))
    node.on_timer()
    current = node.goal
    node.give_up(node.attempt - 1)                               # a stale refusal, about an older drive
    assert node.goal == current, "a word about an older drive dropped the goal it was not about"
    assert node.avoid == []
    node.give_up(node.attempt)                                   # the same word, about the right drive
    assert node.goal is None


def test_a_place_needs_three_nos_before_it_is_written_off(node):
    """A goal that leaves while nav2 is still activating is refused within a millisecond, and a robot started
    with its stack always does that; the third no is the one that means the place."""
    start(node)
    for _ in range(2):
        node.goal, node.attempt = a_frontier(4.0, 4.0), node.attempt + 1
        node.give_up(node.attempt)
    assert node.avoid == [], "two refusals are nav2 coming up, not a verdict about a place"
    node.goal, node.attempt = a_frontier(4.0, 4.0), node.attempt + 1
    node.give_up(node.attempt)
    assert node.avoid, "the third refusal should have written the place off"


# -------------------------------------------------------------------------------- the ranking, and its knobs


def test_the_weight_of_distance_can_be_turned_while_the_robot_is_driving(node):
    """`ros2 param set` on a weight has to reach the next decision, not the next restart — that is what makes
    the tuning panel worth opening, and it is the reason the node reads the weights per decision."""
    start(node)
    node.on_timer()
    wide_first = node.goal
    assert wide_first.y > 3.0, f"with size and distance both at one the wide opening should win: {wide_first}"
    node.set_parameters([Parameter("weight_size", value=0.0), Parameter("weight_orientation", value=0.0)])

    # The knob moves the ranking, and it does not steal the drive in progress: `reselect_margin` is the price
    # of changing one's mind, and a slider moved from a lecture chair is not a reason to drop a drive that is
    # working. Written as "the node must switch at once", this test asked for the flip-flop back.
    ranked = node.candidates()
    assert ranked[0].y < 3.0, f"distance alone should rank the near opening first: {ranked}"
    node.on_timer()
    assert node.goal == wide_first, f"a knob moved mid-drive dropped a working goal: {node.goal}"

    node.goal = None                        # the way a drive ends when the place is reached
    node.on_timer()
    assert node.goal is not None and node.goal.y < 3.0, \
        f"the next goal should be chosen with the weights as they are now: {node.goal}"


def test_a_weight_that_would_reward_a_far_frontier_is_refused(node):
    """A negative weight is not a weaker preference but an inverted one, and a ranking that pays for distance
    is not a ranking. The declared range is the guard, so the node cannot be talked into one."""
    start(node)
    result = node.set_parameters([Parameter("weight_distance", value=-1.0)])
    assert not result[0].successful, "a negative weight was accepted"
    assert node.get_parameter("weight_distance").value == pytest.approx(1.0)


def test_the_weight_of_size_is_worth_something_at_all(node):
    """A slider that changes nothing is indistinguishable from a slider that is not wired up, so each of the
    three is shown moving the ranking — this one is the direction the other tests take for granted."""
    start(node)
    node.set_parameters([Parameter("weight_distance", value=0.0),
                         Parameter("weight_orientation", value=0.0)])
    node.on_timer()
    assert node.goal is not None and node.goal.cells >= 3, \
        f"with only size in the ranking the wide opening must win, got {node.goal}"


# ------------------------------------------------------------------------------------ the picture for RViz


def published_markers(node, ticks: int = 1, clock: "Clock" = None):
    """The marker arrays the node would have sent, with the publisher wired to a list."""
    sent = []
    node.marker_pub.publish = sent.append
    for _ in range(ticks):
        if clock is not None:
            clock.advance(0.5)
        node.on_timer()
    return sent


def test_every_candidate_is_drawn_and_the_chosen_one_apart_from_them(node):
    """RViz shows all frontiers and the selected one, and the two are namespaces of one topic so a display can
    be switched without losing the other. The frame is the one the map message carried."""
    clock = start(node)
    sent = published_markers(node, clock=clock)
    assert sent, "nothing was drawn while the node had a map and a robot"

    # Namespaces, not markers, are what RViz's checkboxes switch, and one text is one marker in ROS 2 — so the
    # picture is counted by namespace, which is also the thing the lecturer is switching on and off.
    by_namespace = {}
    for marker in sent[-1].markers:
        by_namespace.setdefault(marker.ns, []).append(marker)
    assert set(by_namespace) == {FRONTIERS_NAMESPACE, SELECTED_NAMESPACE, SCORES_NAMESPACE,
                                 CLOCK_NAMESPACE, APPROACH_NAMESPACE}, sorted(by_namespace)
    candidates, chosen = by_namespace[FRONTIERS_NAMESPACE][0], by_namespace[SELECTED_NAMESPACE][0]
    assert len(candidates.points) == 2, "the hall offers two openings and the picture shows one"
    assert len(chosen.points) == 1, "one of them is the goal, and only one may be"
    assert candidates.header.frame_id == chosen.header.frame_id == "slam_map", \
        "the markers are in the frame of the map, which is not assumed anywhere"

    # the namespaces beyond the two dots are the reason the picture is worth opening: the arithmetic, the
    # clock the goal is running against, and the way it intends to travel
    written = [marker.text for marker in by_namespace[SCORES_NAMESPACE]]
    assert len(written) == 2, f"every candidate carries its own numbers, these carry {written}"
    assert all("cells" in line and "score" in line for line in written), written
    assert "s left to get nearer" in by_namespace[CLOCK_NAMESPACE][0].text
    assert len(by_namespace[APPROACH_NAMESPACE][0].points) == 2, "an arrow is two points: the robot and the goal"


def test_the_selected_marker_disappears_when_there_is_no_goal(node):
    """A stale orange dot in the middle of a hall reads as "that is where it is going", so the namespace is
    deleted rather than left standing."""
    clock = start(node)
    sent = published_markers(node, ticks=2, clock=clock)
    assert len(sent) == 2, "the picture goes out every tick, not only when something changed"
    alive = {marker.ns: marker for marker in sent[-1].markers}[SELECTED_NAMESPACE]
    assert len(alive.points) == 1, f"while a goal is being driven to, the dot is drawn: {alive}"

    node.goal = None
    # Not reachable through `on_timer`, and that is the behaviour the owner asked for rather than a hole: a
    # goal dropped with openings left on the map is replaced in the same tick. The state a display has to
    # survive is the one where there is nothing left to take — every place reached or written off — so the
    # picture for it is drawn directly, which is the same call `on_timer` would make.
    standing = []
    node.marker_pub.publish = standing.append
    node.publish_markers([])
    gone = [marker for marker in standing[-1].markers if marker.ns == SELECTED_NAMESPACE]
    assert len(gone) == 1 and not gone[0].points and gone[0].action == gone[0].DELETE, gone[0]
    assert all(marker.action != marker.DELETEALL for marker in standing[-1].markers), \
        "DELETEALL would take the candidate dots published earlier in the same array along with the goal"


def test_a_map_that_grew_nothing_is_not_mistaken_for_a_new_map(node):
    """slam_toolbox publishes `/map` on its own clock, and a map that did not change is not news.

    `on_map` compares `Grid.checksum` and keeps the grid it has, which is how the survey cached on that grid
    survives: the clumps of this map are still the clumps of this map. The other half of the test is the risk
    the cache buys — a map that did grow must not be dropped as a duplicate, because a node that stops
    looking at new floor stops exploring.
    """
    node.on_map(two_openings())
    kept = node.grid
    node.on_map(two_openings())
    assert node.grid is kept, "an identical map replaced the grid, its cached survey included"

    grown = two_openings()
    grown.data[0] = FREE_CELLS                          # one cell of floor the map before did not have
    node.on_map(grown)
    assert node.grid is not kept, "a map that grew was treated as the map it was"
