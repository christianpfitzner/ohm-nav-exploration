"""Tests for the frontier rules and the one map they read.

Run from the package directory:

    python3 -m pytest test

`frontiers.py` imports no ROS, so these need no ROS either — the map is built by hand, cell by cell, the
way a student would draw it on paper.
"""
from math import cos, hypot, pi, sin
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, ".")
from ohm_frontier.frontiers import FREE, OCCUPIED, UNKNOWN, Frontier, Grid, to_frame  # noqa: E402

RES = 0.5


def paint(grid, x0, x1, y0, y1, value):
    for x in np.arange(x0, x1 - 1e-9, grid.res):
        for y in np.arange(y0, y1 - 1e-9, grid.res):
            grid._paint(x, y, value)


def two_doorways():
    """A mapped room with its own walls, a wall strip at x = 4.0 … 4.5, and nothing mapped beyond it.

    The wall strip has a 1.0 m opening at y = 1.0 … 2.0 and a 1.5 m one at y = 3.5 … 5.0, so the frontier
    cells are the five cells of those two openings. The outer walls are drawn on purpose: a room whose own
    wall was never mapped would have a frontier along that wall, and that would be right — it is just not
    what this fixture is about.
    """
    grid = Grid(8.0, 6.0, RES)
    paint(grid, 0.5, 4.0, 0.5, 5.5, FREE)                 # the room
    paint(grid, 0.0, 0.5, 0.0, 6.0, OCCUPIED)             # its walls, all mapped
    paint(grid, 0.0, 4.5, 0.0, 0.5, OCCUPIED)
    paint(grid, 0.0, 4.5, 5.5, 6.0, OCCUPIED)
    paint(grid, 4.0, 4.5, 0.0, 6.0, OCCUPIED)             # the wall with the two openings
    for y0, y1 in ((1.0, 2.0), (3.5, 5.0)):
        paint(grid, 4.0, 4.5, y0, y1, FREE)
    return grid


def test_a_frontier_is_free_floor_next_to_unknown_and_next_to_nothing_else():
    grid = two_doorways()
    edge = grid._against_unknown()
    assert edge.sum() == 5, f"expected 5 frontier cells, both openings wide open, got {edge.sum()}"
    assert {(int(r), int(c)) for r, c in np.argwhere(edge)} == \
        {(2, 8), (3, 8), (7, 8), (8, 8), (9, 8)}


def test_a_frontier_at_the_edge_of_the_map_is_still_a_frontier():
    """A SLAM map covers what has been seen. Its border is not a wall, it is "not yet" — so a room that
    has only been seen from one side still has its far side to look for, right at the edge."""
    grid = Grid(4.0, 4.0, RES)
    paint(grid, 0.0, 2.0, 0.0, 4.0, FREE)             # half a room mapped, and the map stops there
    assert grid._against_unknown().sum() > 0, "the map's own edge is unknown ground and must count"


def test_the_wide_opening_wins_over_the_near_crack():
    """The whole prioritisation in one comparison: the robot stands near the narrow opening, which is 1.6 m
    away, and the wide one is 4.6 m away and 50 % wider. Distance alone picks wrong, area alone picks
    right, cells-per-metre picks right and says by how much."""
    found = two_doorways().frontiers(robot=(0.5, 1.5), min_cells=2)
    assert len(found) == 2, [f._asdict() for f in found]
    first, second = found
    assert first.cells > second.cells, "the wider opening must be first"
    assert first.distance > second.distance, "…although it is the further one"
    assert first.score > second.score


def test_a_goal_is_far_enough_away_and_not_inside_a_wall():
    found = two_doorways().frontiers(robot=(0.5, 1.5), min_cells=2)
    for f in found:
        assert f.distance > 0.45, "a goal under the robot only makes it spin"
        assert 4.0 <= f.x <= 4.5, f"the frontier cells are in the wall strip, not at {f.x}"
        assert grid_free(two_doorways(), f), f"the node aimed at a wall: ({f.x}, {f.y})"


def grid_free(grid, f):
    return grid.cells[int(f.y / grid.res), int(f.x / grid.res)] == FREE


def test_min_cells_is_what_tells_a_crack_from_a_room():
    grid = two_doorways()
    assert len(grid.frontiers(robot=(0.5, 1.5), min_cells=2)) == 2
    assert grid.frontiers(robot=(0.5, 1.5), min_cells=6) == [], \
        "both openings are under six cells, so a robot looking for rooms should see none here"


def test_a_failed_goal_lets_the_next_frontier_have_its_turn():
    """Blacklisting: the stack refused this goal, so this frontier is no longer the node's business."""
    grid = two_doorways()
    first = grid.frontiers(robot=(0.5, 1.5), min_cells=2)[0]
    rest = grid.frontiers(robot=(0.5, 1.5), min_cells=2, avoid=[(first.x, first.y)], avoid_radius=0.75)
    assert len(rest) == 1, [f._asdict() for f in rest]
    assert abs(rest[0].y - first.y) > 1.0, "the other opening must have its turn"


def test_a_goal_is_never_the_middle_of_a_bent_clump():
    """A room mapped around a pillar has its frontier cells in a ring — and the middle of a ring is the
    pillar. A goal inside an obstacle is not a slow goal but a refused one, and the opening next to it then
    gets blacklisted as unreachable."""
    grid = Grid(8.0, 6.0, RES)
    paint(grid, 4.0, 7.0, 1.0, 3.5, FREE)          # a room, mapped
    paint(grid, 5.5, 6.0, 1.5, 3.0, OCCUPIED)      # a pillar standing in it
    found = grid.frontiers(robot=(0.5, 0.5), min_cells=4)
    assert len(found) == 1, [f._asdict() for f in found]
    goal = found[0]
    ring = np.argwhere(grid._against_unknown()).mean(axis=0)     # what a centroid would have aimed at
    assert 5.5 <= (ring[1] + .5) * grid.res <= 6.0 and 1.5 <= (ring[0] + .5) * grid.res <= 3.0, \
        "the middle of this ring is the pillar itself — that is the mistake this guards"
    assert grid.cells[int(goal.y / grid.res), int(goal.x / grid.res)] == FREE, \
        f"the node aimed at a wall: ({goal.x}, {goal.y})"


# ---------------------------------------------------------------------------- the map as it arrives


def a_map_as_slam_toolbox_publishes_it(cells, origin=(0.0, 0.0), res=0.5):
    """The payload of a `nav_msgs/msg/OccupancyGrid`: -1 unknown, 0 free, 100 occupied, rows from origin up.

    Not a ROS message here — `Grid.from_map` only ever reads these fields, so a test that fills them is the
    same test with less installed.
    """
    height, width = cells.shape
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="slam_map"),
        info=SimpleNamespace(
            map_load_time=None, resolution=float(res), width=int(width), height=int(height),
            origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1], z=0.0))),
        data=cells.flatten().tolist())


def test_the_states_of_the_map_are_read_as_its_own_convention_says():
    """-1 is unknown and 0 is free, which is the opposite of what a truthiness test concludes — the usual
    way this algorithm stops working after the first room."""
    cells = np.full((4, 4), -1, dtype=int)
    cells[1:3, 1:3] = 0            # free floor
    cells[1, 1] = 100              # a wall
    grid = Grid.from_map(a_map_as_slam_toolbox_publishes_it(cells))
    assert (grid.cells == UNKNOWN).sum() == 12      # of 16: 3 free, 1 wall, 12 not seen yet
    assert grid.cells[2, 2] == FREE and grid.cells[1, 1] == OCCUPIED


def test_the_goal_is_in_metres_of_the_map_not_of_the_hall():
    """A SLAM map is anchored wherever the first scan found the robot, so its origin has to be carried all
    the way to the goal — otherwise every goal is published at the wrong place by exactly the offset of the
    map."""
    cells = np.full((6, 6), -1, dtype=int)
    cells[:4, :4] = 0
    origin = (10.0, 20.0)
    grid = Grid.from_map(a_map_as_slam_toolbox_publishes_it(cells, origin=origin))
    found = grid.frontiers(robot=(origin[0] + 0.75, origin[1] + 0.75), min_cells=1,
                           min_distance=0.4)
    assert found, "a room that ends halfway is unexplored beyond that"
    edge = origin[0] + cells.shape[1] * RES, origin[1] + cells.shape[0] * RES
    for f in found:
        assert origin[0] <= f.x < edge[0] and origin[1] <= f.y < edge[1], \
            f"the goal ({f.x}, {f.y}) is outside the map, which runs from {origin} to {edge}"


def test_a_frontier_comes_with_the_heading_that_faces_into_it():
    cells = np.full((6, 6), -1, dtype=int)
    cells[:4, :4] = 0
    grid = Grid.from_map(a_map_as_slam_toolbox_publishes_it(cells))
    found = grid.frontiers(robot=(0.75, 0.75), min_cells=1, min_distance=0.4)
    assert found
    for f in found:
        assert np.arctan2(f.y - 0.75, f.x - 0.75) == pytest.approx(f.heading, abs=1e-6), \
            "nose-first: the first new scan then looks into the unknown instead of behind"


def test_a_frontier_around_the_robot_gives_a_goal_in_front_of_it():
    """The first minute of every mapping run: a disc of free floor around the robot, unknown beyond it, and
    — until a beam has hit something — no wall in the map at all. The clump is the whole ring, its middle is
    under the robot, and a goal under the robot is no goal at all."""
    grid = Grid(6.0, 6.0, 0.1)
    centre = (3.0, 3.0)
    for row in range(grid.cells.shape[0]):
        for col in range(grid.cells.shape[1]):
            x, y = grid.metres(row, col)
            if hypot(x - centre[0], y - centre[1]) < 1.2:
                grid.cells[row, col] = FREE
    found = grid.frontiers(robot=centre, min_cells=20, min_distance=0.45)
    assert found, "a ring of unexplored floor around the robot is a frontier, not a reason to do nothing"
    assert min(hypot(f.x - centre[0], f.y - centre[1]) for f in found) >= 0.45, (
        "the goal has to be somewhere the robot is not standing")


def a_transform(x, y, yaw):
    """A `geometry_msgs/Transform` saying: this is where the child frame sits inside the parent."""
    return SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=0.0),
                           rotation=SimpleNamespace(x=0.0, y=0.0, z=sin(yaw / 2), w=cos(yaw / 2)))


def test_a_pose_from_the_odometry_arrives_in_the_frame_the_map_is_in():
    """`/map` and `<robot>/odom` are apart by whatever the mapper corrects for the odometry drifting.
    Comparing a map cell with an odometry position without that correction compares two places, not one."""
    assert to_frame((1.0, 0.0, 0.5), a_transform(0.0, 0.0, 0.0)) == pytest.approx((1.0, 0.0, 0.5))
    assert to_frame((1.0, 0.0, 0.5), a_transform(2.0, 3.0, 0.0)) == pytest.approx((3.0, 3.0, 0.5))
    turned = to_frame((1.0, 0.0, 0.0), a_transform(0.0, 0.0, pi / 2))
    assert (turned[0], turned[1], turned[2]) == pytest.approx((0.0, 1.0, pi / 2), abs=1e-9)


def spawn_pocket():
    """One metre of known floor around a robot that has just been set down, unknown everywhere else.

    This is the map a mapper publishes in the first seconds of a run: the robot has not moved yet, so the
    only floor it has validated is the floor under it.
    """
    grid = Grid(10.0, 10.0, RES)
    paint(grid, 4.5, 5.5, 4.5, 5.5, FREE)                  # the metre of known floor
    paint(grid, 4.0, 4.5, 4.0, 6.0, OCCUPIED)              # a wall the lidar did reach
    return grid


def test_a_frontier_under_the_wheels_is_no_goal_but_a_walk_out_is():
    grid, robot = spawn_pocket(), (5.0, 5.0)
    assert grid.frontiers(robot=robot, min_distance=1.2) == []     # everything known is underfoot
    stroll = grid.walk_out(robot, reach=4.0)
    assert stroll is not None and stroll.cells == 0                # not a frontier, just a place to go
    assert 0.35 < stroll.distance <= 4.0                           # far enough to drive, near enough to be known
    row, col = int(stroll.y / grid.res), int(stroll.x / grid.res)
    assert grid.cells[row, col] == FREE                            # never a step into the unknown


def test_a_walk_out_stops_at_its_reach_and_walks_around_the_written_off():
    grid = Grid(20.0, 10.0, RES)
    paint(grid, 1.0, 9.0, 4.5, 5.5, FREE)                          # a corridor, mapped as far as 9 m
    robot = (1.0, 5.0)
    near = grid.walk_out(robot, reach=2.0)
    assert 1.0 <= near.distance <= 2.0 + RES                       # the reach is the reach, not a suggestion
    far = grid.walk_out(robot, reach=8.0)
    assert far.distance > 7.0                                      # with room to walk, it walks
    again = grid.walk_out(robot, reach=8.0, avoid=[(far.x, far.y)], avoid_radius=1.0)
    assert hypot(again.x - far.x, again.y - far.y) >= 1.0           # the end of the corridor is written off


def test_a_map_that_did_not_change_is_walked_once_and_a_map_that_did_is_walked_again():
    """The node looks the map over twice a second while slam_toolbox republishes whether or not it grew.

    `Grid._survey` keeps the clumps and the wall counts against `Grid.checksum` so that per-tick work is the
    aiming and the ranking — 60 ms of the period on a big map is clump-walking, and on an unchanged map it
    buys nothing. What makes the cache legitimate is this test: a map that changed must not be answered with
    the clumps of the map it was, and a hand-painted one must invalidate as surely as a message does.
    """
    grid = two_doorways()
    asked = [(f.x, f.y, f.cells) for f in grid.frontiers(robot=(0.5, 1.5, 0.0))]
    assert grid._surveyed is not None, "the survey was not remembered, so every tick pays for the walk again"

    walked = grid._surveyed[1]
    assert [(f.x, f.y, f.cells) for f in grid.frontiers(robot=(0.5, 1.5, 0.0))] == asked, \
        "the same map asked about twice answers differently"
    assert grid._surveyed[1] is walked, "the survey was thrown away although nothing moved"

    paint(grid, 4.0, 4.5, 3.5, 5.0, OCCUPIED)           # the wide opening becomes wall
    after = grid.frontiers(robot=(0.5, 1.5, 0.0))
    assert grid._surveyed[1] is not walked, "the clumps of the old map answered for the new one"
    assert len(grid._surveyed[1]) < len(walked), f"the wall that closed left {grid._surveyed[1]} standing"
