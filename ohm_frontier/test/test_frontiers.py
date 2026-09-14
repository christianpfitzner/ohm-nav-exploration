"""The grid and its frontiers, tested without ROS — which is why `frontiers.py` imports no rclpy.

A wall, a doorway and an unexplored room behind it is the whole thing this package does, so it is worth
checking with a crayon and a handful of asserts rather than by driving a robot for a minute.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_frontier.frontiers import FREE, OCCUPIED, UNKNOWN, Grid        # noqa: E402


def paint(grid, x0, x1, y0, y1, value):
    """Fill a rectangle given in metres — how the tests build a hall."""
    for x in np.arange(x0, x1 - 1e-9, grid.res):
        for y in np.arange(y0, y1 - 1e-9, grid.res):
            grid._paint(x, y, value)


def two_doorways():
    """Left half mapped, right half never seen, and one wall between them with two holes: a wide
    opening at y ≈ 4.75 and a narrow one at y ≈ 1.25."""
    grid = Grid(8.0, 6.0, 0.5)
    paint(grid, 0.0, 4.0, 0.0, 6.0, FREE)
    paint(grid, 4.0, 4.5, 0.0, 6.0, OCCUPIED)
    for y0, y1 in ((1.0, 2.0), (4.0, 5.5)):
        paint(grid, 4.0, 4.5, y0, y1, FREE)
    return grid


def test_a_scan_paints_floor_up_to_the_wall_and_nothing_behind_it():
    grid = Grid(8.0, 6.0, 0.1)
    grid.scan((1.0, 1.0, 0.0), [0.0], [2.0], range_max=8.0)
    assert grid.cells[10, 12] == FREE, "the cell the robot stands in is free"
    assert grid.cells[10, 30] == OCCUPIED, "a beam that stopped at two metres found a wall"
    assert grid.cells[10, 40] == UNKNOWN, "behind that wall this lidar has never been"


def test_a_beam_that_answers_nothing_is_free_out_of_range():
    grid = Grid(8.0, 6.0, 0.1)
    grid.scan((1.0, 1.0, 0.0), [0.0], [float("inf")], range_max=8.0)
    assert grid.cells[10, 75] == FREE, "no echo, so it is open as far as the beam reached"
    assert grid.cells[10, 2] == UNKNOWN, "and the direction it never looked in is still unknown"


def test_a_frontier_is_free_floor_next_to_unknown():
    found = two_doorways().frontiers(robot=(0.5, 0.5), min_cells=2)
    assert len(found) == 2, [f._asdict() for f in found]
    assert all(f.cells in (2, 3) for f in found), "one clump per doorway, in the wall, not behind it"
    assert all(4.0 <= f.x < 4.5 for f in found), "a goal inside a wall would be refused by every planner"


def test_the_outside_of_the_grid_is_not_an_unexplored_room():
    grid = Grid(8.0, 6.0, 0.5)
    paint(grid, 0.0, 8.0, 0.0, 6.0, FREE)
    assert grid.frontiers(robot=(4.0, 3.0), min_cells=1) == [], \
        "a hall mapped to its own walls is finished, not surrounded by unknown"


def test_the_wide_opening_wins_over_the_near_crack():
    """Both doorways are in the same wall, so the far one is also the more expensive one to drive to —
    and it wins anyway, because three cells of frontier are three chances to get through."""
    found = two_doorways().frontiers(robot=(0.5, 0.5), min_cells=2)
    assert found[0].y == pytest.approx(4.75, abs=0.3), "the wide opening first"
    assert found[0].score > found[1].score
    assert found[1].y == pytest.approx(1.5, abs=0.3)


def test_a_failed_goal_lets_the_next_frontier_have_its_turn():
    wide = two_doorways().frontiers(robot=(0.5, 0.5), min_cells=2)[0]
    left = two_doorways().frontiers(robot=(0.5, 0.5), min_cells=2, avoid=[(wide.x, wide.y)])
    assert len(left) == 1 and left[0].y < 2.0, "the crack in the wall is all that is left to try"


def test_a_goal_is_never_the_middle_of_a_bent_clump():
    """A room mapped around a pillar has its frontier cells in a ring, and the middle of a ring is the
    pillar. Sending the robot into the pillar is not a slow goal, it is a goal every planner refuses."""
    grid = Grid(8.0, 6.0, 0.5)
    paint(grid, 4.0, 7.0, 1.0, 3.5, FREE)          # a room, mapped
    paint(grid, 5.5, 6.0, 1.5, 3.0, OCCUPIED)      # a pillar standing in it
    found = grid.frontiers(robot=(0.5, 0.5), min_cells=4)
    assert len(found) == 1, [f._asdict() for f in found]
    goal = found[0]
    ring = np.argwhere(grid._against_unknown()).mean(axis=0)          # what a centroid would have given
    assert 5.5 <= (ring[1] + .5) * grid.res <= 6.0 and 1.5 <= (ring[0] + .5) * grid.res <= 3.0, \
        "the middle of this ring is the pillar itself — that is the mistake this guards"
    assert grid.cells[int(goal.y / grid.res), int(goal.x / grid.res)] == FREE, \
        f"the node aimed at a wall: ({goal.x}, {goal.y})"


def test_min_cells_is_what_tells_a_crack_from_a_room():
    grid = two_doorways()
    assert len(grid.frontiers(robot=(0.5, 0.5), min_cells=2)) == 2
    assert len(grid.frontiers(robot=(0.5, 0.5), min_cells=3)) == 1, "the two-cell crack is noise"
    assert grid.frontiers(robot=(0.5, 0.5), min_cells=4) == []
