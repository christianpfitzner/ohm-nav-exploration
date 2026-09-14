"""Frontiers: which parts of a hall a lidar has not seen yet, and which of them to drive to first.

No ROS in this file, on purpose — it is the part worth testing on its own. The map is a grid with three
states per cell, painted by lidar rays. A frontier is a free cell that touches an unknown cell, and a
frontier worth driving to is a *clump* of them: one lone cell is a crack between two beams, a clump of
forty is a room the robot has not entered.
"""
from collections import deque, namedtuple
import math

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
AIM_WINDOW = 3                              # cells (0.3 m) a goal is checked for wall proximity in

#: metres, cells of it, and how much this clump is worth driving to now
Frontier = namedtuple("Frontier", "x y cells score")


class Grid:
    """An occupancy grid over the hall's metres, painted one beam at a time."""

    def __init__(self, width: float, height: float, resolution: float = 0.10):
        self.res = float(resolution)
        self.cells = np.full((max(1, round(height / self.res)), max(1, round(width / self.res))),
                             UNKNOWN, dtype=np.uint8)

    # --------------------------------------------------------------------------- painting
    def _paint(self, x: float, y: float, value: int) -> None:
        col, row = int(x / self.res), int(y / self.res)
        if 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]:
            self.cells[row, col] = value

    def _beam(self, x: float, y: float, angle: float, until: float, value: int) -> None:
        """Paint every cell a beam passes on its way to `until`."""
        step = self.res * 0.5
        dx, dy = math.cos(angle) * step, math.sin(angle) * step
        for _ in range(max(1, round(until / step))):
            self._paint(x, y, value)
            x, y = x + dx, y + dy

    def scan(self, pose, angles, ranges, range_max: float) -> None:
        """One scan from `pose` = (x, y, theta): free where the beam travelled, a wall where it stopped.

        A beam that answers nothing is painted free out of range — the one assumption in here. A wall
        that reflects too little to be seen becomes open floor, which is what a real driver does as well
        and which makes the map optimistic by exactly the amount the sensor's own limits are.
        """
        x, y, theta = pose
        self._paint(x, y, FREE)                     # the robot is standing here, whatever the beams say
        for angle, r in zip(angles, ranges):
            if not math.isfinite(r) or r >= range_max:
                self._beam(x, y, theta + angle, range_max, FREE)
                continue
            self._beam(x, y, theta + angle, max(0.0, r - self.res), FREE)
            self._paint(x + math.cos(theta + angle) * r, y + math.sin(theta + angle) * r, OCCUPIED)

    # --------------------------------------------------------------------------- frontiers
    def frontiers(self, robot=None, min_cells: int = 12, min_distance: float = 0.5,
                  avoid=(), avoid_radius: float = 0.8) -> list:
        """Clumps of free-against-unknown, best first.

        The score is cells per unit distance, so a wide opening down the hall beats a crack beside the
        wheel, and the division (rather than a subtraction) keeps the far side of a big hall from losing
        to the near side of a small one. `avoid` is where a drive already failed: blacklisted centres, so
        the next clump in line gets its turn instead of the same unreachable one forever. `robot` only
        costs distance — with none, every clump is ranked by size alone.
        """
        edge = self._against_unknown()
        seen = np.zeros(edge.shape, dtype=bool)
        found = []
        for row, col in np.argwhere(edge):
            if seen[row, col]:
                continue
            clump = self._grow(edge, seen, row, col)
            if len(clump) < min_cells:
                continue
            block = np.array(clump)
            row, col = self._aim(block)
            x = (col + 0.5) * self.res
            y = (row + 0.5) * self.res
            if robot and math.hypot(x - robot[0], y - robot[1]) < min_distance:
                continue                            # our own doorstep is not somewhere to drive
            if any(math.hypot(x - bx, y - by) < avoid_radius for bx, by in avoid):
                continue
            found.append(Frontier(x, y, len(clump), len(clump) / (0.5 + _distance(robot, x, y))))
        return sorted(found, key=lambda f: -f.score)

    def _aim(self, block):
        """Which cell of a clump to drive at: the one with the least wall around it.

        Not the centroid. A clump that bends around the corner of a wall has its middle *in* that wall,
        and a goal inside an obstacle is refused by every planner — so the whole opening would be
        written off as unreachable when in fact it is a door. The cell with the fewest occupied cells in
        its `AIM_WINDOW` neighbourhood is the middle of the opening, which is also where a robot should
        enter a room it has never seen. Ties go to the cell nearest the clump's centroid.
        """
        # padded with walls rather than with nothing: outside this grid there is the hall's own wall,
        # and padding with open space would make every cell near the border look wide open.
        walls = np.pad(self.cells == OCCUPIED, AIM_WINDOW, constant_values=True).astype(int)\
            .cumsum(0).cumsum(1)
        total = np.zeros((walls.shape[0] + 1, walls.shape[1] + 1), dtype=int)
        total[1:, 1:] = walls

        def nearby(rc):                                      # integral image: one window sum, no loop
            row, col = rc
            span = 2 * AIM_WINDOW
            return (total[row + span, col + span] - total[row, col + span]
                    - total[row + span, col] + total[row, col])

        centre = block.mean(axis=0)
        return min((tuple(rc) for rc in block), key=lambda rc: (nearby(rc), abs(rc[0] - centre[0])
                                                                + abs(rc[1] - centre[1])))

    def _against_unknown(self):
        """Free cells with an unknown cell in their 8 neighbourhood."""
        free = self.cells == FREE
        pad = np.pad(self.cells == UNKNOWN, 1, constant_values=False)  # out of the grid is out of the hall
        near = np.zeros(free.shape, dtype=bool)
        for row in (0, 1, 2):
            for col in (0, 1, 2):
                near |= pad[row:row + free.shape[0], col:col + free.shape[1]]
        return free & near

    @staticmethod
    def _grow(edge, seen, row, col) -> list:
        """One 8-connected clump of frontier cells, from (row, col) outwards."""
        clump, queue = [(row, col)], deque([(row, col)])
        seen[row, col] = True
        while queue:
            here = queue.popleft()
            for step in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
                nxt = (here[0] + step[0], here[1] + step[1])
                if 0 <= nxt[0] < edge.shape[0] and 0 <= nxt[1] < edge.shape[1] \
                        and edge[nxt] and not seen[nxt]:
                    seen[nxt] = True
                    clump.append(nxt)
                    queue.append(nxt)
        return clump


def _distance(robot, x: float, y: float) -> float:
    return 0.0 if robot is None else math.hypot(x - robot[0], y - robot[1])
