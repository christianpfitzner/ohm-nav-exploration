"""Frontiers in a map somebody else made, and which of them to drive to first.

Nothing in here imports ROS. The map arrives as the payload of a `nav_msgs/msg/OccupancyGrid` — what
slam_toolbox publishes — which is also what a test can draw by hand.

The idea, in the order it is used:

* **Frontier** — a *free* cell with an *unknown* cell in its 8-neighbourhood: known floor that leads into
  territory the lidar has not seen. One such cell is a crack between two beams; forty of them are a room
  nobody has entered yet.
* **Aim at the opening, not at the clump** — see `_aim`. The middle of a clump that bends around a corner
  stands in that corner, and a goal inside a wall is refused by every planner.
* **Rank by weighted preference** — see `frontiers` and `Weights`: how big the opening is, how much wall is
  around the cell aimed at and how far it is, each in units of the caller's own thresholds so that the three
  can be added at all. `cells / metres` was the same trade-off with the weights fixed at one and no way to
  turn either; the weights are parameters now because which of the three wins is exactly what a lecture wants
  to turn up and watch.
* **Blacklist** — see `frontier_node`. A goal that was refused or never reached writes off its
  surroundings, so the next frontier gets its turn instead of being asked forever.

About `unknown`, where the subtleties are. A map says one of three things about every cell — free,
occupied, unknown — and **unknown is not free**. slam_toolbox writes it as `-1`, and the edge of its grid
is unknown as well, because nothing has been seen there. A frontier rule of "free next to not-free"
cannot tell those apart and stops finding rooms after the first minute.
"""
from collections import namedtuple
from math import atan2, cos, hypot, sin
import zlib

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
AIM_WINDOW = 3          # cells around a goal that are counted for wall proximity (0.15–0.3 m)
WINDOW_CELLS = (2 * AIM_WINDOW) ** 2       # how many cells that window holds, so a count becomes a ratio
NEIGHBOURS = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)]

Frontier = namedtuple("Frontier", "x y heading cells distance score")

#: What the ranking values, as the operator named them. Units differ — cells, a ratio, metres — so `frontiers`
#: divides each by a threshold of the caller's own before adding: one unit of `weight_size` is a clump of
#: `min_cells`, one unit of `weight_distance` is a goal at `min_distance`. That leaves the weights
#: dimensionless and comparable, and — because the divisor is a parameter rather than what one particular
#: call happened to find — it leaves a score meaning the same thing on two maps and on two days.
Weights = namedtuple("Weights", "size orientation distance")

#: Size and distance at one apiece is the trade-off the ranking always made. Orientation is in with a small
#: weight rather than none: walking in nose-first is worth something — the first new scan looks into the
#: unknown instead of at where the robot came from — but nothing measured here says it outweighs a room,
#: and a default that silently prefers the frontier straight ahead can leave a robot circling one doorway.
DEFAULT_WEIGHTS = Weights(size=1.0, orientation=0.25, distance=1.0)


def window_sum(total: np.ndarray, rows, cols) -> np.ndarray:
    """Wall cells in the `AIM_WINDOW` neighbourhood of every cell in `rows`, `cols`, in four lookups.

    The integral image (`Grid._wall_counter`) holds the running sum of occupied cells, so a window sum is
    the sum to its far corner minus the two strips before it plus the overlap counted twice. Row 0 of the
    image is the padding, which is why the far corner is `+ 2 * AIM_WINDOW + 1` away rather than `+ span`.
    """
    span = 2 * AIM_WINDOW
    return (total[rows + span + 1, cols + span + 1] - total[rows, cols + span + 1]
            - total[rows + span + 1, cols] + total[rows, cols])


def to_frame(pose: tuple, transform) -> tuple:
    """A pose in a frame, moved into the frame a `geometry_msgs/Transform` describes as its parent.

    Used for the robot's odometry pose and the SLAM map: `/map` and `<robot>/odom` are apart by exactly the
    correction the mapper makes for the odometry drifting, which is centimetres in a well-behaved simulator
    and metres in the exercises where the odometry is deliberately ruined. Comparing a map cell with an
    odometry position without that correction is comparing two places.
    """
    q, t = transform.rotation, transform.translation
    yaw = 2.0 * atan2(q.z, q.w)                             # the world is flat: yaw only
    x, y = pose[0], pose[1]
    return (t.x + cos(yaw) * x - sin(yaw) * y,
            t.y + sin(yaw) * x + cos(yaw) * y,
            pose[2] + yaw)


class Grid:
    """An occupancy grid plus the frontier rules.

    Rows run upwards from the grid's own origin, so cell (row, col) sits at `origin + (col + .5, row + .5)
    * res`. The grid's origin, not the hall's: a SLAM map is anchored wherever the first scan found the
    robot, which is why nothing here may assume the map starts at (0, 0) of the world.
    """

    def __init__(self, width: float, height: float, res: float, origin: tuple = (0.0, 0.0)):
        self.res = float(res)
        self.origin = (float(origin[0]), float(origin[1]))
        self.cells = np.full((max(1, round(height / res)), max(1, round(width / res))), UNKNOWN,
                             dtype=np.uint8)
        self._checksum, self._surveyed = None, None     # both re-derived: see `checksum` and `_survey`

    @classmethod
    def from_map(cls, msg) -> "Grid":
        """A `nav_msgs/msg/OccupancyGrid` as it comes from slam_toolbox.

        `-1` is unknown there and `0` is free — the other way round from a truthiness test, which is how
        this is usually gotten wrong. At or above 50 counts as occupied, the line nav2 draws too.
        """
        info = msg.info
        grid = cls(info.width * info.resolution, info.height * info.resolution, info.resolution,
                   (info.origin.position.x, info.origin.position.y))
        raw = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        grid.cells = np.where(raw < 0, UNKNOWN, np.where(raw >= 50, OCCUPIED, FREE)).astype(np.uint8)
        return grid

    def checksum(self) -> int:
        """One number for the cells as they stand, cheap enough to ask for twice a second.

        slam_toolbox republishes `/map` on its own clock, map changed or not, and this package looks the map
        over on every one of those messages. A crc32 over the bytes of the grid is 0.2 ms on a 40 x 60 m map,
        which makes "has anything actually changed" a question that can be asked before the 60 ms of work
        rather than after it.
        """
        if self._checksum is None:
            self._checksum = zlib.crc32(self.cells.tobytes())
        return self._checksum

    def _survey(self) -> tuple:
        """The two things about a map that do not depend on where the robot is: its clumps and its walls.

        `frontiers` is asked twice a second and only one part of it moves with the robot. The clumps and the
        integral image are properties of the map, so they are derived once per map and what happens on every
        tick is the aiming and the ranking. Keyed on `checksum`, which `~Grid._paint` clears so a hand-drawn
        map somebody painted over cannot answer with the clumps of the map it was.
        """
        if self._surveyed is None or self._surveyed[0] != self.checksum():
            self._surveyed = (self.checksum(), self._clumps(self._against_unknown()), self._wall_counter())
        return self._surveyed[1], self._surveyed[2]

    def metres(self, row: int, col: int) -> tuple:
        return (self.origin[0] + (col + 0.5) * self.res, self.origin[1] + (row + 0.5) * self.res)

    def _paint(self, x: float, y: float, value: int) -> None:
        """One named cell — so a test can draw a hall without a robot or a lidar."""
        col, row = int(x / self.res), int(y / self.res)
        if 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]:
            self.cells[row, col] = value
            self._checksum = self._surveyed = None      # the map is different now, so nothing cached holds

    # ---------------------------------------------------------------------- the frontier itself

    def _against_unknown(self):
        """Free cells with an unknown cell in their 8-neighbourhood.

        Off the grid counts as unknown: a SLAM grid covers only what has been mapped, so its edge really
        is unexplored floor. (The *hall's* border is a wall — but nothing here knows the hall.)
        """
        unknown = self.cells == UNKNOWN
        padded = np.pad(unknown, 1, constant_values=True)
        rows, cols = self.cells.shape
        edge = np.zeros_like(unknown)
        for dr, dc in NEIGHBOURS:
            edge |= padded[1 + dr:1 + dr + rows, 1 + dc:1 + dc + cols]
        return edge & (self.cells == FREE)

    def _clumps(self, edge) -> list:
        """Connected frontier cells, in the order they are met — the fill itself asks for no thresholds.

        The mask and the visited marks are plain Python sequences of one byte per cell, not the numpy mask.
        A flood fill asks eight neighbour questions per cell, and a numpy boolean lookup costs roughly ten
        times what a Python one costs, so the array that makes the *neighbourhood* fast is the wrong tool
        for the *walking*. Measured on a 40 × 60 m map at 0.05 m whose unknown boundary is one long clump —
        what a robot that has mapped half of a large open hall actually has — the walk went from 273 ms to
        the number in `test_frontiers.py`, on a node that looks the map over twice a second: a fill that
        costs more than the period is a node that never catches up, and it is invisible at the spawn, where
        the map is a metre across.
        """
        rows, cols = edge.shape
        flat = edge.view(np.uint8).tobytes()                    # one memcpy, then no numpy in the loop
        seen = [bytearray(cols) for _ in range(rows)]
        groups = []
        for start in np.argwhere(edge):
            row, col = int(start[0]), int(start[1])
            if seen[row][col]:
                continue
            seen[row][col] = 1
            stack, clump = [(row, col)], []
            while stack:                                        # flood fill with an explicit stack
                row, col = stack.pop()
                clump.append((row, col))
                for dr, dc in NEIGHBOURS:
                    r, c = row + dr, col + dc
                    if 0 <= r < rows and 0 <= c < cols and not seen[r][c] and flat[r * cols + c]:
                        seen[r][c] = 1
                        stack.append((r, c))
            groups.append(clump)
        return groups

    def _aim(self, clump, robot: tuple = None, min_distance: float = 0.0, integral=None):
        """Which cell of a clump to drive at: the one with the least wall around it, and how much that is.

        `robot` and `min_distance` narrow the candidates first. A frontier that runs around the robot — the
        shape of the first minute of every mapping run, and with no wall mapped yet the rule below has
        nothing to choose by — has its best cell under the robot, where a planner can only spin. Choosing
        among the cells far enough away keeps this rule meaningful instead of quietly throwing a whole room
        away, which is what a goal 0.18 m away once did.

        Not the centroid. A clump that bends around the corner of a wall, or rings a pillar, has its middle
        *in* the wall, and a goal inside an obstacle is not a slow goal but a refused one — after which the
        perfectly good opening beside it is blacklisted as unreachable. The cell with the fewest occupied
        cells in its `AIM_WINDOW` neighbourhood is the middle of the opening, which is also where a robot
        should enter a room it has never seen. Ties go to the cell nearest the clump's centroid.

        Every question below is asked of all the clump's cells at once rather than cell by cell, because the
        clump of a frontier that rings a large unknown region is tens of thousands of cells and this runs
        twice a second: `walls` is the window sum of the integral image for the whole clump in four array
        lookups, and `np.lexsort` is the two-line version of "least wall, ties nearest the middle". The rule
        is the one in the paragraph above; only the asking is different.

        The wall count comes back with the cell because the ranking wants the same number: what says "aim
        here" also says "this doorway is wide", and measuring it twice would be two answers to one question.
        """
        cells = np.asarray(clump)                               # (n, 2): the clump's rows and columns
        walls = window_sum(integral, cells[:, 0], cells[:, 1])
        if robot is not None and min_distance > 0.0:
            x, y = self.metres_all(cells)
            far_enough = np.hypot(x - robot[0], y - robot[1]) >= min_distance   # np.hypot: this one is asked
                                                                            # of a whole clump at once
            if far_enough.any():            # all of it underfoot: let the whole clump stay and be rejected below
                cells, walls = cells[far_enough], walls[far_enough]
        middle = cells.mean(axis=0)
        towards_middle = abs(cells[:, 0] - middle[0]) + abs(cells[:, 1] - middle[1])
        best = np.lexsort((towards_middle, walls))[0]           # least wall first, then nearest the middle
        return (int(cells[best][0]), int(cells[best][1])), int(walls[best])

    def _wall_counter(self):
        """The integral image behind `window_sum`: how many wall cells surround a cell, in four lookups.

        Built once per `frontiers` call because every clump of that call asks for the same numbers. Padded
        with walls, not with open space: past the edge of a growing map nothing has been validated, so a
        cell on that edge must not look like the widest option there is.
        """
        sums = np.pad(self.cells == OCCUPIED, AIM_WINDOW, constant_values=True).astype(int) \
            .cumsum(0).cumsum(1)
        total = np.zeros((sums.shape[0] + 1, sums.shape[1] + 1), dtype=int)
        total[1:, 1:] = sums
        return total

    def metres_all(self, cells) -> tuple:
        """Metres for a whole set of cells at once — the same arithmetic as `metres`, asked in bulk."""
        return (self.origin[0] + (cells[:, 1] + 0.5) * self.res,
                self.origin[1] + (cells[:, 0] + 0.5) * self.res)

    def _orientation(self, robot: tuple, x: float, y: float, walls: int) -> float:
        """How much this frontier is worth driving at *now*, as 0 … 1: how little wall is around the cell
        aimed at, and how far that cell lies in the direction the robot is already looking.

        The two halves are the two ways a frontier is awkward. A goal in a narrow crack has to be approached
        slowly and may need a spin to get out of it again; a goal behind the robot needs a turn before it
        needs a planner, and a robot that has to turn first behaves differently in a spawn pocket than in the
        middle of a hall — which is the difference the lecture is about.

        A caller whose pose carries no heading gets the wall half alone. Guessing "straight ahead" for a
        heading nobody gave would have the ranking invent a fact.
        """
        open_space = 1.0 - min(walls, WINDOW_CELLS) / WINDOW_CELLS
        if robot is None or len(robot) < 3:
            return open_space
        straight_ahead = (1.0 + cos(atan2(y - robot[1], x - robot[0]) - robot[2])) / 2.0
        return (open_space + straight_ahead) / 2.0

    def _gap(self, rc: tuple, robot: tuple) -> float:
        x, y = self.metres(rc[0], rc[1])
        return hypot(x - robot[0], y - robot[1])

    def frontiers(self, robot: tuple, min_cells: int = 12, min_distance: float = 0.45,
                  avoid: list | None = None, avoid_radius: float = 0.75,
                  weights: Weights = DEFAULT_WEIGHTS) -> list:
        """Every clump worth driving to, best first, ranked by `weights`.

        `min_cells` is what tells a crack between two beams from a room. `min_distance` keeps a goal from
        being under the robot, where a planner only makes it spin. `avoid` is the blacklist of points that
        did not work out, and `avoid_radius` says how much around each is written off with it.

        The ranking has to divide before it adds: cells, a wall ratio and metres do not sum. Each quantity
        goes by a threshold of the caller's own — `min_cells` for size, `min_distance` for nearness, so that
        one unit of a weight is "a clump just worth driving to" and "a goal as close as one is allowed to
        be" — which leaves three numbers the weights can trade off, keeps the ranking the same when the whole
        hall is scaled, and keeps a score comparable with a score from an earlier map. That last part is what
        `frontier_node.keep_or_switch` needs: it judges a candidate found now against a goal chosen on an
        earlier tick, and a score normalised against "the best of this call" would not mean the same thing in
        both. `cells / metres`, which stood here before, is this trade-off with both weights stuck at one.
        """
        clumps, integral = self._survey()
        found = []
        for clump in clumps:
            if len(clump) < min_cells:
                continue                                        # a crack, not a room
            (row, col), around = self._aim(clump, robot, min_distance, integral)
            x, y = self.metres(row, col)
            distance = hypot(x - robot[0], y - robot[1])
            if distance < min_distance:
                continue
            if any(hypot(x - a[0], y - a[1]) < avoid_radius for a in avoid or []):
                continue
            # nose-first: a goal approached backwards means the first new scan looks where it came from
            found.append(Frontier(x=x, y=y, heading=atan2(y - robot[1], x - robot[0]),
                                  cells=len(clump), distance=distance,
                                  score=weights.size * len(clump) / max(min_cells, 1)
                                  + weights.orientation * self._orientation(robot, x, y, around)
                                  + weights.distance * min_distance / distance))
        return sorted(found, key=lambda f: -f.score)

    def walk_out(self, robot: tuple, reach: float = 4.0, avoid: list | None = None,
                 avoid_radius: float = 0.75) -> Frontier | None:
        """The farthest cell the map calls free, no further away than `reach` — a goal for the first minute.

        A frontier is only a goal when it is out of reach of the robot's own wheels. At the start of a
        mapping run that is exactly when there is nothing to choose: the mapper has painted a metre of floor
        around the robot, every frontier cell lies inside it, and a planner asked for a goal half a metre
        ahead makes the robot spin where it stands — measured: no goal left the node at all, so the robot
        never moved and the map never grew, which is how a deadlock looks from the outside.

        So the first goal is not the interesting place but any place the map already says the robot may go.
        Unknown cells are not candidates, which is what keeps this out of the walls, and the blacklist is
        honoured, so a robot that was written off a spot does not walk back into it.
        """
        rows, cols = np.nonzero(self.cells == FREE)
        if not len(rows):
            return None
        x = self.origin[0] + (cols + 0.5) * self.res
        y = self.origin[1] + (rows + 0.5) * self.res
        distance = np.hypot(x - robot[0], y - robot[1])
        order = np.argsort(-distance)
        for n in order:
            if distance[n] > reach:                           # sorted: everything after this is further too
                continue
            if any(hypot(x[n] - a[0], y[n] - a[1]) < avoid_radius for a in avoid or []):
                continue
            return Frontier(x=float(x[n]), y=float(y[n]),
                            heading=atan2(y[n] - robot[1], x[n] - robot[0]),
                            cells=0, distance=float(distance[n]),
                            score=0.0)          # scoreless on purpose: a real frontier takes it over, see
                                                # frontier_node.keep_or_switch — this place is a step to take,
                                                # not a place worth driving across a hall for
        return None
