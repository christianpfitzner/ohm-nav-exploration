"""Frontiers in a map somebody else made, and which of them to drive to first.

Nothing in here imports ROS. The map arrives as the payload of a `nav_msgs/msg/OccupancyGrid` — what
slam_toolbox publishes — which is also what a test can draw by hand.

The idea, in the order it is used:

* **Frontier** — a *free* cell with an *unknown* cell in its 8-neighbourhood: known floor that leads into
  territory the lidar has not seen. One such cell is a crack between two beams; forty of them are a room
  nobody has entered yet.
* **Aim at the opening, not at the clump** — see `_aim`. The middle of a clump that bends around a corner
  stands in that corner, and a goal inside a wall is refused by every planner.
* **Rank by size per unit distance** — `cells / metres`, see `frontiers`: the wide opening down the hall
  beats the crack beside the wheel, and no hall of a certain size is needed for that to hold.
* **Blacklist** — see `frontier_node`. A goal that was refused or never reached writes off its
  surroundings, so the next frontier gets its turn instead of being asked forever.

About `unknown`, where the subtleties are. A map says one of three things about every cell — free,
occupied, unknown — and **unknown is not free**. slam_toolbox writes it as `-1`, and the edge of its grid
is unknown as well, because nothing has been seen there. A frontier rule of "free next to not-free"
cannot tell those apart and stops finding rooms after the first minute.
"""
from collections import namedtuple
from math import atan2, cos, hypot, sin

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
AIM_WINDOW = 3          # cells around a goal that are counted for wall proximity (0.15–0.3 m)
NEIGHBOURS = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)]

Frontier = namedtuple("Frontier", "x y heading cells distance score")


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

    def metres(self, row: int, col: int) -> tuple:
        return (self.origin[0] + (col + 0.5) * self.res, self.origin[1] + (row + 0.5) * self.res)

    def _paint(self, x: float, y: float, value: int) -> None:
        """One named cell — so a test can draw a hall without a robot or a lidar."""
        col, row = int(x / self.res), int(y / self.res)
        if 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]:
            self.cells[row, col] = value

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
        """Connected frontier cells, as clumps of at least `min_cells`."""
        pending, taken, groups = np.argwhere(edge), np.zeros_like(edge, dtype=bool), []
        for start in pending:
            if taken[start[0], start[1]]:
                continue
            taken[start[0], start[1]] = True
            stack, clump = [tuple(start)], []
            while stack:                                        # flood fill with an explicit stack
                row, col = stack.pop()
                clump.append((row, col))
                for dr, dc in NEIGHBOURS:
                    r, c = row + dr, col + dc
                    if 0 <= r < edge.shape[0] and 0 <= c < edge.shape[1] and edge[r, c] \
                            and not taken[r, c]:
                        taken[r, c] = True
                        stack.append((r, c))
            groups.append(clump)
        return groups

    def _aim(self, clump, robot: tuple = None, min_distance: float = 0.0):
        """Which cell of a clump to drive at: the one with the least wall around it.

        `robot` and `min_distance` narrow the candidates first. A frontier that runs around the robot — the
        shape of the first minute of every mapping run, and with no wall mapped yet the rule below has
        nothing to choose by — has its best cell under the robot, where a planner can only spin. Choosing
        among the cells far enough away keeps this rule meaningful instead of quietly throwing a whole room
        away, which is what a goal 0.18 m away once did.

        Not the centroid. A clump that bends around the corner of a wall, or rings a pillar, has its
        middle *in* the wall, and a goal inside an obstacle is not a slow goal but a refused one — after
        which the perfectly good opening beside it is blacklisted as unreachable. The cell with the fewest
        occupied cells in its `AIM_WINDOW` neighbourhood is the middle of the opening, which is also where
        a robot should enter a room it has never seen. Ties go to the cell nearest the clump's centroid.
        """
        # Padded with walls, not with open space: past the edge of a growing map nothing is validated, so
        # a cell on that edge should not look like the widest option there is.
        sums = np.pad(self.cells == OCCUPIED, AIM_WINDOW, constant_values=True).astype(int) \
            .cumsum(0).cumsum(1)
        total = np.zeros((sums.shape[0] + 1, sums.shape[1] + 1), dtype=int)
        total[1:, 1:] = sums

        def nearby(row, col):                 # integral image: the window sum in four lookups, no loop
            span = 2 * AIM_WINDOW
            return (total[row + span, col + span] - total[row, col + span]
                    - total[row + span, col] + total[row, col])

        reachable = [rc for rc in clump if self._gap(rc, robot) >= min_distance] if robot is not None \
            else clump
        candidates = reachable or clump                 # if all of it is underfoot, let it be rejected below
        middle = np.array(candidates).mean(axis=0)
        return min(candidates, key=lambda rc: (nearby(*rc), abs(rc[0] - middle[0])
                                               + abs(rc[1] - middle[1])))

    def _gap(self, rc: tuple, robot: tuple) -> float:
        x, y = self.metres(rc[0], rc[1])
        return hypot(x - robot[0], y - robot[1])

    def frontiers(self, robot: tuple, min_cells: int = 12, min_distance: float = 0.45,
                  avoid: list | None = None, avoid_radius: float = 0.75) -> list:
        """Every clump worth driving to, best first.

        `min_cells` is what tells a crack between two beams from a room. `min_distance` keeps a goal from
        being under the robot, where a planner only makes it spin. `avoid` is the blacklist of points that
        did not work out, and `avoid_radius` says how much around each is written off with it.
        """
        edge = self._against_unknown()
        found = []
        for clump in self._clumps(edge):
            if len(clump) < min_cells:
                continue                                        # a crack, not a room
            row, col = self._aim(clump, robot, min_distance)
            x, y = self.metres(row, col)
            distance = hypot(x - robot[0], y - robot[1])
            if distance < min_distance:
                continue
            if any(hypot(x - a[0], y - a[1]) < avoid_radius for a in avoid or []):
                continue
            # nose-first: a goal approached backwards means the first new scan looks where it came from
            found.append(Frontier(x=x, y=y, heading=atan2(y - robot[1], x - robot[0]),
                                  cells=len(clump), distance=distance,
                                  score=len(clump) / max(distance, 0.1)))
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
                            cells=0, distance=float(distance[n]), score=0.0)
        return None
