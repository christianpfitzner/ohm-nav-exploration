# Frontier-based exploration, on a student's level

The robot starts in a hall it has never seen, with a lidar and an odometry it should not fully trust. It
has no map, no route and no idea where the doors are. It has to leave nothing out and it has to move while
it still knows almost nothing. This is the whole problem, and one paper solved it in about a page:
*frontier mapping* (Yamauchi, 1997 — see [the algorithm in the
paper](#the-algorithm-in-the-paper) at the end).

Everything below is what that idea looks like once it has to survive a real mapper, a real planner and a
simulator that reports a missing echo as a range.

## The map says three things, not two

An occupancy grid is one byte per cell, and a cell can say three different things:

| the cell is | slam_toolbox writes | what it means |
| --- | --- | --- |
| free | `0` | the lidar looked through here and nothing was there |
| occupied | `100` (this repo draws the line at `50`) | something stopped the beam here |
| unknown | `-1` | nothing has looked here yet — the space may be a room or a wall |

**Unknown is not free, and it is not occupied either.** This is where every first implementation goes
wrong, twice over: `-1` is truthy in Python, and "not free" sounds like "wall". The naive rule

```python
frontier = free cell with a neighbour that is not free       # wrong
```

does not find doorways, it finds *every wall in the building*, and it keeps finding them: after the first
room is mapped there is always a free cell next to a wall, so the robot has an endless supply of goals and
never runs out of work that leads nowhere. Here is a room with one doorway and nothing mapped beyond it —
`#` wall, `.` floor, `?` unknown, `R` the robot:

```
? ? ? ? ? ? ? ? ? ?
? ? ? ? ? ? ? ? ? ?
? ? ? ? ? ? ? ? ? ?
# # # G F # # ? ? ?
# . . . . . # ? ? ?
# R . . . . # ? ? ?
# # # # # # # ? ? ?
```

The correct rule — a free cell with an **unknown** cell in its 8-neighbourhood — hands back **2** cells
here, both of them in the doorway. The naive rule hands back **12**, all of them along walls of a room that
is already mapped. Those two numbers are the whole difference between a robot that explores and a robot
that jitters along its own walls.

One subtlety is worth knowing before it surprises you: the **border of the grid is unknown**, not wall. A
SLAM map covers only what has been seen, so the far side of a room you have looked into from one side is
still unexplored floor, sitting right at the edge of the map. Treating the grid border as a wall loses
exactly the frontiers you most want.

## A frontier is a clump of cells, not a cell

One frontier cell is a gap between two laser beams. Twenty of them together are a doorway nobody has gone
through. So the free-and-next-to-unknown cells are grouped into connected clumps
([`_clumps`](../ohm_frontier/ohm_frontier/frontiers.py)), and a clump under `min_frontier_cells` is
thrown away as a crack.

`min_frontier_cells` is a real number, not a fudge factor: the mapper's cells are 0.05 m
([`config/slam_toolbox.yaml`](../ohm_frontier/config/slam_toolbox.yaml)), so a 1.00 m doorway is a clump of
**20** cells and a 15 cm gap between two beams is 3. The shipped floor of 12 sits between the two.

## Aim at the opening, not at the middle of the clump

The obvious thing to aim at is the clump's centroid. In an L-shaped clump around a corner, or a clump
ringing a pillar, the centroid is **in the wall** — and a goal inside an obstacle is not a slow goal, it is
a refused one, after which the perfectly good opening next to it gets written off as unreachable.

So this repo picks the cell of the clump with the least wall around it, inside a window of
`AIM_WINDOW = 3` cells (0.15 m at the mapper's resolution), ties going to the cell nearest the centroid
([`_aim`](../ohm_frontier/ohm_frontier/frontiers.py)). In the map above that is the `G` cell, the middle of
the doorway — which is also where you would walk into a room you have never seen.

Two more rules protect the goal itself:

* `min_goal_distance` (0.7 m): a goal under the robot's own wheels is not a goal, it is a spin. Below about
  half a metre there is a second problem — the mapper only integrates a scan once the robot has moved
  0.2 m, so a robot shuffling 0.2 m at a time never gives it the reason to.
* The heading is pointed **away from the robot**, along the line from robot to goal. A goal approached
  backwards means the first new scan looks at where you came from instead of into the unknown.

## Which frontier first: cells per metre

With several frontiers, rank them by **clump size divided by distance**
([`frontiers`](../ohm_frontier/ohm_frontier/frontiers.py)): what you get (unexplored boundary, which is
what you will actually see next) per what you pay (metres of driving). A mapped room with a 1.0 m opening
far away and a small crack nearby, robot at (1.0, 1.0), measured with the code:

| clump | cells | distance | score = cells / metres |
| --- | --- | --- | --- |
| the wide opening at y = 3.5 … 5.0 | 3 | 4.60 m | **0.65** |
| the crack at y = 1.0 … 2.0 | 2 | 3.34 m | 0.60 |

The wide opening wins although it is 1.3 m further on — which is the behaviour you want, and which a
"nearest frontier first" rule gets exactly backwards: the nearest frontier is nearly always a crack in the
wall the robot has already touched.

Size and distance are the two terms every implementation has. Orientation — how well the opening faces the
robot's heading — is the third one a lecture can add, and it belongs in the score and nowhere else: the
frontier rule, the aim and the blacklist should not have to know that a weight changed.

## When the goal cannot be reached, which it often can't

A frontier is a **direction**, not a place with a route to it. The cell is free and the cell beyond it is
unknown, so nothing proves a path exists: the doorway may be blocked by a table the planner cannot pass. A
node that does not handle this asks for the impossible forever. Four things are therefore in
[`frontier_node.py`](../ohm_frontier/ohm_frontier/frontier_node.py):

* **A refusal is counted, not believed.** The navigation stack answers `not accepted` within about a
  millisecond when `bt_navigator` is still activating, and a robot started together with its stack always
  hits that. So a place has to be refused **three** times before it is written off.
* **A blacklist, with radius.** A goal that arrived nowhere, or was written off, puts its position on
  `avoid` and `avoid_radius` (0.75 m) takes the surroundings with it — otherwise the next clump is chosen
  two cells along the same blocked doorway and the robot does the same trip again.
* **A stall check.** A goal whose nearest approach stops improving (`stall_distance` within `stall_s`) and
  a goal that has simply taken too long (`patience_s`) are both given up on.
* **The stack's own verdict is read.** `NavigateToPose` reports `aborted` or `canceled`, and the code reads
  the number out of the status message — comparing a `GoalStatus` **message** to an integer is true for
  every goal including an arrived one, which would blacklist every place the robot ever reached.

## When there is no frontier at all

At the start of a run there is nothing to choose: the mapper has painted about a metre of floor around the
robot and every frontier cell on that map lies inside `min_goal_distance`.
[`walk_out`](../ohm_frontier/ohm_frontier/frontiers.py) does what the name says — drive to the farthest
cell the map calls **free**, never into an unknown one —
because a mapper needs the robot to move before it integrates a scan. That is not a detail: standing at the
spawn point the map holds 1 414 free cells and 2 walls; after 20 s of driving the same map holds 18 022
free cells and 767. A mapper that has not been given a reason to integrate a scan is not broken, it is
waiting.

When there is genuinely nothing left — no clump over `min_frontier_cells`, none outside the blacklist, none
beyond `walk_out_reach` — the node says so **once** and stops. Saying it twice a second for the rest of the
run is what an "exploration finished" screen looks like when nobody has thought about the end of the run.

## The algorithm in the paper

Brian Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, Proc. IEEE International Symposium
on Computational Intelligence in Robotics and Automation (CIRA'97), pp. 146–151, 1997 —
<https://doi.org/10.1109/CIRA.1997.613851> (resolves to IEEE Xplore, checked from this checkout).

The companion paper the idea is often cited under is Brian Yamauchi, *Frontier Mapping and Exploration as an
Adaptive Process*, Proc. IEEE International Conference on Systems, Man and Cybernetics, San Diego, 1997. Its
DOI and page numbers are deliberately **not** given here: neither Crossref nor OpenAlex resolved that paper
from this machine, and a guessed identifier in a handout is worse than none. Track it down in the library,
not in a chat window.

| Yamauchi's step | where it lives here |
| --- | --- |
| build an occupancy grid from the sensors | not ours: slam_toolbox, configured in [`config/slam_toolbox.yaml`](../ohm_frontier/config/slam_toolbox.yaml); the node subscribes `/map` and publishes no map of its own |
| find frontier cells: free next to unknown | `Grid._against_unknown` in [`frontiers.py`](../ohm_frontier/ohm_frontier/frontiers.py) |
| group them, discard what is too small to be a doorway | `Grid._clumps` and `min_frontier_cells` |
| choose the frontier to explore by some utility | `Grid.frontiers`, score = cells / metres, sorted best-first |
| navigate to it | `FrontierNode.publish` sends a `nav2_msgs/action/NavigateToPose` goal; `/goal_pose` is RViz's button and no nav2 node subscribes to it, which is the mistake the paper's "send it to the navigator" hides |
| when a frontier is unreachable, drop it and pick another | `FrontierNode.give_up`, `avoid` / `avoid_radius`, `stall_s`, `patience_s` |
| stop when no frontiers remain | `seek_goal`'s empty case, said once (`said_empty`) |
| — nothing in the paper covers this | `Grid.walk_out`: the first minute, when the map is smaller than `min_goal_distance`, has no frontier to pick at all |

Two further readings, both checked:

* **The shipped behaviour tree**, `/opt/ros/kilted/share/nav2_bt_navigator/behavior_trees/`
  (`navigate_to_pose_w_replanning_and_recovery.xml`). This is what actually happens between "goal sent" and
  "goal reached" — where the planner is asked again, where `Spin` and `Backup` are tried, and where
  `distance_travelled` decides that progress has stalled. It answers the question this repo can only ask:
  who gave up on my goal first.
* **`/opt/ros/kilted/share/nav2_msgs/action/NavigateToPose.action`** and
  **`/opt/ros/kilted/share/slam_toolbox/config/mapper_params_online_async.yaml`** on any lab machine. The
  first is the interface the frontier node writes to, including the `error_code` values and the
  `number_of_recoveries` feedback field; the second is what the mapper's defaults are before this repo
  changes them, and it is the fastest way to see which of this repo's parameters are corrections rather
  than tastes.
