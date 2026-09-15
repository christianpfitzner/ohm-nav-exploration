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

## Which frontier first: three weights, one score

With several frontiers there has to be a ranking, and this one is a weighted sum of three quantities
([`Grid.frontiers`](../ohm_frontier/ohm_frontier/frontiers.py)) — **size**, **orientation**, **nearness** —
each divided by a threshold of the caller's own first, because cells, a wall ratio and metres do not add:

| the term | what it is divided by | so one unit of that weight means |
| --- | --- | --- |
| `weight_size` × cells in the clump | `min_frontier_cells` | "a clump just worth driving to" |
| `weight_orientation` × how open and how far ahead the goal is | nothing — it is already 0 … 1 | "entering nose-first, with no wall around the goal" |
| `weight_distance` × `min_goal_distance` / distance | nothing — it is a ratio | "a goal as close as one is allowed to be" |

Dividing first is not decoration. A score normalised against "the best candidate this call happened to find"
would not mean the same thing on two maps or on two ticks, and `keep_or_switch` has to judge a candidate found
now against a goal taken a minute ago.

Measured on the map the tests draw (`two_doorways`: a half-mapped room, a 3-cell opening far away, a 2-cell
crack near, the robot at (0.5, 1.5); `min_cells=2` because the drawn openings are small — a real doorway is 20
cells and the shipped floor is 12):

| clump | cells | distance | score, the shipped weights | score, orientation off | score, robot turned round |
| --- | --- | --- | --- | --- | --- |
| the wide opening | 3 | 4.65 m | **1.82** | **1.60** | **1.72** |
| the crack | 2 | 3.76 m | 1.35 | 1.12 | 1.22 |

The wide opening wins although it is 0.9 m further on, which is the behaviour you want and the one a "nearest
frontier first" rule gets exactly backwards: the nearest frontier is nearly always a crack in a wall the robot
has already touched. `cells / metres`, the ranking this repo shipped first, is the middle column with both
weights stuck at one — it is still reachable, as the case `weight_orientation = 0`.

The last column is the orientation term working: the same map, the same clumps, the robot turned to look over
its own shoulder, and the numbers change although nothing on the map did. That is the third term's whole job —
entering a doorway nose-first means the first new scan looks into the unknown instead of at the room you have
already mapped — and its default of 0.25 is deliberately small: nothing measured here says facing outweighs a
room, and a default that silently prefers the frontier straight ahead leaves a robot circling one doorway.

All three are declared parameters with a description and a range of 0 … 10. `ros2 param describe
/frontier_node weight_distance` shows what a node declares; `ros2 param set /frontier_node weight_orientation
0` takes effect on the next decision, without a restart, because the node answers parameter changes
(`weights_changed`). A declared bounded double is also what a tuning panel turns into a slider —
`rqt_reconfigure` is not installed on this machine, so nobody has checked that it lists a plain rclpy node
here, and nothing in this package depends on it: the two commands above and a parameter file do the same job.
The ranges are guards as well: a weight
that rewarded a frontier for being far away is not a ranking, and rclpy refuses the negative number.

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
* **A clock that runs on progress, not on the calendar.** A goal is written off once the robot has not come
  `progress_distance` (0.25 m) nearer to it within `goal_timeout_s` (10 s). Ten seconds of standing still is
  the measured failure at the spawn pocket — the controller took the goal and published nothing on any
  `cmd_vel` topic for 18 s — and ten seconds without approaching is also what a frontier on the far side of a
  wall looks like from here. It is deliberately not a deadline for the whole drive: a frontier at the end of a
  hall is perfectly reachable and takes far longer, and what restarts the clock is any approach of
  `progress_distance` past the nearest gap so far. The version before this one had `stall_s` (25 s) and
  `patience_s` (120 s), two calendar deadlines, which between them amounted to blacklisting a whole hall
  inside two minutes; both parameters are gone.
* **The stack's own verdict is read.** `NavigateToPose` reports `aborted` or `canceled`, and the code reads
  the number out of the status message — comparing a `GoalStatus` **message** to an integer is true for
  every goal including an arrived one, which would blacklist every place the robot ever reached.

## Committing to a goal, which is a different bug from choosing badly

Choosing the right frontier is half of it. The other half is *keeping* the choice, and the first version of
this node got it wrong in a way that is invisible on a static map and obvious on a live one: the goal was
chosen in the same timer tick that looked at the map, so **the timer period was the re-decision period**. At
`period = 0.5 s`, with a map growing under the robot's own wheels, that handed over a new winner several times
a minute — the robot drove a metre towards one doorway, then a metre towards the next, and arrived at neither.
The symptom on a live run was a frontier that flipped every second and a robot that never reached one.

What is in [`frontier_node.py`](../ohm_frontier/ohm_frontier/frontier_node.py) now:

* **A goal stands until it ends**: reached, or no nearer for `goal_timeout_s`, or refused by the stack. See
  `watch_the_current_goal`.
* **Changing your mind has a price.** A fresh candidate takes over only by beating the score the live goal was
  *taken on* — not the best the map offers now — by `reselect_margin` (1.5). A margin of 1.0 would bring the
  bug straight back, so the code reads the parameter as at least 1.0 rather than trusting whoever set it.
* **The same doorway seen again is not news.** `frontiers` re-aims at a clump as the map grows, so a candidate
  within one cell resolution of the live goal is the same place, not a rival (`keep_or_switch`).
* **A dropped goal is cancelled, not forgotten.** `bt_navigator` serves one goal at a time; a node that only
  forgets its goal leaves the stack driving to the place it forgot, and the stack's answer about that drive
  arrives while a *different* goal is current. Every goal therefore carries an attempt number, a late word
  about an older one is ignored, and `drop_goal` sends the cancel.
* **A switch nobody failed at is not written off.** Only a reached, stalled or refused goal adds to `avoid`;
  switching away from a frontier that was never tried leaves it on the table.

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

## Every parameter the node declares

Defaults as declared in [`frontier_node.py`](../ohm_frontier/ohm_frontier/frontier_node.py). The four that
change behaviour most are also in the README.

| parameter | default | what it is for |
| --- | --- | --- |
| `robot` | `muster` | the name the odom, scan and cmd_vel topics are prefixed with |
| `map_topic`, `goal_topic`, `markers_topic`, `action_topic` | `/map`, `/frontier_goal`, `/frontiers`, `/navigate_to_pose` | the four interfaces, named rather than hardcoded, because a run with two robots needs them moved |
| `period` | 0.5 s | how often the map is looked over — and, since a goal is kept, nothing else |
| `min_frontier_cells` | 12 | a clump under this is a gap between two beams, not a room |
| `min_goal_distance` | 0.7 m | a goal nearer than this is under the robot's own wheels |
| `walk_out_reach` | 4.0 m | how far the filler goal may be while nothing on the map is a frontier yet |
| `avoid_radius` | 0.75 m | how much of the map a written-off goal takes with it |
| `reached_distance` | 0.35 m | when a goal counts as arrived |
| `progress_distance` | 0.25 m | how much nearer than ever before counts as making progress, which resets the timeout |
| `goal_timeout_s` | 10 s | how long without that progress before the place is written off |
| `reselect_margin` | 1.5 (never below 1.0) | how much better a candidate must be to take over a live goal |
| `weight_size`, `weight_orientation`, `weight_distance` | 1.0, 0.25, 1.0 — range 0 … 10 | the ranking, live: see *Which frontier first* above |

## The simulator's two quirks

Both are handled by [`explore.launch.py`](../ohm_frontier/launch/explore.launch.py), and both will cost an
afternoon if they are not written down, because neither looks like a bug in this repo.

**Who publishes the top edge of the tf tree.**

```
map  →  muster/odom  →  muster/base_link  →  muster/laser
 slam         sim              sim                 sim
```

The simulator publishes `map → <robot>/odom` as well by default — a drifting odometry being the second parent
is what its Kalman lab is built on — and two publishers on that one edge give `<robot>/odom` two parents, which
is not a tree. nav2's costmap answers that with `frame does not exist` rather than with a map. So the launch
file asks for `tf_tree:=slam` (`--set tf.tree=slam`, implemented in `mecanum_lab/tf_bcast.py`), after which the
simulator calls the frame of its own hall coordinates `hall` instead of `map` — a mapper anchors `map` wherever
its first scan found the robot, which is not the corner of the hall that GPS and truth positions are measured
from. What plans and draws is the frame named in the map message, which this node reads out of the message
instead of assuming any of this.

**The lidar's missing echo.** The simulator reports a beam that did not come back as the laser's own range,
8.0 m, which is what every laboratory there measures. slam_toolbox cannot use that: a reading at its
`max_laser_range` is not "the space beyond is open" — it is the mapper being told the hall ends there. So the
launch file also passes `lidar_no_echo:=inf` (`--set sensor.lidar.no_echo=inf`, `mecanum_lab/ros_bridge.py`),
which is what `sensor_msgs/msg/LaserScan` documents and what the mapper expects.

## And a third thing, which is in the launch files, not in the simulator

`IncludeLaunchDescription` does not scope the arguments it passes. An included launch file declares its
arguments in **the including file's** configuration space, so a name used by both is the included file's by the
time the including file reads it again:

```python
DeclareLaunchArgument("rviz", default_value="true")          # ours: "open the view"
IncludeLaunchDescription(lab_launch, launch_arguments={"rviz": "false", ...})   # the lab's: "no Pygame-adjacent viewer"
# …and an OpaqueFunction placed after the include, reading `rviz`, is told 'false'.
# It is told '' for `robots` too, which nobody mentioned at all.
```

Measured that way, with a three-entity launch file that prints the value before the include and after it: `true`,
then `false`. `lab.launch.py` declares nineteen arguments (`BASICS`, in `mecanum-lab/launch/lab.launch.py`) and
five of them — `world`, `robot`, `headless`, `use_sim_time`, `rviz` — mean something to this package too. Four of
those five mean the same thing to both files, so the collision is invisible. `rviz` does not: theirs is a viewer
beside the simulator's window, ours is the frontier view, and it is the argument the whole decision is made of.

So both launch files here that open a view put that decision **before** the include, and
`test_launch_files.py::test_the_view_is_decided_before_the_simulator_is_allowed_to_answer_for_its_name` keeps
them there. Worth the ceremony because of the failure mode: with the reading after the include, `rviz:=true`
starts no viewer, prints no error, and leaves the screen to the simulator's window — the one class of launch bug
that cannot be diagnosed from the log, because there is nothing in it. If a student writes their own launch file
that includes the simulator and wonders why their argument stopped working, this is it, and `ros2 launch --show-args`
shows both files' arguments in one list, which is the same fact seen from the outside.

## The algorithm in the paper

Brian Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, Proc. 1997 IEEE International
Symposium on Computational Intelligence in Robotics and Automation (CIRA'97), pp. 146–151,
<https://doi.org/10.1109/CIRA.1997.613851>. Checked against Crossref, which carries that title, venue, page
range and DOI, and the DOI resolves to IEEE Xplore document 613851 (paywalled). The same author's *Frontier
Mapping and Exploration as an Adaptive Process* (SMC'97) is the companion the idea is often cited under; its
DOI and page range are deliberately **not** given here, because neither Crossref nor OpenAlex returned that
record from this machine, and a guessed identifier in a handout is worse than none. Track it down in the
library, not in a chat window.

| Yamauchi's step | where it lives here |
| --- | --- |
| build an occupancy grid from the sensors | not ours: slam_toolbox, configured in [`config/slam_toolbox.yaml`](../ohm_frontier/config/slam_toolbox.yaml); the node subscribes `/map` and publishes no map of its own |
| find frontier cells: free next to unknown | `Grid._against_unknown` in [`frontiers.py`](../ohm_frontier/ohm_frontier/frontiers.py) |
| group them, discard what is too small to be a doorway | `Grid._clumps` and `min_frontier_cells` |
| choose the frontier to explore by some utility | `Grid.frontiers`: `weight_size × cells/min_frontier_cells + weight_orientation × openness-and-facing + weight_distance × min_goal_distance/distance`, sorted best-first; `Grid._aim` picks the cell inside the clump and `Grid._orientation` the facing term |
| navigate to it | `FrontierNode.publish` sends a `nav2_msgs/action/NavigateToPose` goal; `/goal_pose` is RViz's button and no nav2 node subscribes to it, which is the mistake a step worded as "hand it to the navigator" hides — Xplore is paywalled from here, so this table paraphrases the paper and quotes none of its wording |
| when a frontier is unreachable, drop it and pick another | `FrontierNode.give_up` (three refusals, because one refusal while `bt_navigator` activates means nothing), `watch_the_current_goal` (`progress_distance` within `goal_timeout_s`), `avoid` with `avoid_radius`, and `drop_goal`, which cancels at the stack instead of only forgetting |
| keep the plan instead of recomputing it every tick | — the paper does not cover it, and this repo only learned it by watching a robot fail to arrive: `keep_or_switch` and `reselect_margin`, see *Committing to a goal* |
| stop when no frontiers remain | `FrontierNode.candidates`' empty case, reported once (`said_empty`) rather than twice a second |
| — nothing in the paper covers this either | `Grid.walk_out`: the first minute, when the map is smaller than `min_goal_distance`, has no frontier to pick at all |

Two further readings, both checked:

* **The shipped behaviour tree**, `/opt/ros/kilted/share/nav2_bt_navigator/behavior_trees/`
  (`navigate_to_pose_w_replanning_and_recovery.xml`), read here on ROS 2 Kilted. This is what actually happens
  between "goal sent" and "goal reached": the planner is asked again on a `RateController hz="1.0"`, and the
  recovery round is a `RecoveryNode number_of_retries="6"` around a `RoundRobin` of `Spin` (`spin_dist="1.57"`),
  `BackUp` (`backup_dist="0.30"`) and `Wait` (`wait_duration="5.0"`). Worth noticing while reading it: there is
  **no progress or distance term in that tree at all** — nothing in it decides that a goal has stopped being
  approached. It answers the question this repo can only ask from the outside (who gave up on my goal first),
  and it is also why `goal_timeout_s` exists in this node rather than being nav2's job.
* **`/opt/ros/kilted/share/nav2_msgs/action/NavigateToPose.action`** and
  **`/opt/ros/kilted/share/slam_toolbox/config/mapper_params_online_async.yaml`** on any lab machine. The
  first is the interface the frontier node writes to, including the `error_code` values and the
  `number_of_recoveries` feedback field; the second is what the mapper's defaults are before this repo
  changes them, and it is the fastest way to see which of this repo's parameters are corrections rather
  than tastes.
