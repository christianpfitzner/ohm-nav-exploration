# ohm-nav-exploration

Frontier-based exploration on top of the [mecanum-lab](../mecanum-lab) simulator: one ROS 2 Python package
that reads the map somebody else is making, decides which unexplored place to drive to next, and hands that
place to the navigation stack. Mapping is slam_toolbox's job, driving is nav2's, the simulator is used as it
is.

**[docs/frontier-exploration.md](docs/frontier-exploration.md)** explains the algorithm on a student's
level — what a frontier is, why unknown is neither free nor occupied, how candidates are ranked, what the
Yamauchi paper says and where each of its steps lives in this code. This file is only how to run it.

```
ohm_frontier/
  ohm_frontier/frontiers.py       the map and the frontier rules — no ROS in here
  ohm_frontier/frontier_node.py   the node: /map and /odom in, a NavigateToPose goal and the frontiers out
  ohm_frontier/wall_following.py      \
  ohm_frontier/obstacle_avoidance.py   > three reactive demos: a lidar and a rule, no map, no planner
  ohm_frontier/turn_and_move.py      /
  launch/explore.launch.py        simulator + slam_toolbox + nav2 + RViz + this node, one command
  launch/explore_<hall>.launch.py one per hall worth showing: rooms, maze, open, arena
  launch/explore_no_nav2.launch.py the same run with nothing driving — the decisions on their own
  launch/reactive_<demo>.launch.py hall + one reactive node, for the lecture
  launch/frontier.launch.py       this node alone, for when the rest is already running
  config/slam_toolbox.yaml        the mapper: cell size, when a scan is worth adding, loop closure
  config/nav2_rooms.yaml          nav2 for one simulated robot: frames, topics, costmaps
  config/explore.rviz             the frontier view: map, lidar, trajectory, plan, every frontier, the chosen one
  test/                           the frontier rules, the life of a goal, the reactive maths, the launch files
docs/frontier-exploration.md      the explanation, the paper, and the further readings
install.sh, INSTALL.md            ./install.sh --check — what a machine has, and what it lacks
```

## Needs

```bash
./install.sh --check        # what is there, what is missing, what to type about it — installs nothing
```

That is the list, tested rather than written down: python3 and numpy, a sourced ROS 2, the two launch files
this package includes (`slam_toolbox`'s and `nav2_bringup`'s), `nav2_msgs`, **`rviz2`**, and a `mecanum-lab`
checkout new enough to take `tf_tree` and `lidar_no_echo`. Its exit code names which class of thing failed.
`INSTALL.md` has the options and the two failures people actually hit.

* a checkout of `mecanum-lab` (default `~/git/mecanum-lab`, elsewhere by `sim_dir:=`) — the simulator, and
  the worlds (`rooms`, `maze`, `open`, `arena`, `production`, `track`)
* `sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
  ros-$ROS_DISTRO-rviz2`
* built and run on ROS 2 Kilted. `rviz2` is not part of the ROS base install and the default run opens the
  view, so a machine without it needs either that line or `rviz:=false`; the launch file looks for it and
  answers with what to type rather than with a process that died.

## One command

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ohm_frontier explore.launch.py
```

Five things start: the simulator in the `rooms` hall **with its window open** (`headless:=false`),
slam_toolbox mapping the lidar, nav2's navigation servers, **this package's RViz view** (`rviz:=true`,
`config/explore.rviz`, installed under `share/ohm_frontier/rviz`), and this node. Other hall, other robot:

```bash
ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo
```

`robot:=` reaches everything because the launch file rewrites every `<robot>/` in `config/*` at start: nav2
and slam_toolbox are handed a *file* and name frames and topics literally in it, and RViz is handed a file
whose displays name `/<robot>/scan` and `/<robot>/odom`, so a launch argument reaches none of them any other
way. The arguments worth knowing are `world` (rooms | maze | open | arena | production | track), `robot`,
`headless:=false`, `rviz:=false` for no view, `nav2:=false` for no navigation stack, `rviz_config:=` for
another view and `sim_dir:=` for a simulator elsewhere; `--show-args` has a sentence for each.

The simulator is never asked for its own RViz, whatever `rviz` says: two viewers with fixed frame `map` is
one too many, and the one that loses shows an empty map while the other one is right.

### One file per thing worth showing

| command | hall | what it is for |
| --- | --- | --- |
| `explore.launch.py` | `rooms` | the whole stack; the defaults above are its defaults |
| `explore_rooms.launch.py` | `rooms` | 22 × 16 m of walls **with doorways**, so every unentered room is one clump and the ranking has something to choose between |
| `explore_maze.launch.py` | `maze` | 6 × 6 m of corridor one cell wide: a frontier is a corridor that ends, so the blacklist and `min_frontier_cells` are what is on display |
| `explore_open.launch.py` | `open` | 30 × 20 m with nothing but its boundary: one frontier, nothing to rank, so the argument is about what a frontier *is*. Run it first, `rooms` second |
| `explore_arena.launch.py` | `arena` | an empty hall whose painted lanes reflect nothing back to the lidar: the map is what the sensors say, not what the world file says |
| `explore_no_nav2.launch.py` | `rooms` | `nav2:=false` in a file: no planner, no controller, the goals only on `/frontier_goal` and in RViz — the ranking, the timeout and the blacklist in seconds, with nobody to blame |

Each of those is three lines of substance over `explore.launch.py`, and each docstring says what to watch
for in that hall.

## Two things that will otherwise bite you

**Who publishes the top edge of the tf tree.**

```
map  →  muster/odom  →  muster/base_link  →  muster/laser
 slam         sim              sim                 sim
```

The simulator publishes `map → <robot>/odom` as well by default — drifting odometry being the second parent
is what its Kalman lab is built on. Two publishers on that one edge give `odom` two parents, and a tf tree
with two parents is not a tree; nav2's costmap answers that with `frame does not exist` rather than with a
map. So the launch file asks for `tf_tree:=slam` (`--set tf.tree=slam`, `mecanum_lab/tf_bcast.py`), and the
simulator then calls the frame of its own hall coordinates `hall`. What plans and draws is the frame named
in the map message, which this node reads out instead of assuming — and positions the simulator measures
from the corner of its hall (GPS, truth) are in `hall`, which is not the frame this node works in.

**The lidar's missing echo.** The simulator reports a beam that did not come back as the laser's own range,
8.0 m, which is what every laboratory there measures. slam_toolbox cannot use that: a reading at its
`max_laser_range` is not "the space beyond is open". The launch file therefore also passes
`lidar_no_echo:=inf` (`--set sensor.lidar.no_echo=inf`, `mecanum_lab/ros_bridge.py`), which is what
`sensor_msgs/msg/LaserScan` documents.

## What the node decides

* Subscribes `/map` and `<robot>/odom`, and brings the odometry pose into the map's frame before comparing
  the two — measured 0.18 m apart at spawn, which was already enough to put a goal under the robot's own
  wheels. It publishes no map: two mappers in one graph means two answers to "what does the hall look like".
* Free cells next to **unknown** cells, grouped into clumps, clumps under `min_frontier_cells` thrown away.
  Aimed at the cell with the least wall around it, not at the clump's centroid — the middle of a clump that
  bends around a corner is in the wall, and a goal in a wall is refused, after which the good opening beside
  it gets blacklisted. Ranked by **cells per metre**.
* Sends the winner to nav2 as a `nav2_msgs/action/NavigateToPose` goal, nose pointed away from where it came
  from so the first new scan looks into the unknown. **nav2 does not read a `/goal_pose` topic** — that is
  RViz's button, not an interface — so the same pose goes out on `/frontier_goal` for a display or for a
  student's own follower.
* Watches it: arrived, or no longer getting closer, or refused. One refusal is not a verdict — a goal that
  leaves while `bt_navigator` is still activating is answered `not accepted` within about a millisecond — so
  a place has to be refused three times before it and `avoid_radius` around it are written off.
* When nothing on the map is far enough away to be a goal, which at the start of a run is the whole map, it
  walks out to the farthest cell the map calls free instead of standing still (`walk_out`). A mapper only
  takes a scan in once the robot has moved 0.2 m, so a robot that never moves never gets a map.

| parameter | default | what it stops |
| --- | --- | --- |
| `min_frontier_cells` | 12 | a clump that is a gap between two beams being driven to |
| `min_goal_distance` | 0.7 m | a goal under the robot's own wheels |
| `walk_out_reach` | 4 m | how far the first goal may be while nothing is a frontier yet |
| `avoid_radius` | 0.75 m | how much of the map a written-off goal takes with it |
| `reached_distance` | 0.35 m | when a goal counts as arrived |
| `stall_distance` / `stall_s` | 0.25 m / 25 s | a goal that has stopped getting closer |
| `patience_s` | 120 s | the longest one goal is worth |
| `period` | 0.5 s | how often the map is looked over |

## Without nav2, and the tests

```bash
ros2 launch mecanum_lab lab.launch.py world:=rooms
ros2 launch ohm_frontier frontier.launch.py
```

and the frontier rules are checked with no ROS at all — `frontiers.py` imports none:

```bash
cd ohm_frontier && python3 -m pytest test
```

20 passed with ROS in the environment, 14 passed and 1 skipped without it. What they cover is in the test
names; the launch files are among them, because a wrong import in a launch file is invisible until someone
types `ros2 launch`.

## What has been run, and what has not

Run, with slam_toolbox and nav2 on ROS 2 Kilted:

* The stack comes up, every nav2 lifecycle node configured and activated, and this node's goals are accepted
  and driven: a goal 0.2 m away was reported `reached` by both the stack and this node.
* Mapping needs motion. Standing at the `rooms` spawn the map holds 1 414 free cells and 2 walls; after 20 s
  of driving the same map holds 18 022 free cells and 767. Same robot, hall and parameters.
* Getting the stack to come up at all needed four things, each measured and each written into
  `config/nav2_rooms.yaml` with the measurement beside it: the two costmaps are sections of their own rather
  than parts of the server that reads them; `collision_monitor` and `docking_server` must be configured,
  because nav2's lifecycle manager has both in a fixed node list and one node that cannot configure aborts
  the whole bringup; the planner's section is named `GridBased` because that is the id the shipped behaviour
  tree asks for; and `spin` is one of the behaviour plugins because the tree builds an action client for it
  at activation.

Not working yet, measured on this machine:

* In the spawn pocket the controller accepts a goal and then publishes **nothing** — 18 s of listening on
  `/cmd_vel_nav`, `/cmd_vel_smoothed` and `/muster/cmd_vel` while a goal was active, nothing on any of them
  — and after 30 s answers `Failed to make progress`. The simulator is not the problem: the same robot driven
  by hand over the same topic moved 1.19 m in 6 s. So the defect is in `config/nav2_rooms.yaml`, in what the
  controller or the collision monitor makes of a goal inside a map whose every free cell is within half a
  metre of a wall. Exploration as a whole does not take off from the `rooms` spawn yet; the frontier rules
  are tested, and the loop closes once the robot moves.
* The `hall` frame (`tf.tree: slam`) is in neither `mecanum_lab/types.py`'s defaults nor that repo's CONTRACT
  — `mecanum_lab/tf_bcast.py` carries the default.
