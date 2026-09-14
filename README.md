# ohm-nav-exploration

Frontier-based exploration on top of the [mecanum-lab](../mecanum-lab) simulator: one ROS 2 Python package
that reads the map somebody else is making, decides which unexplored place to drive to next, and hands that
place to the navigation stack. Mapping is slam_toolbox's job, driving is nav2's, the simulator is used as it
is. This file is only how to run it; **[docs/frontier-exploration.md](docs/frontier-exploration.md)**
explains the algorithm on a student's level — what a frontier is, why unknown is neither free nor occupied,
how candidates are ranked, what Yamauchi's paper says, and where each of its steps lives in this code.

```
ohm_frontier/
  ohm_frontier/frontiers.py       the map and the frontier rules — no ROS in here
  ohm_frontier/frontier_node.py   the node: /map and /odom in, a NavigateToPose goal and the frontiers out
  ohm_frontier/{wall_following,obstacle_avoidance,turn_and_move}.py   the three reactive demos: a lidar and
                                  a rule, no map, no planner — for the lecture, and launchable on their own
  launch/explore.launch.py        simulator + slam_toolbox + nav2 + RViz + this node, one command
  launch/explore_<hall>.launch.py one per hall worth showing: rooms, maze, open, arena
  launch/explore_no_nav2.launch.py the same run with nothing driving — the decisions on their own
  launch/reactive_<demo>.launch.py hall + one reactive node, for the lecture
  launch/frontier.launch.py       this node alone, for when the rest is already running
  config/slam_toolbox.yaml        the mapper: cell size, when a scan is worth adding, loop closure
  config/nav2_rooms.yaml          nav2 for one simulated robot: frames, topics, costmaps
  config/explore.rviz             the frontier view: map, lidar, trajectory, plan, every frontier, the goal
  test/                           the frontier rules, the life of a goal, the reactive maths, the launch files
docs/, install.sh, INSTALL.md     the explanation; and ./install.sh --check for what a machine lacks
```

## Needs

```bash
./install.sh --check        # what is there, what is missing, what to type about it — installs nothing
```

That is the list, tested rather than written down: python3, numpy and pytest, a sourced ROS 2, the two launch
files this package includes (`slam_toolbox`'s and `nav2_bringup`'s), `nav2_msgs`, **`rviz2`**, and a
`mecanum-lab` checkout (default `~/git/mecanum-lab`, elsewhere by `sim_dir:=`) new enough to take `tf_tree`
and `lidar_no_echo`. Its exit code names which class of thing failed; `INSTALL.md` has the options.

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup \
                 ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-rviz2        # or run it: rviz:=false
```

Built and run on ROS 2 Kilted, in the halls `rooms`, `maze`, `open`, `arena`, `production` and `track`.
`rviz2` is not part of the ROS base install and the default run opens the view, so a machine without it needs
that line or `rviz:=false`; the launch file looks for it and answers with what to type rather than with a
process that died.

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
another view and `sim_dir:=` for a simulator elsewhere; `--show-args` has a sentence for each. Whatever `rviz`
says, the simulator is never asked for its own view: two viewers with fixed frame `map` is one too many, and
the one that loses shows an empty map while the other one is right.

### One file per thing worth showing

| command | hall | what it is for |
| --- | --- | --- |
| `explore.launch.py` | `rooms` | the whole stack; the defaults above are its defaults |
| `explore_rooms.launch.py` | `rooms` | 22 × 16 m of walls **with doorways**, so every unentered room is one clump and the ranking has something to choose between |
| `explore_maze.launch.py` | `maze` | 6 × 6 m of corridor one cell wide: a frontier is a corridor that ends, so the blacklist and `min_frontier_cells` are what is on display |
| `explore_open.launch.py` | `open` | 30 × 20 m with nothing but its boundary: one frontier, nothing to rank, so the argument is about what a frontier *is*. Run it first, `rooms` second |
| `explore_arena.launch.py` | `arena` | an empty hall whose painted lanes reflect nothing back to the lidar: the map is what the sensors say, not what the world file says |
| `explore_no_nav2.launch.py` | `rooms` | `nav2:=false` in a file: no planner, no controller, the goals only on `/frontier_goal` and in RViz — the ranking, the timeout and the blacklist in seconds, with nobody to blame |

## Two things that will otherwise bite you

The launch file handles both; **[the simulator's two quirks](docs/frontier-exploration.md#the-simulators-two-quirks)**
has the measurements and the tf diagram.

* **Who publishes the top edge of the tf tree.** The simulator publishes `map → <robot>/odom` too, and two
  publishers on that one edge give `odom` two parents — not a tree, and nav2's costmap answers with
  `frame does not exist` rather than with a map. The launch file asks for `tf_tree:=slam`, after which the
  simulator's hall coordinates are called `hall`, not `map`: the simulator's truth positions are in `hall`,
  which is not the frame this node works in, so the node reads the frame out of the map message instead.
* **The lidar's missing echo.** A beam that did not come back is reported as the laser's own range, 8.0 m —
  what every laboratory there measures — and slam_toolbox cannot use that, because a reading at its
  `max_laser_range` is not "the space beyond is open". Hence `lidar_no_echo:=inf`, which is what
  `sensor_msgs/msg/LaserScan` documents.

## What the node decides

* Subscribes `/map` and `<robot>/odom`, and brings the odometry pose into the map's frame before comparing the
  two — measured 0.18 m apart at spawn, enough to put a goal under the robot's own wheels. It publishes no map:
  two mappers in one graph means two answers to "what does the hall look like".
* Free cells next to **unknown** cells, grouped into clumps, clumps under `min_frontier_cells` thrown away.
  Aimed at the cell with the least wall around it, not at the clump's centroid — the middle of a clump that
  bends around a corner is in the wall, and a goal in a wall is refused, after which the good opening beside
  it gets blacklisted. Ranked by a weighted sum of **size, orientation and nearness**, the three weights
  being parameters (`weight_size`, `weight_orientation`, `weight_distance`); `cells / metres`, which this
  node used to rank by, is that ranking with two of the three weights stuck at one.
* **Keeps the goal it took.** The first version chose the goal in the same tick that looked at the map, so the
  timer period *was* the re-decision period: at 0.5 s a map growing under the robot's own wheels handed over a
  new winner several times a minute, the robot drove a metre towards one frontier and then towards the next,
  and arrived at none of them. A goal now stands until it is reached, until the robot stops getting nearer to
  it, or until the stack says no; a better candidate has to beat the score the goal was taken on by
  `reselect_margin` to take over. See `keep_or_switch`.
* Sends the winner to nav2 as a `nav2_msgs/action/NavigateToPose` goal, nose pointed away from where it came
  from so the first new scan looks into the unknown. **nav2 does not read a `/goal_pose` topic** — that is
  RViz's button, not an interface — so the pose also goes out on `/frontier_goal` for a display or a student's
  own follower, and the whole ranking on `/frontiers` as markers: the question a lecture asks is not where it
  is going but why that one and not the other four.
* Watches it: arrived, or no longer getting closer, or refused. One refusal is not a verdict — a goal that
  leaves while `bt_navigator` is still activating is answered `not accepted` within about a millisecond — so
  a place has to be refused three times before it and `avoid_radius` around it are written off. A goal the
  node drops is **cancelled** at the stack too, not just forgotten: `bt_navigator` serves one goal at a time.
* When nothing on the map is far enough away to be a goal — at the start of a run that is the whole map — it
  walks out to the farthest cell the map calls free instead of standing still (`walk_out`): a mapper only takes
  a scan in once the robot has moved 0.2 m, so a robot that never moves never gets a map.

| parameter | default | what it stops |
| --- | --- | --- |
| `min_frontier_cells` | 12 | a clump that is a gap between two beams being driven to |
| `goal_timeout_s` | 10 s | a goal the robot has not come `progress_distance` (0.25 m) nearer to in that long |
| `reselect_margin` | 1.5 | a fresh candidate taking over a goal that is still being driven to |
| `weight_size` / `weight_orientation` / `weight_distance` | 1.0 / 0.25 / 1.0 | what the ranking values |

The weights are declared with a description and a range of 0 … 10 — `ros2 param describe /frontier_node
weight_distance` shows it, `ros2 param set /frontier_node weight_distance 0.2` takes effect on the next
decision with no restart (`weights_changed`), and a tuning panel builds sliders out of the very same ranges;
this machine has no `rqt_reconfigure` to look at, and nothing depends on it. A negative weight, which would
reward a frontier for being far away, is refused by rclpy, and `reselect_margin` is read as at least 1.0
because 1.0 is the flip-flop above. Everything else the node declares, with what each one is for:
**[docs](docs/frontier-exploration.md#every-parameter-the-node-declares)**.

## Without nav2, and the tests

```bash
ros2 launch mecanum_lab lab.launch.py world:=rooms     # the simulator alone
ros2 launch ohm_frontier frontier.launch.py            # the decisions, nothing driving them
cd ohm_frontier && python3 -m pytest test              # the rules, with no ROS at all
```

One thing costs a student an hour, so it goes here: **in a shell that has ROS sourced that last command dies
before it collects anything.** ROS 2 Kilted advertises a `launch_testing` pytest plugin whose hook arguments
the pip pytest here (9.1.1) no longer accepts, so the suite needs `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest test -q` — the same line `./install.sh --check` prints. Measured with that: **78 passed**,
in a shell with `/opt/ros/kilted` sourced and `ohm_frontier/` as the working directory. What is covered is in the test
names: the frontier rules, the life of a goal from candidate to blacklist, the reactive maths, and the launch
files — a launch file being the one kind of file here whose wrong imports were invisible until someone typed
`ros2 launch`.

## What has been run, and what has not — with slam_toolbox and nav2 on ROS 2 Kilted

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
