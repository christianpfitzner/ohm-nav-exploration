# ohm-nav-exploration

Frontier-based exploration on top of the [mecanum-lab](../mecanum-lab) simulator. One ROS 2 Python package
that reads the map, decides which unexplored place to drive to next, and hands that place to the navigation
stack. Mapping is slam_toolbox's job, driving is nav2's, the simulator is used as it is.

```
ohm_frontier/
  ohm_frontier/frontiers.py       the map and the frontier rules — no ROS in here
  ohm_frontier/frontier_node.py   the node: /map and /odom in, a NavigateToPose goal out
  launch/explore.launch.py        simulator + slam_toolbox + nav2 + this node, one command
  launch/frontier.launch.py       this node alone, for when the rest is already running
  config/slam_toolbox.yaml        the mapper: cell size, when a scan is worth adding, loop closure
  config/nav2_rooms.yaml          nav2 for one simulated robot: frames, topics, costmaps
  test/                           the frontier rules, and the launch files imported
```

## Needs

* a checkout of `mecanum-lab` (default `~/git/mecanum-lab`) — the simulator, worlds including `rooms`
* `sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox`

## Start everything

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

```bash
ros2 launch ohm_frontier explore.launch.py
```

That starts four things: the simulator in `rooms` (headless), slam_toolbox mapping its lidar, nav2's
navigation servers, and this node. Other hall, other robot:

```bash
ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo
```

`robot:=` reaches everything: the same name is written into `config/nav2_rooms.yaml` and
`config/slam_toolbox.yaml` at launch time, because those two are files and nav2 reads them literally.

### Who publishes which tf frame

```
map  →  muster/odom  →  muster/base_link  →  muster/laser
 slam         sim              sim                 sim
```

The simulator normally publishes this whole tree, `map` being its hall. Here it is told to keep off the top
edge — `tf_tree:=slam`, which is `--set tf.tree=slam` in `mecanum_lab/tf_bcast.py` — and then it calls the
hall's own coordinate frame `hall` instead of `map`. A mapper and the simulator both publishing
`map → <robot>/odom` would give `odom` two parents, which is not a tree; nav2's costmap answers that with
`frame does not exist` rather than with a map. What plans and draws is the frame named in the map message,
which this node reads out instead of assuming.

Positions the simulator measures from the corner of its hall — GPS, truth — are in `hall`, and are not the
coordinates this node works in.

### The lidar's missing echo

The simulator reports a beam that did not come back as the laser's own range, 8.0 m, which is what every
laboratory there measures. slam_toolbox cannot use that: a reading at its `max_laser_range` is not "the
space beyond is open". So the launch file also passes `lidar_no_echo:=inf`
(`--set sensor.lidar.no_echo=inf`, `mecanum_lab/ros_bridge.py`), which is what
`sensor_msgs/msg/LaserScan` documents.

## What the node does

* Subscribes `/map` (slam_toolbox's, growing as it maps) and `<robot>/odom`, and turns the odometry pose
  into the map's frame before comparing the two — measured 0.18 m apart at spawn, which was already enough
  to put a goal under the robot's own wheels. It publishes no map: two mappers in one graph means two
  answers to "what does the hall look like".
* Looks for free cells with an unknown cell next to them, groups them into clumps, and ranks the clumps by
  **cells per unit distance** — the wide opening down the hall beats the crack beside the wheel.
* Aims at the cell of the clump with the least wall around it, not at the clump's centroid. A clump that
  bends around a corner or rings a pillar has its middle *in* the wall, and a goal inside a wall is not a
  slow goal but a refused one.
* Sends the winner to nav2 as a `nav2_msgs/action/NavigateToPose` goal, nose pointed away from where it came
  from so the first new scan looks into the unknown. **nav2 does not read a `/goal_pose` topic** — that one
  is RViz's button, not an interface. The same pose is published on `/frontier_goal` for a display and for a
  run without nav2.
* Watches it. Arrived, or never moved towards, or too slow. One refusal is not a verdict — a goal that
  leaves while nav2 is still activating is answered `not accepted` within a millisecond, so a place has to
  be refused three times before it goes on the blacklist. Then the next clump gets its turn, and with
  nothing left the node says so once and stops.
* When nothing on the map is far enough away to be a goal — which at the start of a run is the whole map,
  because the map is a metre across — it walks to the farthest cell the map calls free instead of standing
  still. That is `walk_out` in `frontiers.py`, and the reason it exists is that a mapper only adds a scan
  once the robot has moved 0.2 m: a robot that shuffles 0.2 m at a time never gives it the reason to.

Parameters worth touching: `min_frontier_cells` (12 — under that a clump is a gap between two beams, not a
room), `min_goal_distance` (0.7 m), `walk_out_reach` (4 m), `avoid_radius` (0.75 m), `reached_distance`
(0.35 m), `stall_s` (25 s), `patience_s` (120 s).

## Without nav2, and tests

```bash
ros2 launch mecanum_lab lab.launch.py world:=rooms headless:=true
```

```bash
ros2 launch ohm_frontier frontier.launch.py
```

and the frontier rules are checked without ROS at all:

```bash
python3 -m pytest test
```

20 passed with ROS in the environment, 14 passed and 1 skipped without it: the three states of a map read
as its own convention says (`-1` unknown, `0` free), the map's origin carried all the way to the goal, a
frontier at the edge of a growing map still being a frontier, the wide opening beating the near crack,
goals not being aimed at a wall or at the robot's own doorstep, a frontier all of whose cells are underfoot
producing a goal at least `min_goal_distance` away, a walk-out goal that stops at its reach and walks around
what is written off, and both launch files being imported — a test because a wrong import in a launch file
is invisible until someone types `ros2 launch`.

## What has been run, and what has not

Run, with slam_toolbox and nav2 on ROS 2 Kilted:

* The whole stack comes up, every nav2 lifecycle node configured and activated, and the frontier node's
  goals are accepted and driven: a goal 0.2 m away was reported `reached` by both the stack and this node.
* Mapping works, and needs motion: standing at the `rooms` spawn the map holds 1 414 free cells and 2
  walls, and after 20 s of driving the same map holds 18 022 free cells and 767 — the same robot, hall and
  parameters. A mapper that has not been given a reason to integrate a scan is not broken, it is waiting.
* Getting the stack to come up at all needed four things that are now in `config/nav2_rooms.yaml` with the
  measurement that found each: the two costmaps are sections of their own rather than parts of the server
  that reads them; `collision_monitor` and `docking_server` have to be configured, because nav2's lifecycle
  manager has both in a fixed node list and one node that cannot configure aborts the whole bringup; the
  planner's section is named `GridBased` because that is the id the shipped behaviour tree asks for; and
  `spin` is one of the behaviour plugins because the tree builds an action client for it at activation.

Not working yet, measured today:

* In the spawn pocket the controller accepts a goal and then publishes **nothing** — 18 s of listening on
  `/cmd_vel_nav`, `/cmd_vel_smoothed` and `/muster/cmd_vel` while a goal was active: no message on any of
  them — and after 30 s answers `Failed to make progress`. The simulator side is not the problem: the same
  robot driven by hand over the same topic moved 1.19 m in 6 s. So the remaining defect is in
  `config/nav2_rooms.yaml`, in what the controller or collision monitor does with a goal inside a map whose
  every free cell is within half a metre of a wall. Exploration as a whole therefore does not take off from
  the `rooms` spawn yet; the frontier rules themselves are tested and the loop closes once the robot moves.
* The `hall` frame (`tf.tree: slam`) is not in `mecanum_lab/types.py` defaults nor in that repo's CONTRACT
  yet — `mecanum_lab/tf_bcast.py` carries the default.
