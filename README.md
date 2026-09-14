# ohm-nav-exploration

Frontier-based exploration on top of the [mecanum-lab](../mecanum-lab) simulator. One ROS 2 Python package
that looks at the map, decides which unexplored place to drive to next, and hands that place to the
navigation stack. Mapping is slam_toolbox's job, driving is nav2's, the simulator is used as it is.

```
ohm_frontier/
  ohm_frontier/frontiers.py       the map and the frontier rules — no ROS in here
  ohm_frontier/frontier_node.py   the node: /map and /odom in, /goal_pose out
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

That starts four things: the simulator in `rooms` (headless, 22 × 16 m), slam_toolbox mapping its lidar,
nav2's navigation servers, and this node. Other hall, other robot:

```bash
ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo
```

`robot:=` also has to match the robot name written into `config/nav2_rooms.yaml`, because nav2 names its
topics and frames absolutely.

### Who publishes which tf frame

```
slam_map  →  map  →  muster/odom  →  muster/base_link  →  muster/laser
  slam       sim         sim              sim                    sim
```

The simulator publishes `map → <robot>/odom` itself and always will — `map → base_link` being the drifting
odometry is the point of its Kalman lab. slam_toolbox wants to publish that same edge, and a frame with two
parents is not a tf tree, so slam_toolbox is given the hall frame as its odometry reference (`odom_frame:
map`) and publishes its own frame above it. `slam_map` is what plans and draws from the map, and it is the
frame this node reads out of the map message rather than assuming. Details in
`launch/explore.launch.py`.

## What the node does

* Subscribes `/map` (slam_toolbox's, growing as it maps) and `<robot>/odom`. It publishes no map: two
  mappers in one graph means two answers to "what does the hall look like".
* Looks for free cells with an unknown cell next to them, groups them into clumps, and ranks the clumps by
  **cells per unit distance** — the wide opening down the hall beats the crack beside the wheel.
* Aims at the cell of the clump with the least wall around it, not at the clump's centroid. A clump that
  bends around a corner has its middle in that corner, and a goal inside a wall is refused by every
  planner.
* Publishes the winner on `/goal_pose`, nose pointed away from where it came from, so the first new scan
  looks into the unknown.
* Watches it. Arrived, or never moved towards, or too slow, and the place goes on a blacklist and the next
  clump gets its turn. With nothing left it says so once and stops.

Parameters worth touching: `min_frontier_cells` (12 — under that a clump is a gap between two beams, not a
room), `min_goal_distance` (0.45 m), `avoid_radius` (0.75 m), `reached_distance` (0.35 m), `stall_s` (25 s),
`patience_s` (120 s).

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

14 tests: the three states of a map read as its own convention says (`-1` unknown, `0` free), the map's
origin carried all the way to the goal, a frontier at the edge of a growing map still being a frontier, the
wide opening beating the near crack, goals not being aimed at a wall or at the robot's own doorstep, the
blacklist giving the next clump its turn — and both launch files being imported, which is a test because a
wrong import in a launch file is invisible until someone types `ros2 launch`.

## What has been run, and what has not

Run: the 14 tests, and the node against the simulator with a stand-in mapper (lidar beams painted into a
growing `/map`) and a stand-in driver in place of nav2. 17 goals, every one of them on a cell its own map
called free, 3 reached, 58 % of the hall's floor known afterwards.

Not run: slam_toolbox and nav2 — neither is installed on the machine this was written on. The parameters in
`config/` are written against their documented defaults, not against a run. Two things to expect:

* The goals are as correct as the map. With a mapper that has no scan matching — the stand-in here paints
  beams onto odometry — 15 of 17 goals sat inside a *real* wall of the hall by up to 25 cm, because that is
  how far its walls sat from the hall's. Everything that reads the map inherits its error.
* One mapper per graph. With two of them publishing `/map` by accident, 9 of the next 10 goals pointed at
  cells that the map they were checked against called unknown. Nothing in ROS warns you about this.
