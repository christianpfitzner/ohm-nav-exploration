# ohm-nav-exploration

Frontier-based exploration on top of the [mecanum-lab](../mecanum-lab) simulator. This repository is one
ROS 2 Python package that looks at the lidar, decides which unexplored place to visit next, and hands
that place to the navigation stack. The simulator is used as it is; nav2 does the driving.

```
ohm_frontier/
  ohm_frontier/frontiers.py       the grid, the ray painting, the frontiers — no ROS in here
  ohm_frontier/frontier_node.py   the node: /scan and /odom in, /map and /goal_pose out
  launch/explore.launch.py        simulator + nav2 + this node, in one command
  launch/frontier.launch.py       this node alone, for when the rest is already running
  config/nav2_rooms.yaml          nav2 for one simulated robot: frames, topics, costmaps
  test/test_frontiers.py          the algorithm, without ROS
```

## Needs

* a checkout of `mecanum-lab` (default `~/git/mecanum-lab`) — the simulator, worlds including `rooms`
* `sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup`

## Start everything

```bash
colcon build --symlink-install && source install/setup.bash
```

```bash
ros2 launch ohm_frontier explore.launch.py
```

That starts the simulator in `rooms` (headless, 22 × 16 m), nav2's navigation servers, and the frontier
node. Other hall, other robot:

```bash
ros2 launch ohm_frontier explore.launch.py world:=maze robot:=carlo
```

Note that `robot:=` also has to match the robot name written in `config/nav2_rooms.yaml`, because nav2
names its topics absolutely.

To watch it, start rviz with the nav2 config and add the `/map` the node publishes — the simulator's own
window shows the same hall from its own point of view:

```bash
ros2 launch mecanum_lab lab.launch.py world:=rooms rviz:=true headless:=false
```

## What the node does

* Subscribes `<robot>/scan`, `<robot>/odom`, `/tf_static` (where the lidar sits on the robot) and
  `/sim/world` (how big the hall is).
* Paints a 10 cm grid: free where a beam travelled, occupied where one stopped, unknown where none has
  been. A beam that answers nothing counts as open as far as it reached.
* Publishes that grid on `/map` at 2 Hz, for nav2's global costmap and for rviz.
* Finds the clumps of free cells that touch unknown cells, ranks them by **cells per unit distance** —
  a wide opening down the hall beats a crack beside the wheel — and publishes the best one on
  `/goal_pose`, facing the way it intends to go. The place it aims at is the cell of the clump with the
  least wall around it, not the clump's centroid: a clump that bends around a corner has its middle in
  that corner, and a goal inside a wall is refused by every planner.
* Waits for that goal. Arrived, refused (the robot never moved) or too slow, the place goes on a
  blacklist and the next clump is tried. When nothing is left it says so once and stops sending goals.

Parameters worth touching: `min_frontier_cells` (12 — below that a clump is a gap between two beams, not
a room), `min_distance` (0.6 m — not aiming at your own doorstep), `avoid_radius` (0.9 m), `resolution`
(0.10 m), `stall_s` (25 s).

## Without nav2, and tests

The mapping half runs against the simulator alone:

```bash
ros2 launch mecanum_lab lab.launch.py world:=rooms headless:=true
```

```bash
ros2 launch ohm_frontier frontier.launch.py
```

and the algorithm is checked without ROS at all (8 tests):

```bash
python3 -m pytest test
```

The unit tests cover the ray painting, a frontier being free floor next to unknown, the outside of the
grid not counting as unexplored, the ranking, and the blacklist. What they cannot check is the driving:
that needs nav2 installed, and `config/nav2_rooms.yaml` is written against nav2's defaults on ROS 2
Kilted rather than against a run.
