# ohm-nav-exploration

Frontier-based exploration on top of the [mecanum-lab](../mecanum-lab) simulator, plus four reactive control
demos. One ROS 2 Python package: it reads the map somebody else is making, decides which unexplored place to
drive to next, and hands that place to the navigation stack. Mapping is slam_toolbox's job, driving is nav2's,
the simulator is used as it is.

This page is a quick start — one command per step, every block copy-pasteable on its own. The detail is in
[docs](docs):

| where | what it holds |
| --- | --- |
| [docs/frontier-exploration.md](docs/frontier-exploration.md) | the algorithm on a student's level: what a frontier is, why unknown is neither free nor occupied, how candidates are ranked, every parameter the node declares, and Yamauchi's paper |
| [docs/control-demos.md](docs/control-demos.md) | the four reactive demos: the rule, the two dialects of a missing echo, mecanum against car, and the metres that say whether a rule drives |
| [docs/troubleshooting.md](docs/troubleshooting.md) | the view is perfect and the robot does not move; empty map; no view at all; two runs on one machine |
| [docs/verification.md](docs/verification.md) | what has been measured on this machine, and what is only asserted by a test |
| [INSTALL.md](INSTALL.md) | what `./install.sh --check` looks at, and what its exit codes mean |

## Needs — one command

```bash
./install.sh --check
```

That list is tested rather than written down: python3, numpy and pytest, a sourced ROS 2, slam_toolbox's and
`nav2_bringup`'s launch files, `nav2_msgs`, `rviz2`, and a `mecanum-lab` checkout (default `~/git/mecanum-lab`,
elsewhere with `sim_dir:=`) new enough to take `tf_tree` and `lidar_no_echo`. The exit code names which class of
thing failed. Missing ROS packages are one line:

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-rviz2
```

## Run it — one command

From the repository root, with ROS 2 Kilted (the line your machine needs is printed by the check above):

```bash
source /opt/ros/kilted/setup.bash && colcon build --symlink-install && source install/setup.bash && ros2 launch ohm_frontier explore.launch.py
```

Five things start: the simulator in the `rooms` hall **with its window open**, slam_toolbox mapping the lidar,
nav2's navigation servers, this package's RViz view (`config/explore.rviz`, installed under
`share/ohm_frontier/rviz`), and the frontier node. The simulator is never asked for its own view: two viewers
with fixed frame `map` is one too many, and the one that loses shows an empty map while the other one is right.

After the first build, the same line without the build is the whole command:

```bash
source install/setup.bash && ros2 launch ohm_frontier explore.launch.py
```

The view shows the ranking as geometry — a dot per candidate, a bigger one on the goal, an arrow to it. The
arithmetic behind it is one word away, and it is off by default because the text is 0.30 m tall and one
sentence per candidate (21 of them on a `rooms` map) is a cloud of words over the hall:

```bash
source install/setup.bash && ros2 launch ohm_frontier explore.launch.py scores:=true
```

The same thing without restarting anything, which is the version to use while the robot is driving:
`ros2 param set /frontier_node show_scores true`.

## The runs worth typing

| command | hall | what it is for |
| --- | --- | --- |
| `ros2 launch ohm_frontier explore.launch.py` | `rooms` | the whole stack, the defaults above are its defaults |
| `ros2 launch ohm_frontier explore_rooms.launch.py` | `rooms` | 22 × 16 m of walls **with doorways**, so every unentered room is one clump and the ranking has something to choose between |
| `ros2 launch ohm_frontier explore_maze.launch.py` | `maze` | corridor one cell wide: a frontier is a corridor that ends, so the blacklist and `min_frontier_cells` are what is on display |
| `ros2 launch ohm_frontier explore_open.launch.py` | `open` | 30 × 20 m with nothing but its boundary: one frontier, nothing to rank, so the argument is what a frontier *is*. Run it first, `rooms` second |
| `ros2 launch ohm_frontier explore_arena.launch.py` | `arena` | a hall whose painted lanes reflect nothing back to the lidar: the map is what the sensors say, not what the world file says |
| `ros2 launch ohm_frontier explore_no_nav2.launch.py` | `rooms` | no planner, no controller, the goals only on `/frontier_goal` and in RViz — the ranking, the timeout and the blacklist, with nobody to blame |
| `ros2 launch ohm_frontier frontier.launch.py` | — | this node alone, for when the rest is already running |

Any of them takes `world:=` (rooms | maze | open | arena | production | track), `robot:=carlo`,
`headless:=false`, `rviz:=false`, `nav2:=false`, `rviz_config:=` and `sim_dir:=`; `--show-args` has a sentence
for each. `robot:=` reaches everything because the launch file rewrites every `<robot>/` in `config/*` at
start — nav2 and slam_toolbox are handed a *file* that names frames and topics literally, and so is RViz.

## The four control demos

One rule per file, no map and no planner, and each draws what it is deciding:

| command | the rule |
| --- | --- |
| `ros2 launch ohm_frontier reactive_wall_follow.launch.py` | find the wall on the right, then keep it at `wall_distance` (0.50 m from the kinematic centre) |
| `ros2 launch ohm_frontier reactive_avoid.launch.py` | drive one way and get round whatever the 360 beams see |
| `ros2 launch ohm_frontier reactive_turn_and_move.launch.py` | turn and drive at once, with a slip check against the odometry |
| `ros2 launch ohm_frontier move_to_point.launch.py` | turn FIRST, then drive dead straight, on a camera close enough to read |

`move_to_point` and `reactive_turn_and_move` take their place from outside, and their launch files publish one
five seconds in (`goal:=3.0,2.0`, empty for none) so that the line above moves the robot. To drive it by hand
instead — which is the cheapest way to watch a controller re-aim — type the place onto the same topic:

```bash
ros2 topic pub --once /muster/move_command std_msgs/msg/String "data: 'go 12.0 10.0'"
```

What each rule is for, and the path-and-net metres that show whether it drives or only moves:
**[docs/control-demos.md](docs/control-demos.md)**.

## The tests — one command

```bash
cd ohm_frontier && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```

The variable is not decoration: **in a shell with ROS sourced, a plain `pytest test` dies before it collects
anything**, because Kilted advertises a `launch_testing` plugin whose hook arguments the pip pytest here
rejects. It is the same line `./install.sh --check` prints. 107 pass on this machine; what they cover — and
what a passing test does not prove — is in **[docs/verification.md](docs/verification.md)**.

## What the node decides, in six lines

* A frontier is a free cell next to an **unknown** one, grouped into clumps; clumps under `min_frontier_cells`
  are a gap between two beams and get thrown away.
* Candidates are ranked by size, orientation and nearness — three weights, all live parameters, so
  `ros2 param set /frontier_node weight_distance 0.2` takes effect on the next decision.
* It **keeps the goal it took**: a better-looking candidate has to beat the score the goal was taken on by
  `reselect_margin` before it takes over. The first version re-chose every tick and arrived nowhere.
* The winner goes to nav2 as a `NavigateToPose` goal — nav2 does not read a `/goal_pose` topic, that is RViz's
  button — and the whole ranking goes out on `/frontiers` for the view.
* Arrived, or no nearer for `goal_timeout_s`, or refused three times: the place is written off with
  `avoid_radius` around it, and the goal is **cancelled** at the stack rather than forgotten.
* With nothing on the map far enough away to aim at, it walks out to the farthest free cell instead of standing
  still: a mapper only takes a scan in once the robot has moved.

Parameters, defaults and what each one stops:
**[docs/frontier-exploration.md#every-parameter-the-node-declares](docs/frontier-exploration.md#every-parameter-the-node-declares)**.

## When it does not do what the view says

* **The view is perfect and the robot does not move** → the last hop of the command chain is a `TwistStamped`
  in front of a base that wants a `Twist`, and DDS says nothing about the mismatch.
* **Empty map, or `frame does not exist`** → two publishers on one tf edge; the launch file asks for
  `tf_tree:=slam`.
* **A map with free space and no unknown** → the simulator's missing echo, answered with `lidar_no_echo:=inf`.
* **`rviz:=true` and no window** → an included launch file answers for your arguments; decide before the include.
* **The simulator dies at start with a missing hall file** → its build predates that hall; rebuild it.
* **Does it drive, or only move?** → `python3 ohm_frontier/tools/measure_drive.py muster 60 /tmp/wall.csv`

All six, with the measurements behind them: **[docs/troubleshooting.md](docs/troubleshooting.md)**.
