# ohm-nav-exploration

Frontier-based exploration for ROS 2, plus four reactive control demos that need no map and no planner. The node
reads a map slam_toolbox is building, picks an unexplored place, hands it to nav2, and writes off the places
that turn out to be unreachable. The robot is the [mecanum-lab](../mecanum-lab) simulator.

![A 150-second exploration run in `rooms`, recorded from /map, /frontiers, odom and tf](docs/images/exploration.gif)

*Dark is unknown, light is mapped floor, black is walls. Pink is the track, the green triangle is the robot, the
blue rings are the frontier clumps, the orange dot is the goal it took.
[Last frame as a PNG](docs/images/exploration.png) · made by
[`tools/record_view.py`](ohm_frontier/tools/record_view.py) from a live run.*

## Requirements

| | |
| --- | --- |
| ROS 2 | Jazzy (current LTS) or newer, **with nav2 installed for it** |
| ROS packages | `ros-$ROS_DISTRO-navigation2` `ros-$ROS_DISTRO-nav2-bringup` `ros-$ROS_DISTRO-slam-toolbox` `ros-$ROS_DISTRO-rviz2` |
| Python | 3.10+, `numpy`, `pytest`; `pygame` for the simulator's window, `Pillow` for the recording tool |
| Simulator | a `mecanum-lab` checkout at `~/git/mecanum-lab`, or pass `sim_dir:=/path/to/it` |

What this machine has and what to do about the rest:

```bash
./install.sh --check          # and ./install.sh --pip for the python side, nothing else
```

## Install and run

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch ohm_frontier explore.launch.py
```

Five things: the simulator in the `rooms` hall, slam_toolbox, nav2, this package's RViz view, the frontier node.
Every launch below also takes `world:=` (rooms | maze | open | arena | production | track), `robot:=`,
`headless:=true`, `rviz:=false`, `nav2:=false` and `sim_dir:=` — `--show-args` has a sentence for each.

### Exploration

| command | hall |
| --- | --- |
| `ros2 launch ohm_frontier explore.launch.py` | `rooms`, the whole stack |
| `ros2 launch ohm_frontier explore_rooms.launch.py` | walls **with doorways**: one clump per unentered room |
| `ros2 launch ohm_frontier explore_open.launch.py` | 30 × 20 m and nothing in it: one frontier, nothing to rank |
| `ros2 launch ohm_frontier explore_maze.launch.py` | corridors one cell wide: blacklist, `min_frontier_cells` |
| `ros2 launch ohm_frontier explore_arena.launch.py` | lanes the lidar does not return: map against world file |
| `ros2 launch ohm_frontier explore_no_nav2.launch.py` | the ranking alone, goals on `/frontier_goal` |
| `ros2 launch ohm_frontier frontier.launch.py` | this node alone, for a stack already running |

The ranking as arithmetic instead of geometry — off by default:

```bash
ros2 launch ohm_frontier explore.launch.py scores:=true
ros2 param set /frontier_node show_scores true        # the same, while it is driving
```

### The four control demos

| command | the rule |
| --- | --- |
| `ros2 launch ohm_frontier reactive_wall_follow.launch.py` | find a wall, then hold it 0.50 m from the kinematic centre |
| `ros2 launch ohm_frontier reactive_avoid.launch.py` | drive one way, get round what the 360 beams see |
| `ros2 launch ohm_frontier reactive_turn_and_move.launch.py` | turn and drive at once, odometry with a slip check |
| `ros2 launch ohm_frontier move_to_point.launch.py` | turn first, then drive straight |

The last two wait for a place to go to; their launch files publish one five seconds in (`goal:=3.0,2.0`,
`goal:=` for none). To type one:

```bash
ros2 topic pub --once /muster/move_command std_msgs/msg/String "data: 'go 12.0 10.0'"
```

### Measure a run

```bash
ohm_frontier/tools/try_demo.sh reactive_wall_follow.launch.py muster 61 70 world:=rooms
```

Headless, on its own `ROS_DOMAIN_ID`, stopped and reported afterwards: path against net displacement, reach,
nearest echo, clearance violations, turn flips, and the node's own log.

The picture at the top:

```bash
ros2 launch ohm_frontier explore_rooms.launch.py headless:=true rviz:=false
python3 ohm_frontier/tools/record_view.py muster 150 exploration.gif --every 1.5 --fps 10 --still exploration.png
```

## Tests

```bash
cd ohm_frontier && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```

142, no robot and no hall. The variable is required in a shell that has ROS sourced — see
[docs/verification.md](docs/verification.md).

## Detail

| page | |
| --- | --- |
| [docs/frontier-exploration.md](docs/frontier-exploration.md) | frontiers, the three weights, goal commitment and blacklisting, every parameter and what it stops, Yamauchi's paper |
| [docs/control-demos.md](docs/control-demos.md) | the four rules, the two dialects of a missing echo, mecanum against car, the metres per rule |
| [docs/troubleshooting.md](docs/troubleshooting.md) | view right and robot still; empty map; no view; two runs or two distros on one machine |
| [docs/verification.md](docs/verification.md) | what has been measured, how, and what is only asserted by a test |
| [INSTALL.md](INSTALL.md) | what `./install.sh --check` looks at and what its exit codes mean |
