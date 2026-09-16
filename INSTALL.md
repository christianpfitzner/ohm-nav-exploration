# Installing this on a machine

One script, and it checks before it installs anything:

```bash
./install.sh --check      # what is there, what is missing, what to do about it — installs nothing
./install.sh --help       # the options, and what each exit code means
```

`--check` is what happens with no arguments. It never calls `sudo` and never touches the network; it
prints one line per requirement — `ok`, `missing` with the command that fixes it, or `note` for something
that is unusual but harmless. `--apt` and `--pip` print the one command each of them would run, and add
`--yes` to have the script run it. `--build` runs `colcon build --symlink-install` in this directory.

## The four commands a run actually needs

```bash
source /opt/ros/$ROS_DISTRO/setup.bash  # once per terminal: without it ros2 is not on the PATH
colcon build --symlink-install           # once, and after every change to setup.py
source install/setup.bash                # so ros2 launch finds ohm_frontier
ros2 launch ohm_frontier explore.launch.py
```

The frontier rules need none of that — they are numpy and pytest:

```bash
cd ohm_frontier && python3 -m pytest test -q
```

## What `--check` looks at

| requirement | why this repo needs it |
|---|---|
| python3 ≥ 3.10, numpy | `frontiers.py` is numpy and nothing else |
| pytest | the tests; never needed by the robot |
| ROS 2, `rclpy` importable | `/map`, `/scan`, the `NavigateToPose` action |
| `slam_toolbox`'s `online_async_launch.py` | the mapper `explore.launch.py` includes |
| `nav2_bringup`'s `navigation_launch.py` | the navigation servers it includes |
| `nav2_msgs` | the action type the node sends its goals as |
| `rviz2` | the default run opens the view; `rviz:=false` is the way out |
| a `mecanum-lab` checkout | the hall, the robot, the lidar — `explore.launch.py` includes its launch file |
| that checkout taking `tf_tree` and `lidar_no_echo` | both are recent there, and this repo depends on both |
| pygame | the simulator's window |

The exit code says which class failed — 3 ROS, 4 ROS packages, 5 python, 6 simulator, 7 build — so the
script is usable from a teacher's own wrapper. Several things can be missing at once; the code reported is
the most fundamental one, since that is the only one worth fixing first.

## The two failures people actually hit

**ROS is installed but not sourced.** `ros2: command not found`, or `package 'ohm_frontier' not found`, on
a machine where everything is installed. A sourced ROS is a property of the terminal, not of the machine,
so a second terminal or a script that forgot the line looks exactly like a broken install.
`./install.sh --check` sources ROS itself in order to look, and says out loud when the terminal it was
called from had not.

**It names the ROS 2 it looked at, and two distros on one machine is ordinary.** `/opt/ros` can hold several,
and only some of them have nav2 on it. This machine's own `--check` says `ROS 2 jazzy … rclpy importable` and
then four `missing` lines asking for `ros-jazzy-slam-toolbox` and friends — because the nav2, slam_toolbox and
RViz set here is installed under the *other* prefix, and jazzy is a base install. Read the distro name in the
first line of the report before apt-ing anything: `--check` will happily find a ROS 2 that cannot run this
package, because `rclpy` being importable is a different question from `nav2_msgs` being importable.

**The simulator checkout is not where the launch file looks.** The default is `~/git/mecanum-lab`;
elsewhere it is `MECANUM_LAB=~/somewhere/mecanum-lab ./install.sh --check` or
`./install.sh --sim-dir=~/somewhere/mecanum-lab`, and the same variable reaches `explore.launch.py`
through its `sim_dir:=` argument. Note that the simulator being *unbuilt* is not this failure: the launch
file falls back to the checkout, which is why that line is a `note` and not a `missing`.

One more, because it looks like a broken test suite and is not one: in a shell with ROS sourced, a pytest
newer than 8 from pip refuses to start at all — ROS's `launch_testing` plugin declares a hook argument
that no longer exists in the hookspec. The tests themselves are fine:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q     # in ohm_frontier/, with ROS sourced
```

## Measured on the machine this was written on

Ubuntu 24.04, Python 3.12.3, ROS 2 Kilted, 2026-09-14. Everything checked `ok` **except** `rviz2`, which
is not installed there — the default run therefore opens no view on that machine until
`sudo apt install ros-$ROS_DISTRO-rviz2` has been typed, and `rviz:=false` runs without it. `mecanum_lab` is
not built as a ROS package there either, which is the expected teaching-machine state and only costs the
`ros2 launch mecanum_lab …` form of the command.
