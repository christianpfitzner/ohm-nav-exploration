# When it does not do what the view says

Four traps that cost somebody days here, each with the measurement that found it, and a checklist for the
one symptom that covers all of them: **the view looks perfect and the robot does not move**.

The short version of why this file exists: in this stack, nearly everything that can break is upstream of a
topic, so it breaks silently. The map, the plan, the costmaps, the tf tree and the RViz view can all be
right at the moment the robot stops, and nothing in the log will disagree.

## The robot does not move and the view is right — check the command chain

Nav2 hands the speed command through four hands, and every hand is a topic:

```
controller_server ─┐
                   ├→ /cmd_vel_nav → /cmd_vel_smoothed → /<robot>/cmd_vel → the simulator
behavior_server  ──┘        (nav2's own launch file renames the first hop)
velocity_smoother ─→ /cmd_vel_nav → /cmd_vel_smoothed
collision_monitor ─→ /cmd_vel_smoothed → /<robot>/cmd_vel
```

Two questions, in this order:

**1. Is the last hop the message type the base listens for?** Every nav2 node that writes or reads a speed
command has `enable_stamped_cmd_vel`, and it picks the **message type** of the hop, not its rate: true is
`geometry_msgs/msg/TwistStamped`, false is `geometry_msgs/msg/Twist`. Kilted ships it as **true**; the
simulator drives from a **`Twist`** on `/<robot>/cmd_vel` (`mecanum-lab` CONTRACT §6.9, and one `Twist` is
what a student's own node publishes while they hold `w`). DDS refuses to match a publisher and a
subscription of different types and does not mention that it refused.

Measured with a goal 1.5 m away on the `rooms` spawn, subscribing both types side by side:

```
/cmd_vel_nav         Twist           0.0 Hz   none
/cmd_vel_nav         TwistStamped   20.0 Hz   vx=+0.350
/cmd_vel_smoothed    Twist           0.0 Hz   none
/cmd_vel_smoothed    TwistStamped   20.0 Hz   vx=+0.350
/muster/cmd_vel      Twist           0.0 Hz   none      ← what the simulator listens for
/muster/cmd_vel      TwistStamped   19.9 Hz   vx=+0.350  ← what nav2 writes
odom: (11.25, 12.25) → (11.25, 12.25)                    # 0.00 m
```

That is 20 Hz of real motion commands, passed through the collision monitor without a scratch, on a topic
nothing was listening to. The fix is four lines of `config/nav2_rooms.yaml` — one setting, not four — and
`test_nav2_command_chain.py` keeps it.

**The trap inside the trap:** `ros2 topic echo` and `ros2 topic hz` pick one of the two types and then
report *no messages* about the other. This repo therefore spent a day believing that the controller
"published nothing for 18 s" and blamed the costmap, when the controller was publishing 20 Hz the whole
time. Subscribe both types when a topic is quiet:

```
ros2 topic info /muster/cmd_vel -v --no-daemon      # the type list, and who publishes and subscribes
```

A topic that lists both `Twist` and `TwistStamped` with one publisher and no subscriber is this bug. (One
`rclpy` node cannot hold both types on one topic name, so a probe needs two nodes in two contexts.)

**2. Is the stack actually up?** `bt_navigator` refuses every goal while the lifecycle manager is still
working, and the refusal comes back in about a millisecond. `ros2 lifecycle get /bt_navigator` says
unconfigured; the frontier node counts three refusals before it writes a place off, precisely so this
startup window does not blacklist the hall.

## The map is empty, or a costmap says `frame does not exist`

Two publishers on the top edge of the tf tree. The simulator publishes `map → <robot>/odom` as well, which
gives `odom` two parents; a tf graph with two parents is not a tree, and nav2's costmap answers with
`frame does not exist` rather than with a map. The launch file asks for `tf_tree:=slam`, after which the
simulator's hall coordinates are called `hall` and not `map` — **[the tf diagram and the measurements are
in the algorithm doc](frontier-exploration.md#the-simulators-two-quirks)**.

The same section covers a second way to get an empty map with nothing wrong in the tf tree: **two viewers**.
`explore.launch.py` includes the simulator's launch file with `rviz:=false` and starts its own, because two
viewers both claiming fixed frame `map` is one too many and the one that loses draws an empty map while the
other one is right. Ask the simulator for its view as well (`rviz:=true` reaches both) and which map you see is
a race.

## The map has free space but no unknown, or the mapper never grows

A beam that did not come back is reported by the simulator as the laser's own range, 8.0 m — what every
laboratory there measures — and slam_toolbox cannot use that, because a reading at its `max_laser_range` is
not "the space beyond is open". Hence `lidar_no_echo:=inf`, which is what `sensor_msgs/msg/LaserScan`
documents. The launch file passes it.

## `rviz:=true` and no view appears at all

An `IncludeLaunchDescription` writes the arguments of the file it includes into the *including* file's
configuration space, so the simulator's `rviz` answers for ours afterwards — and `robots` reads as empty
though nobody mentioned it. The two launch files here that open a view decide it **before** the include, and
`test_launch_files.py` keeps them in that order.
**[The long version, with the three-entity measurement](frontier-exploration.md#and-a-third-thing-which-is-in-the-launch-files-not-in-the-simulator)**.

## The simulator dies at start with a missing hall file

The simulator was built with `--symlink-install` **before** the hall you asked for was added to its
`worlds/`, so the build directory has a symlink per hall that existed at build time and no entry for the new
one. nav2, slam_toolbox and RViz then carry on without it, which is how "the view works and the robot does
not move" arrives from a completely different direction than the section above. Rebuild the simulator:

```
cd ~/git/mecanum-lab && colcon build --symlink-install
```

`ros2 launch ohm_frontier explore.launch.py sim_dir:=/path/to/mecanum-lab` is the workaround: it makes the
launch resolve the simulator from its checkout instead of from the install tree.

## Two runs on one machine

Two stacks on one `ROS_DOMAIN_ID` are two answers to every question — two odom, two scans, two cmd_vel
publishers, and a robot that gets commands from the other run. Give each run its own domain *and* its own
robot name:

```
ROS_DOMAIN_ID=61 ros2 launch ohm_frontier explore.launch.py headless:=true robot:=wally
```

And stop what you started by its process group, `kill -INT -<pgid of your ros2 launch>`: `pkill -f ros2`
kills everybody's stack on the machine, including the ones you cannot see, and a SIGKILLed DDS process
leaves shared-memory segments behind in `/dev/shm` that can make the *next* run's lifecycle service calls
fail in ways that look like a broken nav2.

## Does it actually drive, or does it only move? Measure it

A controller can print `following: wall 0.52 m` a thousand times and go nowhere, and an eye on the
simulator's window is a poor instrument for that — a robot turning on the spot with a small forward
component looks like it is driving. The package carries the instrument:

```
python3 ohm_frontier/tools/measure_drive.py muster 60 /tmp/wall.csv
```

Its `path/net` ratio is the answer: 1.2 is a robot that drove down a corridor, 38 is a robot in a limit
cycle. **[The control-demo page](control-demos.md)** has the numbers this produced for every demo, before
and after.

## The navigation stack refuses to come up at all

Four things this config does that nav2's own does not, each measured, each written into
`config/nav2_rooms.yaml` with the measurement beside it: the two costmaps are sections of their own rather
than parts of the server that reads them (nested, they are read as parameters of that node, the controller
never activates and nothing in the log mentions the costmap); `collision_monitor` **and** `docking_server`
have to be configured, because nav2's lifecycle manager has both in a fixed node list and one node that
cannot configure aborts the whole bringup; the planner's section is named `GridBased` because that is the id
the shipped behaviour tree asks for; and `spin` has to be one of the behaviour plugins because the tree
builds an action client for it at activation.

## What is still broken

* The `hall` frame (`tf.tree: slam`) is in neither `mecanum_lab/types.py`'s defaults nor that repo's
  CONTRACT — `mecanum_lab/tf_bcast.py` carries the default.
