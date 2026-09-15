# What has been run, and what has not

Everything here was run on this machine — ROS 2 Kilted, the `mecanum-lab` simulator, headless unless said
otherwise. The point of keeping this separate from the README is the last section: which half of the claims
above it were *seen*, and which half are on the wire and asserted by a test.

## The stack, with slam_toolbox and nav2

* The stack comes up, every nav2 lifecycle node configured and activated, and this node's goals are accepted
  and driven: a goal 0.2 m away was reported `reached` by both the stack and this node.
* **Exploration has been seen to explore**, headless in `rooms`: the first two goals refused outright while
  `bt_navigator` was still activating (the three-refusal rule absorbing both), then `reached (10.51, 12.33)`,
  `reached (12.21, 11.33)`, and five frontiers reached inside two minutes while the odometry covered 3.60 m
  per 10 s — the pace `desired_linear_vel: 0.35` asks for. One typed goal on the same run: `SUCCEEDED`,
  2.65 m driven, and the command chain on all three hops at 20 Hz as a `Twist`.
* Mapping needs motion. Standing at the `rooms` spawn the map holds 1 414 free cells and 2 walls; after 20 s
  of driving the same map holds 18 022 free cells and 767. Same robot, hall and parameters.
* Getting the stack to come up at all needed four things, each measured and each written into
  `config/nav2_rooms.yaml` with the measurement beside it — **[they are listed in the troubleshooting
  page](troubleshooting.md#the-navigation-stack-refuses-to-come-up-at-all)**.
* **The velocity command reaches the robot as a `Twist`**, measured hop by hop while a goal was active, after
  a day was spent believing the controller published nothing: **[the numbers and the type trap are
  here](troubleshooting.md#the-robot-does-not-move-and-the-view-is-right--check-the-command-chain)**.

## The frontier search costs less than the period it runs in

A 40 × 60 m hall whose whole boundary is one frontier took 527.6 ms per look-over against the node's 500 ms
period — a search that misses every other tick. Now 82.8 ms on a map that changed, and 10.75 ms on one that
did not, which is what slam_toolbox actually sends at 2 Hz. Same hall, same map, this machine,
`test_frontiers.py` keeping the cache honest.

## The control demos

All four have launch files and have been run headless. **[The control-demo page](control-demos.md) carries
the path-and-net metres for each one, before and after the round that fixed the two lidar demos** — the
before numbers are worth reading, because both demos printed a healthy state line while going nowhere.

`move_to_point` across the `open` hall on a goal published by its own launch file: **6.90 m of path against
6.87 m of net**, which is a straight line, ending `arrived at (12.00, 8.00), 0.11 m from the place` with nothing
nearer than 4.66 m and no sample carrying a sideways command. The three overlays are on the wire: `/move_view`
in the namespaces `aim`, `goal`, `command`, `numbers` in frame `<robot>/odom`, plus `/wall_view` and
`/field_view`.

The two point controllers wait for a goal, so their launch files publish one (`goal:=`, empty to take it away);
measured without a goal the node sits in an empty hall printing `waiting for a command` — 1.14 m of path and
0.00 m of net in 45 s — which is a controller waiting rather than failing, and looks identical to a controller
failing unless the run says so.

**The `path` metre on this page is a repaired instrument, and both numbers in the sentence above are why.**
Summing the distance between consecutive odometry samples measures the odometry, not the robot: one robot,
parked, with its wheels commanded to exactly zero (`|vx|` and `|wz|` both 0.000 for 30 s) reported positions
inside a 10 mm box and accumulated **32.06 m of travel** in that half minute. Decimating does not help — the
same 30 s at 10 Hz reads 29.98 m, because the jitter is not high frequency, it is the position being redrawn
thousands of times inside one centimetre. `path_length` now believes a position only once it has moved 50 mm
from the last believed one, and prints what it discarded.

## The view

* **The numbers over the map are off, and the switch is on the wire.** Seen on a live `explore.launch.py` in
  `rooms` with a subscriber on `/frontiers`: by default the topic carries three geometry namespaces and no
  text at all — `{frontiers, selected, approach}` — and after one
  `ros2 param set /frontier_node show_scores true` the same tick carries `{frontiers, scores, clock}` with the
  `scores` namespace holding **one TEXT marker per candidate: 21 of them on that map**, which is the cloud of
  0.30 m words the todo was about. The node answers the switch out loud (`the numbers over the map are on
  again …`) because a parameter nobody hears being taken is a parameter nobody trusts.

    ros2 topic echo /frontiers --once            # what the view is sent, with the numbers off
    ros2 param set /frontier_node show_scores true

* **Seen rendering**, over `Xvfb` with screenshots of real runs: the grid, the robot's trajectory as a chain
  of pose arrows, the lidar's wall, the line to the goal, the aim line and the cone of `aim_tolerance` around
  it.
* **The text overlay has not been seen rendering on this machine.** It is on the wire — `ros2 topic echo
  /move_view` shows the `numbers` namespace carrying the phase and the two numbers, and the bullet above puts
  the frontier labels on the wire too — and `test_view_overlays.py` asserts the four things a text marker needs
  in order to appear (text, a pose at the robot, a size in metres, a non-zero alpha) — but on this machine's
  software-GL RViz the words never came up, while the lines and dots of the same message did. On a machine with
  a real GPU it is one look: start the control demo and see whether the phase is written over the robot.

## The tests

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q

One thing costs a student an hour, so it is said out loud: **in a shell that has ROS sourced, a plain
`python3 -m pytest test` dies before it collects anything.** ROS 2 Kilted advertises a `launch_testing`
pytest plugin whose hook arguments the pip pytest here (9.1.1) no longer accepts. The variable above is the
fix, and it is the line `./install.sh --check` prints.

What is covered is in the test names: the frontier rules, the life of a goal from candidate to blacklist, the
reactive maths, the markers each demo draws, the topic and message type of every hop a velocity command
travels, and the launch files — a launch file being the one kind of file here whose wrong imports were
invisible until someone typed `ros2 launch`, and the overlay code being the part no arithmetic test can
reach. Two bugs shipped in each of those blind spots: a `cos` that was never imported, and a text marker
placed by the field RViz does not read for text. The guards that exist now pay for themselves the way those
two were found — one compares every `executable=` in every launch file against `setup.py` by globbing rather
than by a list of names somebody has to remember to extend, and one checks that each ROS-guarded import
actually got its message types, which is the only way to notice that a file had asked `sensor_msgs` for
`Odometry` and been answered, by its own guard, with "no rclpy here. Source a ROS 2 installation" on a
machine that had one.

Measured with that: **136 passed**, in a shell with `/opt/ros/kilted` sourced and `ohm_frontier/` as the working
directory. The count grew from 122 over this round in three places, each of which is a hole a bug got through:
the reactive family rewritten around `ways`/`menu`/`opening`/`avoid` (24 tests), the launch files' behaviour
when a demo needs a goal and a display, and `tools/measure_drive.py` itself — because a document whose numbers
come from a measuring tool should be able to say the tool measures what it claims, and until this week nothing
tested it.
