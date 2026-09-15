# The control demos: four rules, four questions, and the metres that say whether each one works

The frontier explorer needs a map, a planner and a stack. These four need none of that — one sensor and one
rule each — which is why they are the part of the package a lecture can use in ten minutes. Each is a file,
each has a launch file, and each draws what it is deciding so the rule and the robot are on the same screen.

    ros2 launch ohm_frontier reactive_wall_follow.launch.py      # follow the wall on the right
    ros2 launch ohm_frontier reactive_avoid.launch.py           # drive one way, around whatever is there
    ros2 launch ohm_frontier reactive_turn_and_move.launch.py   # turn and drive at the same time
    ros2 launch ohm_frontier move_to_point.launch.py            # turn FIRST, then drive, with the view open

The three sensors-and-a-rule demos read the lidar; `move_to_point.py` reads odometry. The view for the last
one is `config/drive.rviz` — the same thirteen displays as the exploration view on a camera 8 m from the
robot instead of 20 m, because the text an overlay writes over the robot is metres tall and a camera cannot
scale it.

## What each one is for

| demo | the question it answers | what it reads | the failure it demonstrates |
| --- | --- | --- | --- |
| `wall_following.py` | what does a rule that only looks at the last measurement do? | the lidar, in four bearings | it cannot cross open floor, and it gets nothing out of a wall it has already followed |
| `obstacle_avoidance.py` | how does a robot get round an obstacle it has no map of? | the lidar, all 360 beams | a concave corner: the two walls' repulsions add up along the bisector, which is exactly where the wall is |
| `turn_and_move.py` | what does it cost to turn and drive at once? | odometry, and a slip check against it | odometry that lies: the pose integrates metres the robot never drove |
| `move_to_point.py` | what does it cost to stop and aim first? | odometry and a typed goal | the same drive, with the aiming made explicit — run it against `turn_and_move.py` on the same hall with the same goal |

The numbers worth turning are `aim_tolerance` — 0.08 rad at 10 m is 80 cm of sideways error the node will
accept and still call `arrived`, so compute it before the robot moves — and `gain_turn` with `turn_limit`,
where a proportional turn meets what the wheels can deliver.

It is also the cheapest way to watch the odometry lie: with `slip = 1.0` in the simulator's physics the pose
integrates metres the robot never drove, and a controller with no sensor finds that out on its own, out loud,
by re-aiming.

## The two dialects of a missing echo, which both of the lidar demos have to speak

The simulator reports a beam that came back nowhere as its own `range_max` unless it was started with
`lidar_no_echo:=inf`, which writes infinity; `nan` shows up in recordings. All three mean *nothing there*,
none of them is a distance, and treating 8.0 m as "a wall at 8 m" is how a reactive demo ends up steering
away from the middle of an empty hall.

## Which side is which

The simulator's `LaserScan` starts at the robot's own nose (`angle_min = 0`) and its beam index grows
**counterclockwise**, so the right hand is at negative angles — index 3/4 of the circle. `mecanum_lab/sensors.py`
is where the beam directions are built, and this is where a lidar program goes wrong.

## Commanding a mecanum base, and why the default is now a car

`vy` is available on the simulator's mecanum robots, so a field that chooses a direction 30° off the nose can
be *driven* instead of being projected onto the heading. On the steering car of the same simulator the same
code is a bug — the simulator answers „steering robot cannot strafe, vy=0.25 dropped" and keeps driving
straight (`mecanum_lab/steering.py`).

That comparison is worth making, and it is not worth making in the default view of every demo: a robot that
drives sideways while facing elsewhere reads to an audience as a robot that misbehaves. So the demos command
`vx` and `omega` by default, and each has a `strafe` parameter — `strafe:=true` on its launch file — that
brings the mecanum sum back for the comparison.

## Measuring a demo: path against net

`tools/try_demo.sh <launch file> <robot> <domain id> [seconds] [world:=rooms]` is the whole experiment in one
command: it starts the stack headless, waits for the hall, samples the odometry, the `Twist` on
`/<robot>/cmd_vel` and the lidar's distances in six bearings for N seconds, stops the process group, and
prints what the node said while it was happening. Underneath it is `tools/measure_drive.py <robot> <seconds>
<out.csv> [clearance]`, which prints the ratio that separates driving from spinning, how far the robot ever
 got from where it started, the nearest echo and how often anything came inside the clearance margin, and how
often the commanded turn changed its mind. The numbers below are from this machine, headless, in the halls
named.

**One of those numbers is the path, and it had to be repaired before it could be believed.** Adding up the
distance between consecutive odometry samples looks like the easiest measurement in the file, and on this
simulator it is a measurement of the odometry: a robot standing still with its wheels commanded to *exactly zero*
— max `|vx|` and `|wz|` over 30 s both 0.000 — reported positions inside a 10 mm box and accumulated **32.06 m of
travel** in that half minute. `path_length` counts a step only once the position has moved 50 mm from the last one
it believed, which is over the p99 of the redraw (40 mm) and small against what the docs then claim with the
number; the tool prints the distance it discarded, so the size of the correction is on the screen rather than in a
comment somewhere. `net`, `furthest`, the echoes and the turn flips were never sums of steps and needed no such
repair — which is why the two tables below are compared on those, and why the `path` column of the first is not
measured the way the second's is.

### Before, measured on the commit that started this round

Both lidar demos were broken in the same way and the state lines did not show it:

| demo | hall | path | net | what the log said |
| --- | --- | --- | --- | --- |
| `reactive_wall_follow.launch.py` | `rooms` | 17.15 m | **0.45 m** | `following: wall 3.93 m` throughout, `wz` pinned at its −1.10 rad/s clamp |
| `reactive_avoid.launch.py` | `rooms` | 10.67 m | **0.02 m** | alternating `clear way` / `steering around an obstacle` every second, `vy` = ±0.35 and `wz` = ±1.1 flipping, 99 % of samples strafing |

The wall follower's cause: `steer()` rejected an echo only at the laser's own `range_max`, so a wall 3.93 m
to the right was fed into the keep-the-gap term as an error of 3.5 m against a 0.40 m gap, the proportional
turn saturated, and the robot spun while printing `following`. The field's causes: it re-decided which side
to go round from a fresh scan every 50 ms, so two comparable walls flipped the answer each cycle, and its
`STOPPED` answer was all zeros including the turn, so a blocked robot stayed blocked forever.

### After

Each row is one `try_demo.sh` run. The `path` column is the repaired measurement (50 mm dead band); the
figures quoted from earlier commits in the last column are raw step-sums, so compare the `net` numbers across
the two tables and treat the two `path` columns as different instruments.

| demo | hall | path | net | what the run showed |
| --- | --- | --- | --- | --- |
| `reactive_wall_follow.launch.py` | `rooms` | 18.34 m | **5.68 m** | reached 8.78 m from the spawn; nearest echo 0.24 m and 71 of 36 066 readings inside 0.30 m; longest unbroken hold at **0.50 ± 0.10 m of 7.1 s**, 15.5 s of the run inside that band; 113 turn flips in 4 231 turning samples; the worst 10 s moved 0.23 m and it was the first 10 s, while it was still looking for a wall |
| `reactive_wall_follow.launch.py` | `maze` | 15.57 m | **8.33 m** | 34.8 s of 70 s inside the set gap, in stretches up to **19.5 s**; and 1 050 readings inside 0.30 m with the nearest echo at 0.18 m, because a maze squeezes a robot that is following one wall — the `min_wall_gap` guard stops the wheels and turns the nose off it, which is the refusal, not a fix |
| `reactive_avoid.launch.py` | `rooms` | 20.51 m | **4.35 m** | reached 4.51 m out; **nothing inside 0.30 m in 38 712 readings**, nearest echo 0.41 m; 5 turn flips in 1 698 turning samples; `steering around an obstacle — more than 1.3 m ahead of the nose, +0.96 rad (left) has inf m` |
| `reactive_avoid.launch.py` | `rooms`, again | 21.14 m | **1.12 m** | the same build and the same hall: same clearance (0 readings inside 0.30 m of 38 508, nearest 0.42 m), same 0 % strafing, and three quarters of the run spent turning. Quoted because the difference between 4.35 m and 1.12 m is the honest spread of this rule, not a regression |
| `reactive_avoid.launch.py` | `production` | — | **1.20 m** | the refusal, measured over 120 s: **118.99 s of it in `STOPPED`**, nearest echo 0.70 m. 0.63 m of free aisle against a 0.66 m body diagonal — the aisle is the answer |
| `reactive_turn_and_move.launch.py` | `open` | 1.67 m | **arrived** | `arrived: the goal (4.00, 2.00) is 0.12 m from here, inside the 0.12 m tolerance this node was started with, at (3.96, 2.11, -68°)`, with 0 readings inside 0.30 m and 0 % strafing. Before, on the same hall: 28.04 m of path for **1.30 m of net** — a circle of 4.45 m radius |
| `move_to_point.launch.py` | `open` | 6.90 m | **6.87 m** | path/net 1.0, which is a straight line: `arrived at (12.00, 8.00), 0.11 m from the place`, nothing nearer than 4.66 m, 0 % strafing. Before: 30.08 m of path for **3.11 m of net**, 90 % of the samples carrying a sideways command, and it stopped 1.31 m short *while printing `arrived`* |

**What the wall follower had to be given, in the order it was discovered.** A far echo is not a reference
(`max_wall_range`), so a wall 5 m off the right is a direction to drive at and the phase says
`wall in sight, closing on it`. A gap error in metres times a gain is a heading in disguise, so it is now
explicitly a heading — lean the nose at the gap you want over `aim_lead` metres — and the wall's own angle,
measured from the two diagonal beams, is what closes that loop. The lean then needed a ceiling (`max_lean`, 20°):
at the 45° the first version allowed, the right-behind beam lies along the wall and stops telling the rule
anything about its angle, and a controller with no reading of its own heading drives a 0.23 m circle. And the gap
needed a floor (`min_wall_gap`, 0.30 m), because the robot is 0.46 m wide, its flank sits 0.23 m from the centre
the gap is measured at, and a run without the floor scraped a corner with the wall at 0.25 m while the rule
printed `following`. Four runs of 60 to 75 s went into the parameters that made it drive rather than wheel, and
each one is documented against the run it cost, in
[`wall_following.py`](../ohm_frontier/ohm_frontier/wall_following.py).

`strafe:=true` on any of the three mecanum demos brings the sideways sum back for the comparison; every number
above was measured without it, and `sideways command: 0.00 m/s at its most, 0 % of samples strafing` is
printed by the harness so the claim is checkable rather than asserted.

### What each of these still cannot do

* **The wall follower holds a wall, not a hallway.** 15.5 s of its 70 s in `rooms` are inside the set gap, in
  stretches up to 7.1 s; the rest is doorways and crossings where there is no wall at 0.50 m to hold, and the
  nearer side changes 25 times a run. It also follows the wall on one side only: in `maze` the left wall came to
  0.17 m and the rule has no term that could notice.
* **Obstacle avoidance keeps its clearance better than it makes progress.** Nothing came inside 0.30 m in either
  `rooms` run — 0 readings of 77 220 — but one run got 4.35 m out of the box and the next 1.12 m. It is a rule
  about the next half second, and the next half second is often "turn".
* **It will not fit `production`, and the refusal is the demo.** 0.63 m of free aisle against a 0.66 m body
  diagonal: measured, 118.99 s of 120 in `STOPPED`, 1.20 m from where it started, nearest echo 0.70 m.
* **Neither point controller has anything to do with obstacles.** `move_to_point` across `open` is 6.90 m of path
  against 6.87 m of net — a straight line — and the first default goal in its launch file was a place 0.1 m from
  the hall's own wall, which it drove to and then parked against with its lidar at 0.16 m. A pose controller that
  argues with a wall is a different program, and this package has one: `obstacle_avoidance.py`.
* **Nothing here has been driven in a corridor**, and `turn_and_move` integrates whatever the odometry tells it:
  with `slip = 1.0` in the simulator's physics, `drive 3.0` into a wall ends after 3 m of *wheels* with the robot
  still in the corner. That one is on purpose and is in the docstring.
