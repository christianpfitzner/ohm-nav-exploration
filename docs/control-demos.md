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

`tools/measure_drive.py <robot> <seconds> <out.csv>` samples the odometry, the `Twist` on `/<robot>/cmd_vel`
and the lidar's distances in six bearings, and prints the ratio that separates driving from spinning. Run it
against a live demo. The numbers below are from this machine, headless, in the halls named.

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

Filled in from the four lanes' measurements; each number is a `measure_drive` report quoted in the commit
that made it.
