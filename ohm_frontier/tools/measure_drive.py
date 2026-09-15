"""Measure what a controller actually did, in metres — the number a state line cannot give you.

    python3 tools/measure_drive.py muster 60 /tmp/wall.csv          # hall, robot, seconds, csv

A reactive demo can print `following: wall 0.52 m` a thousand times and still have gone nowhere, and the
eye in the simulator's window is a poor instrument for that: a robot turning on the spot with a small
forward component looks like it is driving. So this samples three things at ~80 Hz — the odometry pose, the
`Twist` on `/<robot>/cmd_vel`, and the lidar's own distances in eight bearings — writes them as CSV, and
prints the one ratio that separates driving from spinning:

    path length   the sum of the steps between samples
    net           the distance between the first sample and the last

A robot that drove down a corridor has a path length near its net displacement. A robot in a limit cycle
has a large path and a net displacement near zero — measured on this package's own wall follower, before
its find-the-wall phase existed: **17.15 m of path, 0.45 m of net**, in 45 s, with the state line saying
`following` the whole time. The CSV columns of the lidar distances are what tells you why: the bearing the
rule reads, and how far the thing in that direction actually was.

Run it against a live stack: `ros2 launch ohm_frontier reactive_wall_follow.launch.py headless:=true
robot:=muster` in one terminal, this in another. `use_sim_time` is irrelevant here — the clock in the CSV is
wall-clock seconds since this started, because the question is "how far did it get in a minute".
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

#: the bearings the reactive family looks in, named as the demos name them
BEARINGS = (("ahead", 0.0, 3), ("right", -math.pi / 2, 2), ("right_ahead", -math.pi / 4, 2),
            ("right_behind", -3 * math.pi / 4, 2), ("left", math.pi / 2, 2), ("behind", math.pi, 4))


def yaw_of(odom) -> float:
    """Heading out of an `Odometry` message — the only thing of the orientation a 2D lecture needs."""
    q = odom.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def beam(scan, angle: float, window: int = 2):
    """The nearest echo around this bearing, or "" where nothing came back.

    `window` beams either side, because one beam of a 360-beam scan is one degree and a wall edge lands
    between beams more often than not. A missing echo is `range_max`, infinity or nan depending on how the
    simulator was started — all three are "nothing there", none of them is a distance.
    """
    if scan is None or not len(scan.ranges):
        return ""
    middle = round(angle / float(scan.angle_increment)) % len(scan.ranges)
    seen = [float(scan.ranges[(middle + offset) % len(scan.ranges)])
            for offset in range(-window, window + 1)]
    returned = [r for r in seen if r == r and r != float("inf") and r > 0.0]
    return f"{min(returned):.2f}" if returned else ""


def main(argv):
    if len(argv) < 3:
        raise SystemExit(__doc__)
    robot, seconds, out = argv[1], float(argv[2]), argv[3] if len(argv) > 3 else "/tmp/drive.csv"
    rclpy.init()
    node = Node("measure_drive")
    seen = {"odom": None, "cmd": None, "scan": None}
    node.create_subscription(Odometry, f"/{robot}/odom", lambda m: seen.__setitem__("odom", m), 10)
    node.create_subscription(Twist, f"/{robot}/cmd_vel", lambda m: seen.__setitem__("cmd", m), 10)
    node.create_subscription(LaserScan, f"/{robot}/scan", lambda m: seen.__setitem__("scan", m), 10)

    rows, start = [], time.time()
    while time.time() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if seen["odom"] is None:
            continue
        odom, cmd, scan = seen["odom"], seen["cmd"], seen["scan"]
        p = odom.pose.pose.position
        rows.append([round(time.time() - start, 2), round(p.x, 2), round(p.y, 2), round(yaw_of(odom), 2),
                     round(cmd.linear.x, 3) if cmd else "",
                     round(cmd.linear.y, 3) if cmd else "",
                     round(cmd.angular.z, 3) if cmd else ""]
                    + [beam(scan, angle, window) for _, angle, window in BEARINGS])
        if len(rows) % 40 == 0:
            last = rows[-1]
            print(f"{last[0]:6.1f} s  x={last[1]:7.2f} y={last[2]:7.2f} yaw={last[3]:+6.2f}  "
                  f"vx={last[4]:>6} vy={last[5]:>6} wz={last[6]:>6}  "
                  + "  ".join(f"{name}={value}" for name, value in
                              zip([n for n, _, _ in BEARINGS], last[7:])))

    with open(out, "w", newline="") as fh:
        write = csv.writer(fh)
        write.writerow(["t", "x", "y", "yaw", "vx", "vy", "wz"] + [n for n, _, _ in BEARINGS])
        write.writerows(rows)
    if not rows:
        print(f"nothing heard on /{robot}/odom in {seconds:.0f} s — is the simulator running?")
        return 1

    points = [(r[1], r[2]) for r in rows]
    path = sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))
    net = math.dist(points[0], points[-1])
    # The worst 10 s window: a robot that is stuck somewhere is stuck for a stretch of the run, not at its
    # end, and a total over the whole run is how a stuck stretch hides inside a good one.
    span = rows[-1][0] - rows[0][0]
    window = int(10 * (len(rows) - 1) / span) if span > 0 else 0     # samples in 10 s, averaged over the run
    worst = (None, 0.0)
    if window and len(rows) > window:
        for i in range(0, len(rows) - window, window):
            moved = math.dist(points[i], points[i + window])
            if worst[0] is None or moved < worst[0]:
                worst = (moved, rows[i][0])
    strafed = [abs(r[5]) for r in rows if r[5] != ""]
    ratio = f"{path / net:.1f}" if net > 0.05 else "— it came back to itself"
    print(f"\n{len(rows)} samples over {seconds:.0f} s on /{robot}/odom")
    print(f"  path {path:6.2f} m   net {net:6.2f} m   path/net {ratio}")
    print(f"  worst 10 s: {worst[0]:.2f} m of net displacement, starting at t={worst[1]:.0f} s"
          if worst[0] is not None else "  run too short for a 10 s window")
    if strafed:
        print(f"  sideways command: {max(strafed):.2f} m/s at its most, "
              f"{100 * len([s for s in strafed if s > 0.05]) / len(strafed):.0f} % of samples strafing")
    print(f"  csv: {out}")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
