"""Pick where to explore next in a hall nobody has mapped, and tell the navigation stack about it.

Listens:
    /map                                      slam_toolbox's map: free, occupied and unknown cells
    <robot>/odom                              where the robot thinks it is
Says:
    /goal_pose                                the frontier to drive to, and the heading to face there

Only deciding is in here. Mapping is slam_toolbox's job, driving is nav2's, so this node keeps no map of
its own and publishes none: two mappers in one graph means two answers to "what does the hall look like",
and the one RViz shows is never the one the planner used.

The map is the interesting input, because a cell there has three states — free, occupied, **unknown** —
and unknown is neither of the others. A frontier is free floor next to unknown floor: the next room. A
rule of "free next to not-free" cannot tell unknown from a wall and stops finding rooms after the first
room is mapped, which is the classic way this algorithm ends up doing nothing.

Two things the node does that the textbook description leaves out, both because running it showed them:

* **A goal is given up on.** A frontier is a direction, not a reachable pose, and nav2 is free to refuse
  one. A goal whose robot never moves towards it, or which the robot never arrives at, writes off its
  surroundings and the next frontier is asked. Without that the node asks one impossible place forever.
* **Empty is said once.** With nothing left to map, the same message would otherwise arrive twice a
  second.
"""
from math import atan2, cos, hypot, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from .frontiers import Grid


class FrontierNode(Node):
    def __init__(self):
        super().__init__("frontier_node")
        for name, value in {
            "robot": "muster",
            "map_topic": "/map",
            "goal_topic": "/goal_pose",
            "period": 0.5,                    # how often the map is looked over
            "min_frontier_cells": 12,         # below this a clump is a crack between two beams
            "min_goal_distance": 0.45,
            "avoid_radius": 0.75,             # written off around a goal that did not work out
            "reached_distance": 0.35,
            "stall_distance": 0.25,           # less approach than this in `stall_s` is not moving
            "stall_s": 25.0,
            "patience_s": 120.0,              # after this a goal is given up on
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.grid = None                      # slam_toolbox's map, as cells
        self.map_frame = ""
        self.pose = None                      # (x, y, theta) in the odometry frame
        self.goal = None                      # the Frontier under way
        self.goal_since = 0.0
        self.closest = 1e9                    # nearest approach to it so far
        self.avoid = []                       # centres reached or given up on
        self.said_empty = False

        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, 10)
        self.create_subscription(Odometry, f"/{self.robot}/odom", self.on_odom, 10)
        self.create_timer(float(self.get_parameter("period").value), self.on_timer)

    # ------------------------------------------------------------------------------- what comes in

    def on_map(self, msg: OccupancyGrid):
        self.grid = Grid.from_map(msg)
        if not self.map_frame:
            self.map_frame = msg.header.frame_id
            self.get_logger().info(
                f"map on frame {msg.header.frame_id}: {msg.info.width} × {msg.info.height} cells at "
                f"{msg.info.resolution:g} m per cell, origin "
                f"({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})")

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     2.0 * atan2(q.z, q.w))            # yaw of a quaternion; the world is flat

    # ------------------------------------------------------------------------------- what goes out

    def on_timer(self):
        if self.grid is None or self.pose is None:
            return
        if self.goal is not None:
            self.watch_the_current_goal()
        if self.goal is None:
            self.seek_goal()

    def watch_the_current_goal(self):
        x, y, _ = self.pose
        gap = hypot(x - self.goal.x, y - self.goal.y)
        self.closest = min(self.closest, gap)
        age = self.now() - self.goal_since
        if gap < float(self.get_parameter("reached_distance").value):
            self.get_logger().info(f"reached ({self.goal.x:.2f}, {self.goal.y:.2f})")
            self.goal = None
            return
        stalled = age > float(self.get_parameter("stall_s").value) \
            and self.closest > gap - float(self.get_parameter("stall_distance").value)
        if stalled or age > float(self.get_parameter("patience_s").value):
            self.get_logger().warn(f"({self.goal.x:.2f}, {self.goal.y:.2f}) given up on after {age:.0f} s"
                                   f" — nearest approach {self.closest:.2f} m")
            self.avoid.append((self.goal.x, self.goal.y))
            self.goal = None

    def seek_goal(self):
        candidates = self.grid.frontiers(
            robot=self.pose,
            min_cells=int(self.get_parameter("min_frontier_cells").value),
            min_distance=float(self.get_parameter("min_goal_distance").value),
            avoid=self.avoid, avoid_radius=float(self.get_parameter("avoid_radius").value))
        if not candidates:
            if not self.said_empty:
                self.get_logger().info("no frontier on this map — everything around has been reached or "
                                       "refused")
                self.said_empty = True
            return
        self.said_empty = False
        best = candidates[0]
        self.goal, self.goal_since, self.closest = best, self.now(), 1e9
        self.publish(best)
        self.get_logger().info(f"goal ({best.x:.2f}, {best.y:.2f}) — {best.cells} frontier cells, "
                               f"{best.distance:.1f} m away")

    def publish(self, frontier):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame or "map"         # the map's own frame, never assumed
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(frontier.x), float(frontier.y)
        msg.pose.orientation.z = sin(frontier.heading / 2.0)  # nose-first: the first new scan then looks
        msg.pose.orientation.w = cos(frontier.heading / 2.0)  # into the unknown instead of behind
        self.goal_pub.publish(msg)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
